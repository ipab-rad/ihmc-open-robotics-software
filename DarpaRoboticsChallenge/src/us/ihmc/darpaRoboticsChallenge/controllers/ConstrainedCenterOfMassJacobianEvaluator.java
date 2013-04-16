package us.ihmc.darpaRoboticsChallenge.controllers;

import com.google.common.primitives.Doubles;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import javax.vecmath.Vector3d;
import java.util.*;

/**
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianEvaluator implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ConstrainedCenterOfMassJacobianCalculator constrainedCenterOfMassJacobianCalculator;
   private final ConstrainedCentroidalMomentumMatrixCalculator constrainedCentroidalMomentumMatrixCalculator;
   private final ReferenceFrame centerOfMassFrame;

   private final DoubleYoVariable comJacobianConditionNumber = new DoubleYoVariable("comJacCondition", registry);
   private final DoubleYoVariable comJacobianSigmaMin = new DoubleYoVariable("comJacobianSigmaMin", registry);
   private final DoubleYoVariable constrainedComJacobianConditionNumber = new DoubleYoVariable("constrComJacCondition", registry);
   private final DoubleYoVariable constrainedComJacobianSigmaMin = new DoubleYoVariable("constrComJacobianSigmaMin", registry);
   private final YoFrameVector comVelocity = new YoFrameVector("comJacComVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector constrainedComVelocity = new YoFrameVector("constrComJacComVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable cmmConditionNumber = new DoubleYoVariable("CMMCondition", registry);
   private final DoubleYoVariable cmmSigmaMin = new DoubleYoVariable("CMMSigmaMin", registry);
   private final DoubleYoVariable constrainedCMMConditionNumber = new DoubleYoVariable("constrCMMCondition", registry);
   private final DoubleYoVariable constrainedCMMSigmaMin = new DoubleYoVariable("constrCMMSigmaMin", registry);

   private final InverseDynamicsJoint[] allJoints;
   private final DenseMatrix64F v;
   private final DenseMatrix64F vActuated;
   private final DenseMatrix64F tempCoMVelocityMatrix = new DenseMatrix64F(3, 1);
   private final Vector3d tempCoMVelocity = new Vector3d();
   private final Collection<InverseDynamicsJoint> actuatedJoints;

   public ConstrainedCenterOfMassJacobianEvaluator(FullRobotModel fullRobotModel)
   {
      Map<RigidBody, DenseMatrix64F> constrainedBodiesAndSelectionMatrices = new HashMap<RigidBody, DenseMatrix64F>();
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         constrainedBodiesAndSelectionMatrices.put(foot, selectionMatrix);
      }

      allJoints = ScrewTools.computeJointsInOrder(fullRobotModel.getElevator());
      v = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(allJoints), 1);

//      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, SpatialMotionVector.SIZE);
//      orientationSelectionMatrix.set(0, 0, 1.0);
//      orientationSelectionMatrix.set(1, 1, 1.0);
//      orientationSelectionMatrix.set(2, 2, 1.0);
//      constrainedBodiesAndSelectionMatrices.put(fullRobotModel.getPelvis(), orientationSelectionMatrix);

      actuatedJoints = new ArrayList<InverseDynamicsJoint>();
//      for (RobotSide robotSide : RobotSide.values())
//      {
//         InverseDynamicsJoint[] jointPath = ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getFoot(robotSide));
//         actuatedJoints.addAll(Arrays.asList(jointPath));
//      }
      actuatedJoints.addAll(Arrays.asList(allJoints));
      actuatedJoints.remove(fullRobotModel.getRootJoint());

      vActuated = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(actuatedJoints), 1);

      centerOfMassFrame = new CenterOfMassReferenceFrame("CoM", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());

      constrainedCenterOfMassJacobianCalculator = new ConstrainedCenterOfMassJacobianCalculator(fullRobotModel.getRootJoint(),
              constrainedBodiesAndSelectionMatrices, actuatedJoints);

      DenseMatrix64F linearMomentumSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, SpatialMotionVector.SIZE);
      linearMomentumSelectionMatrix.set(0, 3, 1.0);
      linearMomentumSelectionMatrix.set(1, 4, 1.0);
      linearMomentumSelectionMatrix.set(2, 5, 1.0);

      constrainedCentroidalMomentumMatrixCalculator = new ConstrainedCentroidalMomentumMatrixCalculator(fullRobotModel.getRootJoint(), centerOfMassFrame,
              constrainedBodiesAndSelectionMatrices, actuatedJoints, linearMomentumSelectionMatrix);
   }

   public void doControl()
   {
      centerOfMassFrame.update();

      constrainedCenterOfMassJacobianCalculator.compute();
      DenseMatrix64F centerOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getCenterOfMassJacobian();
      DenseMatrix64F constrainedCenterOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getConstrainedCenterOfMassJacobian();

      this.comJacobianConditionNumber.set(NormOps.conditionP2(centerOfMassJacobian));
      this.comJacobianSigmaMin.set(computeSmallestSingularValue(centerOfMassJacobian));
      this.constrainedComJacobianConditionNumber.set(NormOps.conditionP2(constrainedCenterOfMassJacobian));
      this.constrainedComJacobianSigmaMin.set(computeSmallestSingularValue(constrainedCenterOfMassJacobian));

      ScrewTools.packJointVelocitiesMatrix(allJoints, v);
      CommonOps.mult(centerOfMassJacobian, v, tempCoMVelocityMatrix);
      MatrixTools.denseMatrixToVector3d(tempCoMVelocityMatrix, tempCoMVelocity, 0, 0);
      comVelocity.set(tempCoMVelocity);

      ScrewTools.packJointVelocitiesMatrix(actuatedJoints, vActuated);
      CommonOps.mult(constrainedCenterOfMassJacobian, vActuated, tempCoMVelocityMatrix);
      MatrixTools.denseMatrixToVector3d(tempCoMVelocityMatrix, tempCoMVelocity, 0, 0);
      constrainedComVelocity.set(tempCoMVelocity);

      constrainedCentroidalMomentumMatrixCalculator.compute();
      DenseMatrix64F centroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getCentroidalMomentumMatrix();
      DenseMatrix64F constrainedCentroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getConstrainedCentroidalMomentumMatrix();

      this.cmmConditionNumber.set(NormOps.conditionP2(centroidalMomentumMatrix));
      this.cmmSigmaMin.set(computeSmallestSingularValue(centroidalMomentumMatrix));
      this.constrainedCMMConditionNumber.set(NormOps.conditionP2(constrainedCentroidalMomentumMatrix));
      this.constrainedCMMSigmaMin.set(computeSmallestSingularValue(constrainedCentroidalMomentumMatrix));
   }

   public void initialize()
   {
      // empty
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   private static double computeSmallestSingularValue(DenseMatrix64F A)
   {
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(A.numRows, A.numCols, false, false, true);
      svd.decompose(A);
      double[] singularValues = svd.getSingularValues();
      return Doubles.min(singularValues);
   }
}
