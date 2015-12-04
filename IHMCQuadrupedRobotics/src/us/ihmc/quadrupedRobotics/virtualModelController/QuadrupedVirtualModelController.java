package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class QuadrupedVirtualModelController
{
   private ReferenceFrame comFrame;
   private QuadrantDependentList<ReferenceFrame> toeFrame;

   private FrameVector desiredComForce;
   private FrameVector optimalComForce;
   private FrameVector desiredBodyTorque;
   private FrameVector optimalBodyTorque;
   private QuadrantDependentList<FrameVector> desiredToeForce;
   private QuadrantDependentList<FrameVector> optimalToeForce;

   private QuadrantDependentList<double[]> jointEffortLowerLimit;
   private QuadrantDependentList<double[]> jointEffortUpperLimit;
   private QuadrantDependentList<double[]> jointPositionLowerLimit;
   private QuadrantDependentList<double[]> jointPositionUpperLimit;
   private QuadrantDependentList<double[]> jointPositionLimitStiffness;
   private QuadrantDependentList<double[]> jointPositionLimitDamping;
   private double coefficientOfFriction;

   private QuadrantDependentList<OneDoFJoint[]> legJoints;
   private QuadrantDependentList<FramePoint> toePosition;
   private QuadrantDependentList<GeometricJacobian> footJacobian;
   private QuadrantDependentList<PointJacobian> toeJacobian;

   private DenseMatrix64F comWrenchMap;
   private DenseMatrix64F comWrenchMapInverse;
   private DenseMatrix64F comWrenchVector;
   private DenseMatrix64F toeForcesVector;
   private DenseMatrix64F toeForceVector;
   private QuadrantDependentList<DenseMatrix64F> legEffortVector;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap)
   {
      // initialize reference frames
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      toeFrame = referenceFrames.getFootReferenceFrames();

      // initialize desired and optimal values
      desiredComForce = new FrameVector(comFrame);
      optimalComForce = new FrameVector(comFrame);
      desiredBodyTorque = new FrameVector(comFrame);
      optimalBodyTorque = new FrameVector(comFrame);
      desiredToeForce = new QuadrantDependentList<FrameVector>();
      optimalToeForce = new QuadrantDependentList<FrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredToeForce.set(robotQuadrant, new FrameVector(comFrame));
         optimalToeForce.set(robotQuadrant, new FrameVector(comFrame));
      }

      // initialize jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      toePosition = new QuadrantDependentList<FramePoint>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      toeJacobian = new QuadrantDependentList<PointJacobian>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointNameMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         toePosition.set(robotQuadrant, new FramePoint(foot.getBodyFixedFrame()));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         toeJacobian.set(robotQuadrant, new PointJacobian());
      }

      // initialize limits
      jointEffortLowerLimit = new QuadrantDependentList<double[]>();
      jointEffortUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLowerLimit = new QuadrantDependentList<double[]>();
      jointPositionUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLimitStiffness = new QuadrantDependentList<double[]>();
      jointPositionLimitDamping = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jointEffortLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointEffortUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitStiffness.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitDamping.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         for (int i = 0; i < legJoints.get(robotQuadrant).length; i++)
         {
            jointEffortLowerLimit.get(robotQuadrant)[i] = -(Double.MAX_VALUE - 1);
            jointEffortUpperLimit.get(robotQuadrant)[i] = Double.MAX_VALUE;
            jointPositionLowerLimit.get(robotQuadrant)[i] = -(Double.MAX_VALUE - 1);
            jointPositionUpperLimit.get(robotQuadrant)[i] = Double.MAX_VALUE;
            jointPositionLimitStiffness.get(robotQuadrant)[i] = 1000;
            jointPositionLimitDamping.get(robotQuadrant)[i] = 250;
         }
      }
      coefficientOfFriction = 0.6;

      // initialize matrix terms
      comWrenchMap = new DenseMatrix64F(6, 12);
      comWrenchMapInverse = new DenseMatrix64F(12, 6);
      comWrenchVector = new DenseMatrix64F(6, 1);
      toeForcesVector = new DenseMatrix64F(12, 1);
      toeForceVector = new DenseMatrix64F(3, 1);
      legEffortVector = new QuadrantDependentList<DenseMatrix64F>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void setJointEffortLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointEffortLowerLimit.get(robotQuadrant)[index] = lower;
               jointEffortUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLowerLimit.get(robotQuadrant)[index] = lower;
               jointPositionUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitStiffness(String jointName, double stiffness)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitStiffness.get(robotQuadrant)[index] = stiffness;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitDamping(String jointName, double damping)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitDamping.get(robotQuadrant)[index] = damping;
            }
            index++;
         }
      }
   }

   public void setDesiredComForce(FrameVector comForce)
   {
      desiredComForce.setIncludingFrame(comForce);
   }

   public void setDesiredBodyTorque(FrameVector bodyTorque)
   {
      desiredBodyTorque.setIncludingFrame(bodyTorque);
   }

   public void setDesiredToeForce(RobotQuadrant robotQuadrant, FrameVector toeForce)
   {
      desiredToeForce.get(robotQuadrant).set(toeForce);
   }

   public void getOptimalComForce(FrameVector comForce)
   {
      comForce.setIncludingFrame(optimalComForce);
   }

   public void getOptimalBodyTorque(FrameVector bodyTorque)
   {
      bodyTorque.setIncludingFrame(optimalBodyTorque);
   }

   public void getOptimalToeForce(RobotQuadrant robotQuadrant, FrameVector toeForce)
   {
      toeForce.set(optimalToeForce.get(robotQuadrant));
   }

   public void compute()
   {
      // rotate desired forces and torques to center of mass frame
      desiredComForce.changeFrame(comFrame);
      desiredBodyTorque.changeFrame(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredToeForce.get(robotQuadrant).changeFrame(comFrame);
      }

      // compute toe positions and jacobians in center of mass frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         toePosition.get(robotQuadrant).setToZero(toeFrame.get(robotQuadrant));
         toePosition.get(robotQuadrant).changeFrame(comFrame);
         footJacobian.get(robotQuadrant).compute();
         toeJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), toePosition.get(robotQuadrant));
         toeJacobian.get(robotQuadrant).compute();
      }

      // compute map from toe forces to centroidal forces and torques
      comWrenchMap.zero();
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         comWrenchMap.set(0, 1 + columnOffset, -toePosition.get(robotQuadrant).getZ()); // mX row
         comWrenchMap.set(0, 2 + columnOffset, toePosition.get(robotQuadrant).getY());
         comWrenchMap.set(1, 0 + columnOffset, toePosition.get(robotQuadrant).getZ()); // mY row
         comWrenchMap.set(1, 2 + columnOffset, -toePosition.get(robotQuadrant).getX());
         comWrenchMap.set(2, 0 + columnOffset, -toePosition.get(robotQuadrant).getY()); // mZ row
         comWrenchMap.set(2, 1 + columnOffset, toePosition.get(robotQuadrant).getX());
         comWrenchMap.set(3, 0 + columnOffset, 1.0); // fX row
         comWrenchMap.set(4, 1 + columnOffset, 1.0); // fY row
         comWrenchMap.set(5, 2 + columnOffset, 1.0); // fZ row
         columnOffset += 3;
      }

      // compute centroidal wrench vector
      comWrenchVector.set(0, 0, desiredBodyTorque.getX());
      comWrenchVector.set(1, 0, desiredBodyTorque.getY());
      comWrenchVector.set(2, 0, desiredBodyTorque.getZ());
      comWrenchVector.set(3, 0, desiredComForce.getX());
      comWrenchVector.set(4, 0, desiredComForce.getY());
      comWrenchVector.set(5, 0, desiredComForce.getZ());

      // compute optimal toe forces using least squares solution
      try
      {
         CommonOps.pinv(comWrenchMap, comWrenchMapInverse);
      }
      catch (Exception e)
      {
         return;
      }
      CommonOps.mult(comWrenchMapInverse, comWrenchVector, toeForcesVector);
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         optimalToeForce.get(robotQuadrant).changeFrame(comFrame);
         optimalToeForce.get(robotQuadrant).setX(toeForcesVector.get(0 + rowOffset, 0));
         optimalToeForce.get(robotQuadrant).setY(toeForcesVector.get(1 + rowOffset, 0));
         optimalToeForce.get(robotQuadrant).setZ(toeForcesVector.get(2 + rowOffset, 0));
         rowOffset += 3;
      }

      // apply contact force limits
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double fx = optimalToeForce.get(robotQuadrant).getX();
         double fy = optimalToeForce.get(robotQuadrant).getY();
         double fz = optimalToeForce.get(robotQuadrant).getZ();
         fz = Math.max(fz, 0);
         fx = Math.min(fx, coefficientOfFriction * fz / Math.sqrt(2));
         fy = Math.min(fy, coefficientOfFriction * fz / Math.sqrt(2));
         optimalToeForce.get(robotQuadrant).setX(fx);
         optimalToeForce.get(robotQuadrant).setY(fy);
         optimalToeForce.get(robotQuadrant).setZ(fz);
      }

      // compute optimal centroidal forces and torques
      CommonOps.mult(comWrenchMap, toeForcesVector, comWrenchVector);
      optimalBodyTorque.changeFrame(comFrame);
      optimalBodyTorque.setX(comWrenchVector.get(0, 0));
      optimalBodyTorque.setY(comWrenchVector.get(1, 0));
      optimalBodyTorque.setZ(comWrenchVector.get(2, 0));
      optimalComForce.changeFrame(comFrame);
      optimalComForce.setX(comWrenchVector.get(3, 0));
      optimalComForce.setY(comWrenchVector.get(4, 0));
      optimalComForce.setZ(comWrenchVector.get(5, 0));

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = toeJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = toeJacobian.get(robotQuadrant).getFrame();
         optimalToeForce.get(robotQuadrant).changeFrame(jacobianFrame);
         toeForceVector.set(0, 0, -optimalToeForce.get(robotQuadrant).getX());
         toeForceVector.set(1, 0, -optimalToeForce.get(robotQuadrant).getY());
         toeForceVector.set(2, 0, -optimalToeForce.get(robotQuadrant).getZ());
         optimalToeForce.get(robotQuadrant).changeFrame(comFrame);
         CommonOps.multTransA(jacobianMatrix, toeForceVector, legEffortVector.get(robotQuadrant));

         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            // apply joint position and torque limits
            double tauPositionLowerLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionLowerLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauPositionUpperLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionUpperLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauEffortLowerLimit = jointEffortLowerLimit.get(robotQuadrant)[index];
            double tauEffortUpperLimit = jointEffortUpperLimit.get(robotQuadrant)[index];
            double tau = legEffortVector.get(robotQuadrant).get(index, 0);
            tau = Math.min(Math.max(tau, tauPositionLowerLimit), tauPositionUpperLimit);
            tau = Math.min(Math.max(tau, tauEffortLowerLimit), tauEffortUpperLimit);

            // update joint torques in full robot model
            joint.setTau(tau);
            index++;
         }
      }

      // TODO
      // compute quadratic cost terms (com wrench error, foot force error, force regularization)
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute min / max toe pressure constraints?
      // compute toe forces using quadratic program
   }

}