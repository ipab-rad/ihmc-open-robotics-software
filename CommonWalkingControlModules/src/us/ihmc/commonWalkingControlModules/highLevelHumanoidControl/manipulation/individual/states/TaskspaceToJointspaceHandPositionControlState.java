package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.RateLimitedYoVariable;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;

public class TaskspaceToJointspaceHandPositionControlState extends TrajectoryBasedTaskspaceHandControlState
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private PoseTrajectoryGenerator poseTrajectoryGenerator;

   private final FramePose desiredPose = new FramePose();
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;

   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers;
   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations;
   private final LinkedHashMap<OneDoFJoint, AlphaFilteredYoVariable> filteredFeedForwardAccelerations;
   private final DoubleYoVariable maxAcceleration;

   private final DoubleYoVariable percentOfTrajectoryWithOrientationBeingControlled;
   private final DoubleYoVariable startTimeInStateToIgnoreOrientation;
   private final DoubleYoVariable endTimeInStateToIgnoreOrientation;
   private final DoubleYoVariable currentOrientationControlFactor;
   private final DoubleYoVariable activeTrajectoryTime;

   private final DoubleYoVariable doneTrajectoryTime;
   private final DoubleYoVariable holdPositionDuration;

   private final boolean doPositionControl;

   private final DenseMatrix64F identityScaledWithOrientationControlFactor = CommonOps.identity(SpatialMotionVector.SIZE);
   private final DenseMatrix64F selectionMatrixWithReducedAngularControl = CommonOps.identity(SpatialMotionVector.SIZE);

   private final double dt;
   private final OneDoFJoint[] oneDoFJoints;
   private final boolean[] doIntegrateDesiredAccelerations;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable initialTime;
   private final DoubleYoVariable currentTimeInState;

   public static TaskspaceToJointspaceHandPositionControlState createControlStateForForceControlledJoints(String namePrefix,
         MomentumBasedController momentumBasedController, RigidBody base, RigidBody endEffector, double controlDT, YoPIDGains gains, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry)
   {
      return new TaskspaceToJointspaceHandPositionControlState(namePrefix, momentumBasedController, base, endEffector, false, controlDT, gains, yoTime, parentRegistry);
   }

   public static TaskspaceToJointspaceHandPositionControlState createControlStateForPositionControlledJoints(String namePrefix, RigidBody base,
         RigidBody endEffector, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      return new TaskspaceToJointspaceHandPositionControlState(namePrefix, null, base, endEffector, true, Double.NaN, null, yoTime, parentRegistry);
   }

   public static TaskspaceToJointspaceHandPositionControlState createControlStateForPositionControlledJoints(String namePrefix,
         MomentumBasedController momentumBasedController, RigidBody base, RigidBody endEffector, double controlDT, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry)
   {
      return new TaskspaceToJointspaceHandPositionControlState(namePrefix, momentumBasedController, base, endEffector, true, controlDT, null, yoTime, parentRegistry);
   }

   private TaskspaceToJointspaceHandPositionControlState(String namePrefix, MomentumBasedController momentumBasedController, RigidBody base,
         RigidBody endEffector, boolean doPositionControl, double controlDT, YoPIDGains gains, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, HandControlState.TASK_SPACE_POSITION, momentumBasedController, -1, base, endEffector, parentRegistry);

      this.yoTime = yoTime;

      initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
      currentTimeInState = new DoubleYoVariable(namePrefix + "CurrentTimeInState", registry);

      doneTrajectoryTime = new DoubleYoVariable(namePrefix + "DoneTrajectoryTime", registry);
      holdPositionDuration = new DoubleYoVariable(namePrefix + "HoldPositionDuration", registry);

      this.doPositionControl = doPositionControl;

      percentOfTrajectoryWithOrientationBeingControlled = new DoubleYoVariable(namePrefix + "PercentOfTrajectoryWithOrientationBeingControlled", registry);
      percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);

      activeTrajectoryTime = new DoubleYoVariable(namePrefix + "ActiveTrajectoryTime", registry);
      activeTrajectoryTime.set(Double.NaN);

      startTimeInStateToIgnoreOrientation = new DoubleYoVariable(namePrefix + "StartTimeInStateToIgnoreOrientation", registry);
      startTimeInStateToIgnoreOrientation.set(Double.NaN);

      endTimeInStateToIgnoreOrientation = new DoubleYoVariable(namePrefix + "EndTimeInStateToIgnoreOrientation", registry);
      endTimeInStateToIgnoreOrientation.set(Double.NaN);

      currentOrientationControlFactor = new DoubleYoVariable(namePrefix + "CurrentOrientationControlFactor", registry);
      currentOrientationControlFactor.set(Double.NaN);

      dt = controlDT;

      oneDoFJoints = ScrewTools.createOneDoFJointPath(base, endEffector);

      doIntegrateDesiredAccelerations = new boolean[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         doIntegrateDesiredAccelerations[i] = joint.getIntegrateDesiredAccelerations();
      }

      if (doPositionControl)
      {
         maxAcceleration = null;
         pidControllers = null;
         rateLimitedAccelerations = null;

         if (momentumBasedController != null)
         {
            filteredFeedForwardAccelerations = new LinkedHashMap<OneDoFJoint, AlphaFilteredYoVariable>();
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, dt);

            for (OneDoFJoint joint : oneDoFJoints)
            {
               AlphaFilteredYoVariable filteredFeedForwardAcceleration = new AlphaFilteredYoVariable("qdd_ff_filt_" + joint.getName(), registry, alpha);
               filteredFeedForwardAccelerations.put(joint, filteredFeedForwardAcceleration);
            }
         }
         else
         {
            filteredFeedForwardAccelerations = null;
         }
      }
      else // Force control at the joints
      {
         filteredFeedForwardAccelerations = null;

         maxAcceleration = gains.getYoMaximumAcceleration();

         pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
         rateLimitedAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();

         for (OneDoFJoint joint : oneDoFJoints)
         {
            String suffix = FormattingTools.lowerCaseFirstLetter(joint.getName());
            PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
            pidControllers.put(joint, pidController);

            RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(suffix + "Acceleration", registry, gains.getYoMaximumJerk(), dt);
            rateLimitedAccelerations.put(joint, rateLimitedAcceleration);
         }
      }
   }

   /** Either {@link #initializeWithCurrentJointAngles()} or {@link #initializeWithDesiredJointAngles()} needs to be called before {@link #doAction()} when this class is used as a standalone controller. */
   public void initializeWithDesiredJointAngles()
   {
      taskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
      doTransitionIntoAction();
   }

   /** Either {@link #initializeWithCurrentJointAngles()} or {@link #initializeWithDesiredJointAngles()} needs to be called before {@link #doAction()} when this class is used as a standalone controller. */
   public void initializeWithCurrentJointAngles()
   {
      taskspaceToJointspaceCalculator.initializeFromCurrentJointAngles();
      doTransitionIntoAction();
   }

   @Override
   public void doAction()
   {
      currentTimeInState.set(getTimeInCurrentState());

      if (poseTrajectoryGenerator.isDone())
         recordDoneTrajectoryTime();

      poseTrajectoryGenerator.compute(currentTimeInState.getDoubleValue());

      poseTrajectoryGenerator.get(desiredPose);
      poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();
      desiredVelocity.changeFrame(controlFrame);
      desiredAngularVelocity.changeFrame(controlFrame);

      decayAngularControl(currentTimeInState.getDoubleValue());

      taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
      taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
      taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
      taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);

      if (momentumBasedController != null)
      {
         if (doPositionControl)
         {
            for (int i = 0; i < oneDoFJoints.length; i++)
            {
               OneDoFJoint joint = oneDoFJoints[i];
               AlphaFilteredYoVariable filteredFeedForwardAcceleration = filteredFeedForwardAccelerations.get(joint);
               filteredFeedForwardAcceleration.update(joint.getQddDesired());
               double desiredAcceleration = filteredFeedForwardAcceleration.getDoubleValue();

               momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
            }
         }
         else
         {
            for (int i = 0; i < oneDoFJoints.length; i++)
            {
               OneDoFJoint joint = oneDoFJoints[i];

               double q = joint.getQ();
               double qd = joint.getQd();
               double qDesired = joint.getqDesired();
               double qdDesired = joint.getQdDesired();

               RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);

               // TODO No feed-forward acceleration yet, needs to be implemented.
               double feedforwardAcceleration = 0.0;

               PIDController pidController = pidControllers.get(joint);
               double desiredAcceleration = feedforwardAcceleration + pidController.computeForAngles(q, qDesired, qd, qdDesired, dt);

               desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAcceleration.getDoubleValue());
               rateLimitedAcceleration.update(desiredAcceleration);
               desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

               momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
            }
         }
      }
   }

   @Override
   public double getTimeInCurrentState()
   {
      return yoTime.getDoubleValue() - initialTime.getDoubleValue();
   }

   private void decayAngularControl(double time)
   {
      if (activeTrajectoryTime.isNaN() || percentOfTrajectoryWithOrientationBeingControlled.isNaN())
         return;

      if (time < startTimeInStateToIgnoreOrientation.getDoubleValue())
      {
         double alpha = 1.0 - time / startTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
         alpha *= alpha;
         currentOrientationControlFactor.set(alpha);
         applyAlphaFactorForOrientationControl(alpha);
      }
      else if (time > endTimeInStateToIgnoreOrientation.getDoubleValue())
      {
         double alpha = time - endTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha /= activeTrajectoryTime.getDoubleValue() - endTimeInStateToIgnoreOrientation.getDoubleValue();
         alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
         alpha *= alpha;
         currentOrientationControlFactor.set(alpha);
         applyAlphaFactorForOrientationControl(alpha);
      }
      else
      {
         setupSelectionMatrixForLinearControlOnly();
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialTime.set(yoTime.getDoubleValue());
      currentTimeInState.set(0.0);

      poseTrajectoryGenerator.showVisualization();
      poseTrajectoryGenerator.initialize();
      doneTrajectoryTime.set(Double.NaN);

      saveDoAccelerationIntegration();

      if (doPositionControl)
         enablePositionControl();

      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

      selectionMatrixWithReducedAngularControl.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(identityScaledWithOrientationControlFactor);
      CommonOps.setIdentity(selectionMatrixWithReducedAngularControl);

      if (activeTrajectoryTime.isNaN() || percentOfTrajectoryWithOrientationBeingControlled.isNaN())
      {
         startTimeInStateToIgnoreOrientation.set(Double.NaN);
         endTimeInStateToIgnoreOrientation.set(Double.NaN);
         currentOrientationControlFactor.set(Double.NaN);
      }
      else if (MathTools.epsilonEquals(percentOfTrajectoryWithOrientationBeingControlled.getDoubleValue(), 0.0, 1.0e-2))
      {
         activeTrajectoryTime.set(Double.NaN);
         percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);
         startTimeInStateToIgnoreOrientation.set(Double.NaN);
         endTimeInStateToIgnoreOrientation.set(Double.NaN);
         currentOrientationControlFactor.set(0.0);
         setupSelectionMatrixForLinearControlOnly();
      }
      else
      {
         double timeToDecayOrientationControl = 0.5 * percentOfTrajectoryWithOrientationBeingControlled.getDoubleValue() * activeTrajectoryTime.getDoubleValue();
         startTimeInStateToIgnoreOrientation.set(timeToDecayOrientationControl);
         endTimeInStateToIgnoreOrientation.set(activeTrajectoryTime.getDoubleValue() - timeToDecayOrientationControl);
      }
   }

   private void setupSelectionMatrixForLinearControlOnly()
   {
      applyAlphaFactorForOrientationControl(0.0);
   }

   private void applyAlphaFactorForOrientationControl(double alpha)
   {
      CommonOps.setIdentity(identityScaledWithOrientationControlFactor);
      for (int i = 0; i < 3; i++)
         identityScaledWithOrientationControlFactor.set(i, i, alpha);

      selectionMatrixWithReducedAngularControl.reshape(selectionMatrix.getNumRows(), SpatialMotionVector.SIZE);
      CommonOps.mult(selectionMatrix, identityScaledWithOrientationControlFactor, selectionMatrixWithReducedAngularControl);

      MatrixTools.removeZeroRows(selectionMatrixWithReducedAngularControl, 1.0e-12);

      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrixWithReducedAngularControl);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      holdPositionDuration.set(0.0);

      if (doPositionControl)
         disablePositionControl();

      startTimeInStateToIgnoreOrientation.set(Double.NaN);
      endTimeInStateToIgnoreOrientation.set(Double.NaN);
      currentOrientationControlFactor.set(Double.NaN);
   }

   private void saveDoAccelerationIntegration()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         doIntegrateDesiredAccelerations[i] = joint.getIntegrateDesiredAccelerations();
      }
   }

   private void enablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);
      }
   }

   private void disablePositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccelerations[i]);
         joint.setUnderPositionControl(false);
      }
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
         return false;

      return getTimeInCurrentState() > doneTrajectoryTime.getDoubleValue() + holdPositionDuration.getDoubleValue();
   }

   private void recordDoneTrajectoryTime()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
      {
         doneTrajectoryTime.set(getTimeInCurrentState());
      }
   }

   @Override
   public void setHoldPositionDuration(double holdPositionDuration)
   {
      this.holdPositionDuration.set(holdPositionDuration);
   }

   @Override
   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
   {
      setTrajectoryWithAngularControlQuality(poseTrajectoryGenerator, Double.NaN, Double.NaN);
   }

   @Override
   public void setTrajectoryWithAngularControlQuality(PoseTrajectoryGenerator poseTrajectoryGenerator, double percentOfTrajectoryWithOrientationBeingControlled, double trajectoryTime)
   {
      this.poseTrajectoryGenerator = poseTrajectoryGenerator;
      percentOfTrajectoryWithOrientationBeingControlled = MathTools.clipToMinMax(percentOfTrajectoryWithOrientationBeingControlled, 0.0, 1.0);
      if (MathTools.epsilonEquals(percentOfTrajectoryWithOrientationBeingControlled, 1.0, 0.01))
      {
         this.percentOfTrajectoryWithOrientationBeingControlled.set(Double.NaN);
         this.activeTrajectoryTime.set(Double.NaN);
      }
      else
      {
         this.percentOfTrajectoryWithOrientationBeingControlled.set(percentOfTrajectoryWithOrientationBeingControlled);
         this.activeTrajectoryTime.set(trajectoryTime);
      }
   }

   @Override
   public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule)
   {
   }

   @Override
   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
   {
      this.taskspaceToJointspaceCalculator = taskspaceToJointspaceCalculator;
   }

   @Override
   public FramePose getDesiredPose()
   {
      taskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredPose, desiredPose.getReferenceFrame());
      return desiredPose;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return desiredPose.getReferenceFrame();
   }
}