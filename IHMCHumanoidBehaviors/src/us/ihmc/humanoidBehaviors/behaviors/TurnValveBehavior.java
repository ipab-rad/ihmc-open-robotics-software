package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTurnAndUnGraspTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ScriptTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnValveBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private static final boolean graspValveRim = true;
   private static final ValveType valveType = ValveType.BIG_VALVE;
   private static final Vector3d valvePinJointAxisInValveFrame = new Vector3d(1, 0, 0);
   public static final RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
   public static final double howFarToStandToTheRightOfValve = robotSideOfHandToUse.negateIfRightSide(0.13); //0.13
   public static final double howFarToStandBackFromValve = 0.64; //0.64

   public static final Vector3d graspApproachDirectionInValveFrame = new Vector3d(valvePinJointAxisInValveFrame);
   public static final double turnValveAngle = Math.toRadians(180.0);

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final WalkingControllerParameters walkingControllerParameters;

   private final ArrayList<BehaviorInterface> childBehaviors;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final GraspValveTurnAndUnGraspBehavior graspValveTurnAndUnGraspBehavior;
   private final ScriptBehavior scriptBehavior;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   private final double minDistanceForWalkingBetweenTurns = 0.1;
   private final double minYawDeltaForWalkingBetweenTurns = Math.toRadians(5.0);

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.walkingControllerParameters = walkingControllerParameters;

      childBehaviors = new ArrayList<BehaviorInterface>();
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkToLocationBehavior);
      graspValveTurnAndUnGraspBehavior = new GraspValveTurnAndUnGraspBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime,
            tippingDetectedBoolean);
      childBehaviors.add(graspValveTurnAndUnGraspBehavior);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);
      childBehaviors.add(scriptBehavior);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      walkToLocationBehavior.setDistanceThreshold(minDistanceForWalkingBetweenTurns);
      walkToLocationBehavior.setYawAngleThreshold(minYawDeltaForWalkingBetweenTurns);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   @Override
   public void doControl()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(scriptBehaviorInputPacketListener.getNewestPacket());
      }

      pipeLine.doControl();

      //      if (!currentBehavior.equals(walkToLocationBehavior))
      //      {
      //         pauseIfCapturePointErrorIsTooLarge();
      //      }
   }

   private void pauseIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue())
      {
         this.pause();
         if (DEBUG)
            SysoutTool.println("TurnValveBehavior: Tipping detected! Pausing behavior.");
      }
   }

   public void setInput(RigidBodyTransform valveTransformToWorld, double turnValveAngle)
   {
      pipeLine.submitSingleTaskStage(createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength()));

      SysoutTool.println("Not using script behavior.", DEBUG);

      double turnValveAnglePerGraspUngraspCycle = Math.toRadians(90.0);
      GraspValveTurnAndUnGraspTask graspValveTurnAndUnGraspTask = new GraspValveTurnAndUnGraspTask(graspValveTurnAndUnGraspBehavior, valveType,
            valveTransformToWorld, graspApproachDirectionInValveFrame, Axis.X, graspValveRim, turnValveAnglePerGraspUngraspCycle, yoTime);

      int numberOfGraspUngraspCycles = (int) Math.ceil(turnValveAngle / turnValveAnglePerGraspUngraspCycle);

      for (int i = 0; i < numberOfGraspUngraspCycles; i++)
      {
         pipeLine.submitSingleTaskStage(graspValveTurnAndUnGraspTask);
      }

   }

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      String scriptName = scriptBehaviorInputPacket.getScriptName();
      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();

      if (scriptName == null)
      {
         setInput(valveTransformToWorld, turnValveAngle);
      }
      else
      {
         SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);

         pipeLine.submitSingleTaskStage(createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength()));
         pipeLine.submitSingleTaskStage(new ScriptTask(scriptBehavior, scriptBehaviorInputPacket, yoTime));
      }

      hasInputBeenSet.set(true);
   }

   private WalkToLocationTask createWalkToValveTask(RigidBodyTransform valveTransformToWorld, double stepLength)
   {
      FramePose2d valvePose2d = new FramePose2d();
      valvePose2d.setPose(valveTransformToWorld);
      double valveYaw = valvePose2d.getYaw();

      FramePose2d manipulationMidFeetPose = computeDesiredMidFeetPoseForManipulation(valvePose2d);
      WalkToLocationTask ret = new WalkToLocationTask(manipulationMidFeetPose, walkToLocationBehavior, valveYaw, stepLength, yoTime);

      return ret;
   }

   private FramePose2d computeDesiredMidFeetPoseForManipulation(FramePose2d valvePose2d)
   {
      FramePose2d ret = new FramePose2d(valvePose2d);

      RigidBodyTransform yawTransform = new RigidBodyTransform();
      TransformTools.rotate(yawTransform, valvePose2d.getYaw(), Axis.Z);

      Vector3d desiredRobotPosFromValve = new Vector3d(-howFarToStandBackFromValve, -howFarToStandToTheRightOfValve, 0.0);
      Vector3d desiredRobotPosFromValveInWorld = TransformTools.getTransformedVector(desiredRobotPosFromValve, yawTransform);

      ret.setX(ret.getX() + desiredRobotPosFromValveInWorld.x);
      ret.setY(ret.getY() + desiredRobotPosFromValveInWorld.y);

      return ret;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      pipeLine.getCurrentStage().stop();
   }

   @Override
   public void enableActions()
   {
      //      SysoutTool.println("Current Child Behavior: " + currentBehavior.getName());

      SysoutTool.println("max wrist force : " + maxObservedWristForce);
      SysoutTool.println("max capture point error : " + maxObservedCapturePointError);
   }

   @Override
   public void pause()
   {
      pipeLine.getCurrentStage().pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      pipeLine.getCurrentStage().resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void finalize()
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.finalize();
      }

      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.initialize();
      }
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}