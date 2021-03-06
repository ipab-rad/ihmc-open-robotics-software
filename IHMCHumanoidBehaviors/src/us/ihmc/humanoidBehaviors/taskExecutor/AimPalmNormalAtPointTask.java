package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotSide;

public class AimPalmNormalAtPointTask extends BehaviorTask
{
   private final HandPoseBehavior handPoseBehavior;

   private final RobotSide robotSide;
   private final FramePoint targetToAimAt;
   private final FullHumanoidRobotModel fullRobotModel;
   private final double trajectoryTime;

   public AimPalmNormalAtPointTask(RobotSide robotSide, FramePoint targetToAimAt, FullHumanoidRobotModel fullRobotModel, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior,
         double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;
      this.targetToAimAt = targetToAimAt;
      this.trajectoryTime = trajectoryTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseBehavior.aimPalmNormalAtPoint(robotSide, targetToAimAt, fullRobotModel, trajectoryTime);
   }
}
