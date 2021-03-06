package us.ihmc.commonWalkingControlModules.packetProducers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandPoseStatusProducer
{
   private final HumanoidGlobalDataProducer objectCommunicator;

   public HandPoseStatusProducer(HumanoidGlobalDataProducer objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }

   public void sendStatus(Point3d currentPosePosition, Quat4d currentPoseOrientationInWorldFrame, Point3d desiredPosePosition,
         Quat4d desiredPoseOrientationInWorldFrame, RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = new HandPoseStatus(currentPosePosition, currentPoseOrientationInWorldFrame, desiredPosePosition,
            desiredPoseOrientationInWorldFrame, robotSide);
      objectCommunicator.queueDataToSend(handPoseStatus);
   }

   public void sendCompletedStatus(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = HandPoseStatus.createPositionIsReachedPacket(robotSide);
      objectCommunicator.queueDataToSend(handPoseStatus);
   }
   
   public void sendStartedStatus(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = HandPoseStatus.createHandPoseIsStarted(robotSide);
      objectCommunicator.queueDataToSend(handPoseStatus);
   }

   public void sendTimeOutIsReachedStatus(RobotSide robotSide)
   {
      HandPoseStatus handPoseStatus = HandPoseStatus.createTimeOutIsReachedPacket(robotSide);
      objectCommunicator.queueDataToSend(handPoseStatus);
   }
}
