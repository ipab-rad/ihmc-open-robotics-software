package us.ihmc.atlas.ObstacleCourseTests;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseWholeBodyTrajectoryTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

@BambooPlan(planType = { BambooPlanType.Slow, BambooPlanType.VideoA })
public class AtlasObstacleCourseWholeBodyTrajectoryTest extends DRCObstacleCourseWholeBodyTrajectoryTest
{
   private final DRCRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();
   private final int numberOfArmJoints = robotModel.getJointMap().getArmJointNames().length;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   private double[] ensureJointAnglesSize(double[] desiredJointAngles)
   {
      double[] ret;
      double[] src = desiredJointAngles;
      if (numberOfArmJoints > src.length)
      {
         ret = new double[numberOfArmJoints];
         System.arraycopy(src, 0, ret, 0, src.length);
      }
      else
         ret = src;
      return ret;
   }

   private double[][] generateArmTrajectory(RobotSide robotSide, int numOfWaypoints)
   {
      // Put the arm down
      //waypoints isnt used in this function
      double halfPi = Math.PI / 2.0;
      double[] armDown2 = ensureJointAnglesSize(new double[] { 0.0, -0.4, halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayUp = ensureJointAnglesSize(new double[] { 0.0, -halfPi, halfPi / 2.0, -halfPi / 2.0 });
      double[] armUp1 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, halfPi / 2.0, 0.0 });
      double[] armUp2 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, -halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayDown = ensureJointAnglesSize(new double[] { 0.0, -halfPi, -halfPi / 2.0, -halfPi / 2.0 });
      double[] armDown1 = ensureJointAnglesSize(new double[] { 0.0, -0.4, -halfPi / 2.0, 0.0 });
      double[][] armFlyingSequence = new double[numberOfArmJoints][numOfWaypoints];

      for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
      {
         for (int poseIndex = 0; poseIndex < numOfWaypoints; poseIndex++)
         {
            double desiredJointAngle;
            switch (poseIndex % 6)
            {
            case 0:
               desiredJointAngle = armDown1[jointIndex];
               break;
            case 1:
               desiredJointAngle = armDown2[jointIndex];
               break;
            case 2:
               desiredJointAngle = armIntermediateOnWayUp[jointIndex];
               break;
            case 3:
               desiredJointAngle = armUp1[jointIndex];
               break;
            case 4:
               desiredJointAngle = armUp2[jointIndex];
               break;
            case 5:
               desiredJointAngle = armIntermediateOnWayDown[jointIndex];
               break;
            default:
               throw new RuntimeException("Should not get there!");
            }

            armFlyingSequence[jointIndex][poseIndex] = desiredJointAngle;
         }
      }
      return armFlyingSequence;
   }

   @Override
   public WholeBodyTrajectoryPacket getRobotSpecificWholeBodyTrajectoryPacket(double trajectoryTime)
   {
      int waypoints = 8;
      WholeBodyTrajectoryPacket packet = new WholeBodyTrajectoryPacket(waypoints, numberOfArmJoints);
      try
      {
         for (int w = 0; w < waypoints; w++)
         {
            packet.timeAtWaypoint[w] = trajectoryTime/waypoints;
            if (w > 0)
               packet.timeAtWaypoint[w] += packet.timeAtWaypoint[w - 1];
            
            packet.pelvisWorldPosition[w] = new Point3d(-0.05, 0.0, 0.2 * (w % 2) + 0.6);

            Quat4d rotation = new Quat4d();
            rotation.set(new AxisAngle4d(0, 1, 0, 0.2 * ((w % 2) * 2 - 1)));

            packet.chestWorldOrientation[w] = rotation;
            packet.pelvisWorldOrientation[w] = new Quat4d();
         }

         packet.allocateArmTrajectory(RobotSide.LEFT);
         packet.allocateArmTrajectory(RobotSide.RIGHT);

         packet.leftArmJointAngle = generateArmTrajectory(RobotSide.LEFT, waypoints);
         packet.rightArmJointAngle = generateArmTrajectory(RobotSide.RIGHT, waypoints);

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      return packet;
   }

   @Override
   public WholeBodyIkSolver createRobotSpecificWholeBodyIKSolver()
   {
      return new AtlasWholeBodyIK(robotModel);
   }

}