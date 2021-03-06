package us.ihmc.atlas.ObstacleCourseTests;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseWholeBodyTrajectoryTest;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;

@DeployableTestClass(targets = {TestPlanTarget.Slow})
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

   @Override
   @DeployableTestMethod(estimatedDuration = 18.8, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 94000)
   public void testArmMovementsWithTrajectoryPacket() throws SimulationExceededMaximumTimeException
   {
      super.testArmMovementsWithTrajectoryPacket();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 78.2)
   @Test(timeout = 390000)
   public void testChestControlWithTrajectoryPacket() throws SimulationExceededMaximumTimeException
   {
      super.testChestControlWithTrajectoryPacket();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 50.0, targets = TestPlanTarget.Flaky)
   @Test(timeout = 300000)
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 23.3)
   @Test(timeout = 120000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSeconds();
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

   private ArmJointTrajectoryPacket generateArmTrajectory(RobotSide robotSide, int numOfWaypoints)
   {
      // Put the arm down
      // waypoints isnt used in this function
      double halfPi = Math.PI / 2.0;
      double[] armDown2 = ensureJointAnglesSize(new double[] {0.0, -0.4, halfPi / 2.0, 0.0});
      double[] armIntermediateOnWayUp = ensureJointAnglesSize(new double[] {0.0, -halfPi, halfPi / 2.0, -halfPi / 2.0});
      double[] armUp1 = ensureJointAnglesSize(new double[] {0.0, -1.5 * halfPi, halfPi / 2.0, 0.0});
      double[] armUp2 = ensureJointAnglesSize(new double[] {0.0, -1.5 * halfPi, -halfPi / 2.0, 0.0});
      double[] armIntermediateOnWayDown = ensureJointAnglesSize(new double[] {0.0, -halfPi, -halfPi / 2.0, -halfPi / 2.0});
      double[] armDown1 = ensureJointAnglesSize(new double[] {0.0, -0.4, -halfPi / 2.0, 0.0});
      ArmJointTrajectoryPacket armFlyingSequence = new ArmJointTrajectoryPacket(robotSide, numOfWaypoints, numberOfArmJoints);
      for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
      {
         for (int poseIndex = 0; poseIndex < numOfWaypoints; poseIndex++)
         {
            double desiredJointAngle;
            switch (poseIndex % 6)
            {
               case 0 :
                  desiredJointAngle = armDown1[jointIndex];

                  break;

               case 1 :
                  desiredJointAngle = armDown2[jointIndex];

                  break;

               case 2 :
                  desiredJointAngle = armIntermediateOnWayUp[jointIndex];

                  break;

               case 3 :
                  desiredJointAngle = armUp1[jointIndex];

                  break;

               case 4 :
                  desiredJointAngle = armUp2[jointIndex];

                  break;

               case 5 :
                  desiredJointAngle = armIntermediateOnWayDown[jointIndex];

                  break;

               default :
                  throw new RuntimeException("Should not get there!");
            }

            armFlyingSequence.trajectoryPoints[poseIndex].positions[jointIndex] = desiredJointAngle;
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
            packet.timeAtWaypoint[w] = trajectoryTime / waypoints;
            if (w > 0)
               packet.timeAtWaypoint[w] += packet.timeAtWaypoint[w - 1];
            packet.pelvisWorldPosition[w] = new Point3d(-0.05, 0.0, 0.2 * (w % 2) + 0.6);
            Quat4d rotation = new Quat4d();
            rotation.set(new AxisAngle4d(0, 1, 0, 0.2 * ((w % 2) * 2 - 1)));
            packet.chestWorldOrientation[w] = rotation;
            packet.pelvisWorldOrientation[w] = new Quat4d();
         }

         packet.allocateArmTrajectories();
         packet.leftArmTrajectory = generateArmTrajectory(RobotSide.LEFT, waypoints);
         packet.rightArmTrajectory = generateArmTrajectory(RobotSide.RIGHT, waypoints);
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
