package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyIKTrajectoryBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.LockLevel;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCWholeBodyIKTrajectoryBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   
   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage();
      
      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel(), controllerCommunicator);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @EstimatedDuration(duration = 15.0)
   @Test(timeout = 45000)
   public void testConstructorAndSetInput()
   {
      WholeBodyIKTrajectoryBehavior behaviorA = new WholeBodyIKTrajectoryBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            getRobotModel(), drcBehaviorTestHelper.getSDFFullRobotModel(), drcBehaviorTestHelper.getYoTime());
      
      behaviorA.setInput(new WholeBodyTrajectoryPacket());
      assertTrue(behaviorA.hasInputBeenSet());
      
      WholeBodyIKTrajectoryBehavior behaviorB = new WholeBodyIKTrajectoryBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            getRobotModel(), drcBehaviorTestHelper.getSDFFullRobotModel(), drcBehaviorTestHelper.getYoTime());
      
      behaviorB.setInput(ControlledDoF.DOF_NONE, null, ControlledDoF.DOF_NONE, null);
      assertFalse(behaviorB.hasInputBeenSet());
      
      behaviorB.setInput(ControlledDoF.DOF_NONE, null, ControlledDoF.DOF_3P3R, new FramePose());
      assertTrue(behaviorB.hasInputBeenSet());
   }
   
   @EstimatedDuration(duration = 20.0)
   @Test(timeout = 60000)
   public void testMoveOneHandToPosition() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior = new WholeBodyIKTrajectoryBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel(), drcBehaviorTestHelper.getSDFFullRobotModel(),
            drcBehaviorTestHelper.getYoTime());
      
      FramePose desiredRight = new FramePose(worldFrame);
      desiredRight.setPosition(new Point3d(0.6, 0.3, 0.8));
      desiredRight.setOrientation(new double[]{Math.PI / 4.0, 0.0, 0.0});
      
      wholeBodyIKTrajectoryBehavior.initialize();
      wholeBodyIKTrajectoryBehavior.setInput(ControlledDoF.DOF_NONE, null, ControlledDoF.DOF_3P, desiredRight);
      
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(wholeBodyIKTrajectoryBehavior);
      assertTrue(success);
      
      ReferenceFrame rightHandFrame = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(RobotSide.RIGHT);
      FramePose finalHandPoseRight = new FramePose(rightHandFrame);
      FramePoint finalHandPosition = finalHandPoseRight.getFramePointCopy();
      finalHandPosition.add(getRobotModel().getArmControllerParameters().getWristHandCenterOffset(), 0.0, 0.0);
      finalHandPoseRight.setPosition(finalHandPosition);
      finalHandPoseRight.changeFrame(worldFrame);
      assertTrue(finalHandPoseRight.getFramePointCopy().epsilonEquals(desiredRight.getFramePointCopy(), 0.01));
   }
   
   @EstimatedDuration(duration = 25.0)
   @Test(timeout = 75000)
   public void testMoveBothHandsToPose() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior = new WholeBodyIKTrajectoryBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), getRobotModel(), drcBehaviorTestHelper.getSDFFullRobotModel(),
            drcBehaviorTestHelper.getYoTime());
      
      FramePose desiredRight = new FramePose(worldFrame);
      desiredRight.setPosition(new Point3d(0.8, -0.2, 0.9));
      desiredRight.setOrientation(new double[]{Math.PI / 4.0, 0.0, -Math.PI / 2.0});
      
      FramePose desiredLeft = new FramePose(worldFrame);
      desiredLeft.setPosition(new Point3d(0.8, 0.2, 0.9));
      desiredLeft.setOrientation(new double[]{-Math.PI / 4.0, 0.0, Math.PI / 2.0});
      
      wholeBodyIKTrajectoryBehavior.setLockLevel(LockLevel.LOCK_LEGS);
      wholeBodyIKTrajectoryBehavior.initialize();
      wholeBodyIKTrajectoryBehavior.setInput(ControlledDoF.DOF_3P3R, desiredLeft, ControlledDoF.DOF_3P3R, desiredRight);
      
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(wholeBodyIKTrajectoryBehavior);
      assertTrue(success);
      
      ReferenceFrame rightHandFrame = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(RobotSide.RIGHT);
      ReferenceFrame leftHandFrame = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(RobotSide.LEFT);
      FramePose finalHandPoseRight = new FramePose(rightHandFrame);
      FramePose finalHandPoseLeft = new FramePose(leftHandFrame);
      Vector3d offset = new Vector3d(getRobotModel().getArmControllerParameters().getWristHandCenterOffset(), 0.0, 0.0);
      finalHandPoseLeft.setPosition(offset);
      finalHandPoseRight.setPosition(offset);
      finalHandPoseLeft.changeFrame(worldFrame);
      finalHandPoseRight.changeFrame(worldFrame);
      
      assertTrue(finalHandPoseRight.getFramePointCopy().epsilonEquals(desiredRight.getFramePointCopy(), 0.1));
      assertTrue(finalHandPoseLeft.getFramePointCopy().epsilonEquals(desiredLeft.getFramePointCopy(), 0.1));
      assertTrue(finalHandPoseRight.getFrameOrientationCopy().epsilonEquals(desiredRight.getFrameOrientationCopy(), 0.3));
      assertTrue(finalHandPoseLeft.getFrameOrientationCopy().epsilonEquals(desiredLeft.getFrameOrientationCopy(), 0.3));
   }
}