package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCWholeBodyInverseKinematicBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;


@DeployableTestClass(targets = {TestPlanTarget.Exclude})
public class AtlasWholeBodyInverseKinematicBehaviorTest extends DRCWholeBodyInverseKinematicBehaviorTest
{
	private final AtlasRobotModel robotModel;
	
	public AtlasWholeBodyInverseKinematicBehaviorTest() 
	{
		robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
	}

	@Override
	public DRCRobotModel getRobotModel() {
	      return robotModel;
	}

	@Override
	public String getSimpleRobotName()
	{
		return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
	}
	
	/**
	 * Memory hog. Crashing a lot.
	 */
	@Override
	@DeployableTestMethod(estimatedDuration = 90.0)
   @Test(timeout = 300000)
	public void testRandomRightHandPose() throws SimulationExceededMaximumTimeException
	{
	   super.testRandomRightHandPose();
	}
	
	/**
    * Memory hog. Crashing a lot.
    */
   @Override
   @DeployableTestMethod(estimatedDuration = 90.0)
   @Test(timeout = 300000)
	public void testWholeBodyInverseKinematicsMoveToPoseAcheivedInJointSpace() throws SimulationExceededMaximumTimeException
	{
	   super.testWholeBodyInverseKinematicsMoveToPoseAcheivedInJointSpace();
	}
}
