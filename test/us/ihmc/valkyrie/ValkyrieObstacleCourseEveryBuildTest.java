package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseEveryBuildTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.ExcludedTest;

@ExcludedTest() // Crashes Bamboo
public class ValkyrieObstacleCourseEveryBuildTest extends DRCObstacleCourseEveryBuildTest
{

   private final DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

}