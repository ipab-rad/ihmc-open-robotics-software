package us.ihmc.atlas.ObstacleCourseTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseRampFootstepSnapperTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.unitTesting.BambooPlanType;

@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.Video})
public class AtlasObstacleCourseRampFootstepSnapperTest extends DRCObstacleCourseRampFootstepSnapperTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

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

}