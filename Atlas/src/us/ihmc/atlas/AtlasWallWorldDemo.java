package us.ihmc.atlas;

import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallWorldEnvironment;

import com.martiansoftware.jsap.JSAPException;

public class AtlasWallWorldDemo
{

   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
      CommonAvatarEnvironmentInterface environment = new DRCWallWorldEnvironment(-10.0, 10.0);

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();

//      double stepHeight = 0.2;
//      CommonAvatarEnvironmentInterface environment = new BigStepUpWithHandPlatformEnvironment(stepHeight);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);

      simulationStarter.startSimulation(true, true);
   }
}
