package us.ihmc.atlas;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.MultiContactDRCRobotInitialSetup;
import us.ihmc.atlas.initialSetup.PushUpDRCRobotInitialSetup;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasRobotMultiContactControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MultiContactTestHumanoidControllerFactory;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.MultiContactTestEnvironment;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.inputdevices.MidiSliderBoard;

public class AtlasMultiContact
{
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS;

   private final DRCSimulationFactory drcSimulation;
   private final MultiContactTestEnvironment environment;
   private final SimulationConstructionSet simulationConstructionSet;

   public static enum MultiContactTask
   {
      DEFAULT, PUSHUP
   };

   public AtlasMultiContact(AtlasRobotModel robotModel, DRCGuiInitialSetup guiInitialSetup, MultiContactTask task)
   {
      DRCSCSInitialSetup scsInitialSetup;

      RobotSide[] footContactSides;
      RobotSide[] handContactSides;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup;
      switch (task)
      {
      case PUSHUP:
         footContactSides = RobotSide.values;
         handContactSides = RobotSide.values;
         robotInitialSetup = new PushUpDRCRobotInitialSetup();
         break;
      default:
         footContactSides = RobotSide.values;
         handContactSides = new RobotSide[] { RobotSide.LEFT };
         robotInitialSetup = new MultiContactDRCRobotInitialSetup();
         break;

      }

      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters(false, false);
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();

      environment = new MultiContactTestEnvironment(robotInitialSetup, robotModel, footContactSides, handContactSides,
            AtlasContactPointParameters.invisibleContactablePlaneHandContactPointTransforms);
      scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(robotModel.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();

      WalkingControllerParameters controllerParameters = new AtlasRobotMultiContactControllerParameters()
      {
         public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
         {
            Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();
            EnumMap<ArmJointName, Double> defaultArmPosition = MultiContactDRCRobotInitialSetup.getDefaultArmPositionForMultiContactSimulation().get(robotSide);

            for (ArmJointName armJointName : defaultArmPosition.keySet())
            {
               double position = defaultArmPosition.get(armJointName);
               OneDoFJoint joint = fullRobotModel.getArmJoint(robotSide, armJointName);
               jointPositions.put(joint, position);
            }

            return jointPositions;
         }
      };

      MultiContactTestHumanoidControllerFactory highLevelHumanoidControllerFactory = new MultiContactTestHumanoidControllerFactory(contactableBodiesFactory,
            footContactSides, handContactSides, controllerParameters, armControllerParameters);
      DRCRobotMomentumBasedControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(contactableBodiesFactory, highLevelHumanoidControllerFactory,
            DRCConfigParameters.contactTresholdForceForSCS, sensorInformation.getFeetForceSensorNames());

      drcSimulation = new DRCSimulationFactory(robotModel, controllerFactory, environment.getTerrainObject(), robotInitialSetup, scsInitialSetup,
            guiInitialSetup, null);

      simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      MidiSliderBoard sliderBoard = new MidiSliderBoard(simulationConstructionSet);
      sliderBoard.setSlider(1, "desiredCoMX", simulationConstructionSet, -0.2, 0.2);
      sliderBoard.setSlider(2, "desiredCoMY", simulationConstructionSet, -0.2, 0.2);
      sliderBoard.setSlider(3, "desiredCoMZ", simulationConstructionSet, -0.5, 1.2);
      sliderBoard.setSlider(4, "desiredPelvisYaw", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setSlider(5, "desiredPelvisPitch", simulationConstructionSet, -Math.PI, Math.PI);
      sliderBoard.setSlider(6, "desiredPelvisRoll", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(1, "desiredChestOrientationYaw", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(2, "desiredChestOrientationPitch", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(3, "desiredChestOrientationRoll", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);

      simulationConstructionSet.setCameraPosition(6.0, -2.0, 4.5);
      simulationConstructionSet.setCameraFix(-0.44, -0.17, 0.75);

      drcSimulation.start();
   }

   public DRCSimulationFactory getDRCSimulation()
   {
      return drcSimulation;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public static void main(String[] args) throws JSAPException
   {

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, true);

      new AtlasMultiContact(new AtlasRobotModel(ATLAS_ROBOT_VERSION, false, false), guiInitialSetup, MultiContactTask.DEFAULT);
   }

}
