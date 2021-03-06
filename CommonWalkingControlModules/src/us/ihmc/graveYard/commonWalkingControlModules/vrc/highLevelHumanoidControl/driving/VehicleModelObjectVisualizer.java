package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;



public class VehicleModelObjectVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsList yoGraphicsList;
   private final double objectFrameScale = 0.2;
   private final double vehicleFrameScale = 1.0;

   public VehicleModelObjectVisualizer(ReferenceFrame vehicleFrame, VehicleModelObjects vehicleModelObjects,
                                YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      yoGraphicsList = new YoGraphicsList("vehicleObjects");

      for (VehicleObject vehicleObject : VehicleObject.values())
      {

         FramePose framePose = vehicleModelObjects.getFramePose(vehicleFrame, vehicleObject);

         String objectName = FormattingTools.underscoredToCamelCase(vehicleObject.toString(), false);
         ReferenceFrame objectFrame = new PoseReferenceFrame(objectName, framePose);
         objectFrame.update();

         YoGraphicReferenceFrame dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(objectFrame, registry, objectFrameScale);
         yoGraphicsList.add(dynamicGraphicReferenceFrame);
      }

      YoGraphicReferenceFrame vehicleFrameViz = new YoGraphicReferenceFrame(vehicleFrame, registry, vehicleFrameScale);
      yoGraphicsList.add(vehicleFrameViz);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
      {
         yoGraphic.update();
      }
   }

   public void setVisible(boolean visible)
   {
      yoGraphicsList.setVisible(visible);
   }
}
