package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceTexture;
import us.ihmc.utilities.math.geometry.Box3d;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import com.yobotics.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCWallWorldEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   public DRCWallWorldEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      combinedTerrainObject.addTerrainObject(setupWallSlightlyInFront("WallInFront"));
   }
   
   private static CombinedTerrainObject3D setupWallSlightlyInFront(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      double xStart = 0.6;
      double wallWidth = 10.0;
      
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      combinedTerrainObject.addBox(xStart, -wallWidth/2.0, xStart + 0.1, wallWidth/2.0, 0.5, 1.8, appearance);
      return combinedTerrainObject;
   }

   private static CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-0.18, -10.0, 0.18, 10.0, -0.05, 0.0, YoAppearance.Green());
      
//    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
//      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
//
//      Transform3D location = new Transform3D();
//      location.setTranslation(new Vector3d(0, 0, -0.5));
//
//      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), texture);
//      combinedTerrainObject.addTerrainObject(newBox);
//      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 200, 200, 0.75), YoAppearance.DarkGray());
//      combinedTerrainObject.addTerrainObject(newBox2);

      return combinedTerrainObject;
   }
   
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub
      
   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
      // TODO Auto-generated method stub
      
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub
      
   }

}
