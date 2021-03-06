package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

public class FlatGroundTerrainObject extends FlatGroundProfile implements TerrainObject3D
{
   private final Graphics3DObject groundGraphics;
   
   public FlatGroundTerrainObject()
   {
      groundGraphics = new Graphics3DObject();
      groundGraphics.translate(0.0, 0.0, -0.03);
      groundGraphics.addCube(20.0, 20.0, 0.03);
   }
   
   public Graphics3DObject getLinkGraphics()
   {
      return groundGraphics;
   }

}
