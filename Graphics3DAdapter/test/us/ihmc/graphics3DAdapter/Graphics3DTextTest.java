package us.ihmc.graphics3DAdapter;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType={BambooPlanType.UI})
public class Graphics3DTextTest implements Graphics3DFrameListener
{
   int counter = 0;
   Graphics3DAddExtrusionInstruction instruction;

	@AverageDuration
	@Test(timeout=300000)
   public void testTextDisplay()
   {
      Graphics3DWorld world = new Graphics3DWorld(new JMEGraphics3DAdapter());

      String text = "IHMC";

      Graphics3DObject textObject = new Graphics3DObject();
      textObject.setChangeable(true);
      textObject.rotate(-Math.PI / 2.0, Axis.Y);
      instruction = textObject.addText(text, 20, YoAppearance.Blue());
      Graphics3DNode textNode = new Graphics3DNode("textNode", textObject);
      
      world.addChild(textNode);
      
      world.startWithGui(1000, 800);
      
      world.addFrameListener(this);
      
      world.keepAlive(3);
      
      world.stop();
   }

   @Override
   public void postFrame(double timePerFrame)
   {
      if (counter % 10 == 0)
      {
         instruction.setAppearance(YoAppearance.Red());
         instruction.setText("Hello");
      }
      else if (counter % 5 == 0)
      {
         instruction.setText("IHMC!");
         instruction.setAppearance(YoAppearance.Blue());
      }
      
      counter++;
   }
}