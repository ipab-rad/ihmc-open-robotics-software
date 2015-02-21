package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//   us.ihmc.valkyrie.simulation.ValkyriePosePlaybackDemoTest.class, // Moved to flaky
   us.ihmc.valkyrie.kinematics.transmissions.InefficientPushrodTransmissionJacobianTest.class,
   us.ihmc.valkyrie.kinematics.transmissions.InefficientPushRodTransmissionTest.class,
   us.ihmc.valkyrie.kinematics.PushrodTransmissionTest.class,
   us.ihmc.valkyrie.kinematics.ClosedFormJacobianTest.class,
//   us.ihmc.valkyrie.codeGenerators.APIBuilderTest.class,
   us.ihmc.valkyrie.kinematics.transmissions.ComparePushRodTransmissionsTest.class,
//   us.ihmc.valkyrie.networkProcessor.depthData.ValkyrieDepthDataProcessorTest.class, // Feature not yet implemented @dcalvert
})

public class ValkyrieBambooValkyrieTestSuite
{
   public static void main(String[] args)
   {
   }
}