package us.ihmc.communication.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.communication.net.KryoObjectCommunicatorTest.class,
   us.ihmc.communication.net.KryoStreamSerializerTest.class,
   us.ihmc.communication.networkProcessor.NetworkProcessorTest.class,
   us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicatorTest.class,
   us.ihmc.communication.packetCommunicator.KryoPacketCommunicatorTest.class,
   us.ihmc.communication.remote.serialization.JointConfigurationDataSenderTest.class,
   us.ihmc.communication.serializers.driving.OverheadMapPacketSerializerTest.class,
   us.ihmc.communication.serializers.driving.VehiclePosePacketSerializerTest.class,
   us.ihmc.communication.serializers.HighLevelStatePacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.FingerStatePacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.HandLoadBearingPacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.HandPosePacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.HandStatePacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.SpigotPosePacketSerializerTest.class,
   us.ihmc.communication.serializers.manipulation.TorusPosePacketSerializerTest.class,
   us.ihmc.communication.serializers.NChooseKSerializerTest.class,
   us.ihmc.communication.serializers.sensing.DRCLidarScanSerializerTest.class,
   us.ihmc.communication.serializers.sensing.IntrinsicCameraParametersPacketTest.class,
   us.ihmc.communication.serializers.sensing.LidarClearCommandSerializerTest.class,
   us.ihmc.communication.serializers.sensing.LidarStateSerializerTest.class,
   us.ihmc.communication.serializers.sensing.LookAtPacketSerializerTest.class,
   us.ihmc.communication.serializers.sensing.VideoPacketSerializerTest.class,
   us.ihmc.communication.serializers.walking.BlindWalkingPacketSerializerTest.class,
   us.ihmc.communication.serializers.walking.ChestOrientationPacketSerializerTest.class,
   us.ihmc.communication.serializers.walking.FootPosePacketSerializerTest.class,
   us.ihmc.communication.serializers.walking.FootstepDataListSerializerTest.class,
   us.ihmc.communication.serializers.walking.HandstepPacketSerializerTest.class,
   us.ihmc.communication.serializers.walking.PauseCommandSerializerTest.class,
   us.ihmc.communication.serializers.walking.PelvisPosePacketSerializerTest.class,
   us.ihmc.communication.streamingData.PersistentTCPClientTest.class,
   us.ihmc.communication.streamingData.PersistentTCPServerTest.class,
   us.ihmc.communication.streamingData.StreamingDataProducerConsumerTest.class,
   us.ihmc.communication.streamingData.StreamingDataTCPServerTest.class,
   us.ihmc.communication.util.NetworkConfigParametersTest.class
})

public class IHMCCommunicationDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(IHMCCommunicationDockerTestSuite.class);
   }
}
