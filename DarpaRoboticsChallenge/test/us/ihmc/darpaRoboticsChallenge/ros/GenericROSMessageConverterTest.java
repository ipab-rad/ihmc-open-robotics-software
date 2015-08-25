package us.ihmc.darpaRoboticsChallenge.ros;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Random;

import org.junit.Test;
import org.ros.internal.message.Message;

import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.darpaRoboticsChallenge.ros.IHMCRosApiMessageMap;
import us.ihmc.tools.agileTesting.BambooAnnotations;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter;

/**
 * Created by agrabertilton on 4/23/15.
 */
@BambooAnnotations.BambooPlan(planType = BambooPlanType.Flaky)
public class GenericROSMessageConverterTest
{
   private int numberOfTimesToRunEachPacketType = 1000;

   @EstimatedDuration(duration = 2.3)
   @Test(timeout = 300000)
   public void AllPacketTest()
   {
      PrintTools.info(
          this,
          "If this test fails, run ROSMessageGenerator then MessageAndServiceInterfaceGenerator. see Confluence for more details (https://confluence.ihmc.us/display/HOWTO/How+to+Generate+Code+after+Packet+Change)");

      Random random = new Random(6547568l);
      for (Class clazz : IHMCRosApiMessageMap.PACKET_LIST)
      {
         for (int i = 0; i < numberOfTimesToRunEachPacketType; i++)
         {
            assertTrue(testPacketTranslation(clazz, random));
         }
      }
   }

   public static Packet convertRosMsgToPacket(Message message)
           throws IllegalArgumentException, IllegalAccessException, InvocationTargetException, NoSuchMethodException, SecurityException, InstantiationException
   {
      String messageType = message.toRawMessage().getType();
      Class packetClass = IHMCRosApiMessageMap.PACKET_MESSAGE_NAME_MAP.get(messageType);
      Constructor<Packet> emptyConstructor = packetClass.getDeclaredConstructor();
      Packet outputPacket = emptyConstructor.newInstance();
      GenericRosMessageConverter.convertToPacket(message, outputPacket);

      return outputPacket;
   }


//   @EstimatedDuration(duration = 0.05)
//   @Test(timeout=300000)
//   public void SinglePacketTest()
//   {
//      Random random = new Random(357765l);
//      Class clazz = 
//      for (int i = 0; i < numberOfTimesToRunEachPacketType; i++)
//      {
//         assertTrue(testPacketTranslation(clazz, random));
//      }
//   }


   private static <T extends IHMCRosApiPacket> boolean testPacketTranslation(Class<T> clazz, Random random)
   {
      T untranslated = null;
      Message translatedMessage;
      T translated = null;
      try
      {
         Constructor constructor = clazz.getDeclaredConstructor(Random.class);
         untranslated = (T) constructor.newInstance(random);

         translatedMessage = GenericRosMessageConverter.convertToIHMCRosMessage(untranslated);
         translated = (T) convertRosMsgToPacket(translatedMessage);

         boolean packetConversionResult = untranslated.rosConversionEpsilonEquals(translated, 0.0003);
         if (!packetConversionResult)
         {
            System.out.println(clazz.getSimpleName() + " failed!");
            System.out.println(untranslated.toString());
            System.out.println(translated.toString());
         }

         return packetConversionResult;
      }
      catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException | ClassNotFoundException e)
      {
         System.out.println("Please make sure you generated the messages and interfaces, which may be the issue");
         e.printStackTrace();
      }

      return false;
   }
}