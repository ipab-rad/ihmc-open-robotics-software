package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.io.InputStream;
import java.net.Authenticator;
import java.net.PasswordAuthentication;
import java.net.URL;

import us.ihmc.utilities.ThreadTools;

/**
 * <p>Title: DrillDetector</p>
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data from the Atlas Chest Webcam microphone around a given frequency</p>
 * 
 * @author Will
 * @author Igor
 */
public class DrillDetectorProcess
{
   // The annoying audio.cgi disconnects us every ~33 seconds
   private static final int reconnectPeriodSeconds = 32;
   private static final int checkForDrillFrequencyHz = 4;
   private static final int iterationsCount = reconnectPeriodSeconds * checkForDrillFrequencyHz;
   private static final long iterationSleep = 1000 / checkForDrillFrequencyHz;

   // initial state
   private static DrillDetector detector = new DrillDetector();
   private static boolean lastDrillState = false;
   private static InputStream inputStream = null;

   private static void connectToStream()
   {
      try
      {
//         URL url = new URL("http://10.6.100.57:80/audio.cgi"); //Robotlab
//         URL url = new URL("http://192.168.0.19:80/audio.cgi"); //Home

         URL url = new URL("http://139.169.44.114:80/audio.cgi"); //JSC
         inputStream = url.openStream();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         inputStream = null;
      }
   }

   private static void startDrillDetectionLoop()
   {
      while (true)
      {
         connectToStream();
         if (inputStream == null)
         {
            ThreadTools.sleep(iterationSleep);
            continue;
         }

         System.out.println("Connected to the stream!");

         for (int i = 0; i < iterationsCount; i++)
         {
            boolean newState = detector.isDrillOn(inputStream);
            if (lastDrillState != newState)
            {
               System.out.println(" - state change detected: isOn=" + newState);
               lastDrillState = newState;
            }

            ThreadTools.sleep(iterationSleep);
         }

         System.out.println("Waiting for reconnect...");
      }
   }

   public static void main(String[] args)
   {
      // set webcam authentification
      Authenticator.setDefault(new Authenticator()
      {
         protected PasswordAuthentication getPasswordAuthentication() { return new PasswordAuthentication("admin", "unknownpw".toCharArray()); }
      });

      startDrillDetectionLoop();
   }
}