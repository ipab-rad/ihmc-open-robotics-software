package us.ihmc.robotiq;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.wimpi.modbus.msg.ModbusRequest;
import net.wimpi.modbus.msg.ModbusResponse;
import sun.awt.RepaintArea;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.robotiq.communication.JamodTCPConnection;
import us.ihmc.robotiq.communication.RobotiqReadRequestFactory;
import us.ihmc.robotiq.communication.RobotiqWriteRequestFactory;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommunicator
{
   private final int PORT = 502;

   private JamodTCPConnection communicator;
   
   private RobotiqWriteRequestFactory writeRequestFactory = new RobotiqWriteRequestFactory();
   private RobotiqReadRequestFactory readRequestFactory = new RobotiqReadRequestFactory();
   private ModbusResponse response;
   
   private RobotiqGraspMode graspMode = RobotiqGraspMode.BASIC_MODE;
   private FingerState fingerState = FingerState.OPEN;
   
   public RobotiqHandCommunicator(RobotSide robotSide)
   {
      NetworkParameterKeys key = robotSide.equals(RobotSide.LEFT) ? NetworkParameterKeys.leftHand : NetworkParameterKeys.rightHand;
      try
      {
         communicator = new JamodTCPConnection(InetAddress.getByName(NetworkParameters.getHost(key)), PORT);
         communicator.connect();
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
   }
   
   public boolean isConnected()
   {
      return communicator.isConnected();
   }
   
   public void read()
   {
      ModbusRequest request = readRequestFactory.getReadRequest();
      response = communicator.sendRequest(request);
   }
   
   public void initialize()
   {
      ModbusRequest request = writeRequestFactory.createActivationRequest();
      response = communicator.sendRequest(request);
   }
   
   public void open()
   {
      ModbusRequest request = writeRequestFactory.createFingerPositionRequest(graspMode, FingerState.OPEN);
      response = communicator.sendRequest(request);
   }
   
   public void close()
   {
      ModbusRequest request = writeRequestFactory.createFingerPositionRequest(graspMode, FingerState.CLOSE);
      System.out.println("Close request: " + request.getHexMessage());
      response = communicator.sendRequest(request);
      System.out.println("Close response: " + response.getHexMessage());
   }
   
   public void crush()
   {
      
   }
   
   public void basicGrip()
   {
      
   }
   
   public void pinchGrip()
   {
      
   }
   
   public void wideGrip()
   {
      
   }
   
   public RobotiqHandSensorData updateHandStatus()
   {
      // TODO
      // take stuff from ModbusResponse and put into something nice
      // so other stuff doesn't need to see Modbus stuff
      
      return null;
   }
   
   public static void main(String[] args)
   {
      RobotSide robotSide = RobotSide.LEFT;
      
      final RobotiqHandCommunicator hand = new RobotiqHandCommunicator(robotSide);
      
      hand.initialize();
      
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            hand.read();
         }
      }, 0, 10, TimeUnit.MILLISECONDS);
      
      executor.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            System.out.println("Closing...");
            hand.close();
         }
      }, 10, TimeUnit.SECONDS);
   }
}