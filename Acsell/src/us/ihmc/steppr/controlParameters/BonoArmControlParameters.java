package us.ihmc.steppr.controlParameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;


/**
 * Created by dstephen on 2/14/14.
 */
public class BonoArmControlParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   
   public BonoArmControlParameters()
   {
      this(false);
   }
   
   public BonoArmControlParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public boolean useInverseKinematicsTaskspaceControl()
   {
      return false;
   }

   @Override
   public boolean doLowLevelPositionControl()
   {
      return false;
   }
   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
   }

   @Override
   public double getWristHandCenterOffset()
   {
      // TODO Auto-generated method stub
      return 0.0;
   }
}
