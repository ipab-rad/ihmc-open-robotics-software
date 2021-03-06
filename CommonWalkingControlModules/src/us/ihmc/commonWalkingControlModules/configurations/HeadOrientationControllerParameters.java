package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;


public interface HeadOrientationControllerParameters
{
   public abstract String[] getDefaultHeadOrientationControlJointNames();

   public abstract YoOrientationPIDGains createHeadOrientationControlGains(YoVariableRegistry registry);

   public abstract double[] getInitialHeadYawPitchRoll();

   public abstract boolean isNeckPositionControlled();

   public abstract double getNeckPitchUpperLimit();

   public abstract double getNeckPitchLowerLimit();

   public abstract double getHeadYawLimit();

   public abstract double getHeadRollLimit();
   
   public abstract double getTrajectoryTimeHeadOrientation();
}
