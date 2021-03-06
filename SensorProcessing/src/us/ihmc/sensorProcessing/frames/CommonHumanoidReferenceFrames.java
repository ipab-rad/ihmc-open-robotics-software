package us.ihmc.sensorProcessing.frames;

import java.util.EnumMap;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface CommonHumanoidReferenceFrames
{
   public abstract ReferenceFrame getABodyAttachedZUpFrame();

   public abstract ReferenceFrame getMidFeetZUpFrame();
   public abstract ReferenceFrame getMidFeetUnderPelvisFrame();


   public abstract SideDependentList<ReferenceFrame> getAnkleZUpReferenceFrames();

   public abstract SideDependentList<ReferenceFrame> getFootReferenceFrames();

   public abstract SideDependentList<ReferenceFrame> getSoleFrames();

   public abstract ReferenceFrame getPelvisFrame();

   public abstract ReferenceFrame getAnkleZUpFrame(RobotSide robotSide);

   public abstract ReferenceFrame getFootFrame(RobotSide robotSide);

   public abstract ReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName);

   public abstract void updateFrames();

   public abstract EnumMap<LegJointName, ReferenceFrame> getLegJointFrames(RobotSide robotSide);

   public abstract ReferenceFrame getIMUFrame();

   public abstract ReferenceFrame getCenterOfMassFrame();

   public abstract ReferenceFrame getPelvisZUpFrame();

   public abstract ReferenceFrame getSoleFrame(RobotSide robotSide);
}
