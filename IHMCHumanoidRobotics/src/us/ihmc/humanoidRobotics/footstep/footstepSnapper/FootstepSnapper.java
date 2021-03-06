package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FootstepSnapper
{
   public abstract Footstep.FootstepType snapFootstep(Footstep footstep, HeightMapWithPoints heightMap);
   /**
    *
    * @param footstep the footstep position and orientaion of the sole
    * @param heightMap the heightmap
    */
   public abstract Footstep.FootstepType snapFootstep(FootstepData footstep, HeightMapWithPoints heightMap);

   public abstract Footstep generateSnappedFootstep(double soleX, double soleY, double yaw, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
         HeightMapWithPoints heightMap) throws InsufficientDataException;

   public abstract Footstep generateFootstepUsingHeightMap(FramePose2d desiredSolePosition, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
         HeightMapWithPoints heightMap) throws InsufficientDataException;

   /**
    *
    * @param footstep the footstep position and orientation, etc of the sole
    * @param height the height to set the foot at
    * @param planeNormal the z normal of the foot
    */
   public void adjustFootstepWithoutHeightmap(FootstepData footstep, double height, Vector3d planeNormal);
   public abstract void adjustFootstepWithoutHeightmap (Footstep footstep, double height, Vector3d planeNormal);
   public abstract Footstep generateFootstepWithoutHeightMap(FramePose2d desiredSolePosition, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height, Vector3d planeNormal);

   public abstract void setMask(List<Point2d> footShape);
   public abstract void setUseMask(boolean useMask, double kernelMaskSafetyBuffer, double boundingBoxDimension);
   public abstract List<Point3d> getPointList();
}