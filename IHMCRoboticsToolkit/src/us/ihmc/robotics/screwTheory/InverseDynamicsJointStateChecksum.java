package us.ihmc.robotics.screwTheory;

import us.ihmc.tools.compare.GenericCRC32;

public class InverseDynamicsJointStateChecksum extends AbstractInverseDynamicsChecksum
{

   public InverseDynamicsJointStateChecksum(RigidBody rootJoint, GenericCRC32 checksum)
   {
      super(rootJoint, checksum);
   }

   @Override
   public void calculateJointChecksum(InverseDynamicsJoint joint)
   {
      joint.calculateJointStateChecksum(checksum);
   }

}