package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RigidBody
{
   private final RigidBodyInertia inertia;
   private final ReferenceFrame bodyFixedFrame;
   private final InverseDynamicsJoint parentJoint;
   private final ArrayList<InverseDynamicsJoint> childrenJoints = new ArrayList<InverseDynamicsJoint>();
   private final List<InverseDynamicsJoint> childrenJointsReadOnly = Collections.unmodifiableList(childrenJoints);
   private final String name;

   public RigidBody(String name, ReferenceFrame rootBodyFrame)    // root body constructor
   {
      this.name = name;
      this.inertia = null;
      this.bodyFixedFrame = rootBodyFrame;
      this.parentJoint = null;
   }

   public RigidBody(String name, RigidBodyInertia inertia, InverseDynamicsJoint parentJoint)
   {
      inertia.getBodyFrame().checkReferenceFrameMatch(inertia.getExpressedInFrame());    // inertia should be expressed in body frame, otherwise it will change
      this.name = name;
      this.inertia = inertia;
      this.bodyFixedFrame = inertia.getBodyFrame();
      this.parentJoint = parentJoint;
      this.parentJoint.setSuccessor(this);
   }

   public RigidBodyInertia getInertia()
   {
      return inertia;
   }

   public RigidBodyInertia getInertiaCopy()
   {
      return new RigidBodyInertia(getInertia());
   }

   public ReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   public InverseDynamicsJoint getParentJoint()
   {
      return parentJoint;
   }

   public void addChildJoint(InverseDynamicsJoint joint)
   {
      this.childrenJoints.add(joint);
   }

   public List<InverseDynamicsJoint> getChildrenJoints()
   {
      return childrenJointsReadOnly;
   }

   public boolean hasChildrenJoints()
   {
      return !childrenJoints.isEmpty();
   }

   public boolean isRootBody()
   {
      return parentJoint == null;
   }

   public String getName()
   {
      return name;
   }

   public void packCoMOffset(FramePoint comOffsetToPack)
   {
      inertia.getCenterOfMassOffset(comOffsetToPack);
   }
   
   public void setCoMOffset(FramePoint comOffset)
   {
      inertia.setCenterOfMassOffset(comOffset);
   }

   public void updateFramesRecursively()
   {
      this.bodyFixedFrame.update();

      for(int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
      {
         childrenJoints.get(childIndex).updateFramesRecursively();
      }
   }

   @Override
   public String toString()
   {
      return name;
   }
}
