package us.ihmc.robotics.screwTheory;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class CenterOfMassJacobianTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RigidBody elevator;
   private Random random;

   @Before
   public void setUp()
   {
      elevator = new RigidBody("elevator", worldFrame);
      random = new Random(1986L);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSingleJoint()
   {
      ArrayList<RevoluteJoint> joints = setUpSingleJoint();
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianTwoJointsSimple()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianRandomChain()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomChain(elevator);
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTree()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomTree(elevator);
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRigidBodyListSortInvariant()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomTree(elevator);
      RigidBody[] rigidBodiesInOrder = ScrewTools.computeSupportAndSubtreeSuccessors(elevator);
      List<RigidBody> rigidBodiesInOrderArrayList = Arrays.asList(rigidBodiesInOrder);

      Collections.shuffle(rigidBodiesInOrderArrayList, random);
      RigidBody[] rigidBodiesOutOrder = rigidBodiesInOrderArrayList.toArray(new RigidBody[rigidBodiesInOrder.length]);

      ScrewTestTools.setRandomPositions(joints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(joints, random);

      CenterOfMassJacobian jacobianInOrder = new CenterOfMassJacobian(rigidBodiesInOrder, elevator.getBodyFixedFrame());
      jacobianInOrder.compute();
      FrameVector velocityFromJacobianInOrder = new FrameVector(ReferenceFrame.getWorldFrame());
      jacobianInOrder.packCenterOfMassVelocity(velocityFromJacobianInOrder);

      CenterOfMassJacobian jacobianOutOrder = new CenterOfMassJacobian(rigidBodiesOutOrder, elevator.getBodyFixedFrame());
      jacobianOutOrder.compute();
      FrameVector velocityFromJacobianOutOrder = new FrameVector(ReferenceFrame.getWorldFrame());
      jacobianOutOrder.packCenterOfMassVelocity(velocityFromJacobianOutOrder);

      JUnitTools.assertTuple3dEquals(velocityFromJacobianInOrder.getVectorCopy(), velocityFromJacobianOutOrder.getVectorCopy(), 1e-5);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSixDoFPlusRandomChain()
   {
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevator.getBodyFixedFrame());
      RigidBody floatingBody = ScrewTools.addRigidBody("floating", sixDoFJoint, new Matrix3d(), random.nextDouble(), RandomTools.generateRandomVector(random));
      ArrayList<RevoluteJoint> revoluteJoints = setUpRandomChain(floatingBody);

      CenterOfMassJacobian jacobian = new CenterOfMassJacobian(elevator);

      sixDoFJoint.setPositionAndRotation(RigidBodyTransform.generateRandomTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.packJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.setLinearPart(RandomTools.generateRandomVector(random));
      sixDoFJointTwist.setAngularPart(RandomTools.generateRandomVector(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      ScrewTestTools.setRandomPositions(revoluteJoints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(revoluteJoints, random);

      jacobian.compute();
      FrameVector velocityFromJacobian = new FrameVector(ReferenceFrame.getWorldFrame());
      jacobian.packCenterOfMassVelocity(velocityFromJacobian);

      RigidBody rootBody = elevator;
      FrameVector velocityNumerical = computeCenterOfMassVelocityNumerically(sixDoFJoint, revoluteJoints, rootBody,
                                         ScrewTools.computeSupportAndSubtreeSuccessors(rootBody), elevator.getBodyFixedFrame());

      Matrix3d rotation = new Matrix3d();
      sixDoFJoint.packRotation(rotation);

      JUnitTools.assertTuple3dEquals(velocityNumerical.getVectorCopy(), velocityFromJacobian.getVectorCopy(), 4e-5);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSkipLeafBody()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      RigidBody[] rigidBodies = new RigidBody[] {joints.get(0).getSuccessor()};
      testComputeJacobianRevoluteJoints(joints, rigidBodies, elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSkipIntermediateBody()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      RigidBody[] rigidBodies = new RigidBody[] {joints.get(joints.size() - 1).getSuccessor()};
      testComputeJacobianRevoluteJoints(joints, rigidBodies, elevator.getBodyFixedFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeRootJointLinearVelocity()
   {
      Random random = new Random(44345L);

      RigidBody elevator = new RigidBody("elevator", worldFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, worldFrame);
      RigidBody body0 = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<RevoluteJoint> revoluteJoints = new ArrayList<RevoluteJoint>();
      ScrewTestTools.createRandomTreeRobot(revoluteJoints, body0, 25, random);
      
      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(revoluteJoints, random);
      ScrewTestTools.setRandomVelocities(revoluteJoints, random);
      elevator.updateFramesRecursively();

      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      Twist rootJointTwist = new Twist();
      rootJoint.packJointTwist(rootJointTwist);
      FrameVector rootJointLinearVelocity = new FrameVector(rootJointFrame);
      rootJointTwist.packLinearPart(rootJointLinearVelocity);
      
      CenterOfMassJacobian jacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator), ScrewTools.computeSubtreeJoints(body0), elevator.getBodyFixedFrame());
      jacobianBody.compute();
      FrameVector comVelocityBody = new FrameVector(worldFrame);
      jacobianBody.packCenterOfMassVelocity(comVelocityBody);
      comVelocityBody.changeFrame(rootJointFrame);
      
      CenterOfMassJacobian jacobianWorld = new CenterOfMassJacobian(elevator);
      jacobianWorld.compute();
      FrameVector comVelocityWorld = new FrameVector(worldFrame);
      jacobianWorld.packCenterOfMassVelocity(comVelocityWorld);
      comVelocityWorld.changeFrame(rootJointFrame);
      
      FrameVector angularVelocityBody = new FrameVector(rootJointFrame);
      rootJointTwist.packAngularPart(angularVelocityBody);

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJointFrame);
      centerOfMassCalculator.compute();
      FramePoint centerOfMassBody = new FramePoint(worldFrame);
      centerOfMassCalculator.packCenterOfMass(centerOfMassBody);
      FrameVector crossPart = new FrameVector(rootJointFrame);
      crossPart.cross(angularVelocityBody, centerOfMassBody);
      
      FrameVector rootJointLinearVelocityBack = new FrameVector(comVelocityWorld);
      rootJointLinearVelocityBack.sub(crossPart);
      rootJointLinearVelocityBack.sub(comVelocityBody);

      assertTrue(rootJointLinearVelocity.epsilonEquals(rootJointLinearVelocityBack, 1e-12));
   }
   
   public static FrameVector computeCenterOfMassVelocityNumerically(SixDoFJoint sixDoFJoint, ArrayList<RevoluteJoint> revoluteJoints, RigidBody rootBody,
           RigidBody[] rigidBodiesToUse, ReferenceFrame referenceFrame)
   {
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rigidBodiesToUse, referenceFrame);

      centerOfMassCalculator.compute();
      FramePoint centerOfMass1 = new FramePoint(centerOfMassCalculator.getCenterOfMass());

      double dt = 1e-8;
      if (sixDoFJoint != null)
         ScrewTestTools.integrateVelocities(sixDoFJoint, dt);
      ScrewTestTools.integrateVelocities(revoluteJoints, dt);
      rootBody.updateFramesRecursively();

      centerOfMassCalculator.compute();
      FramePoint centerOfMass2 = new FramePoint(centerOfMassCalculator.getCenterOfMass());

      FrameVector velocityNumerical = new FrameVector(centerOfMass2);
      velocityNumerical.sub(centerOfMass1);
      velocityNumerical.scale(1.0 / dt);

      return velocityNumerical;
   }

   private void testComputeJacobianRevoluteJoints(ArrayList<RevoluteJoint> joints, RigidBody[] rigidBodies, ReferenceFrame referenceFrame)
   {
      CenterOfMassJacobian jacobian = new CenterOfMassJacobian(rigidBodies, referenceFrame);

      ScrewTestTools.setRandomPositions(joints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(joints, random);

      jacobian.compute();
      FrameVector velocityFromJacobian = new FrameVector(ReferenceFrame.getWorldFrame());
      jacobian.packCenterOfMassVelocity(velocityFromJacobian);

      RigidBody rootBody = elevator;
      FrameVector velocityNumerical = computeCenterOfMassVelocityNumerically(null, joints, rootBody, rigidBodies, referenceFrame);

      JUnitTools.assertTuple3dEquals(velocityNumerical.getVectorCopy(), velocityFromJacobian.getVectorCopy(), 1e-5);
   }

   private ArrayList<RevoluteJoint> setUpSingleJoint()
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      joints = new ArrayList<RevoluteJoint>();
      RevoluteJoint joint = ScrewTools.addRevoluteJoint("joint", elevator, RandomTools.generateRandomVector(random), X);
      joints.add(joint);
      ScrewTools.addRigidBody("body", joint, new Matrix3d(), random.nextDouble(), RandomTools.generateRandomVector(random));

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpTwoJointsSimple()
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RevoluteJoint joint1 = ScrewTools.addRevoluteJoint("joint1", elevator, new Vector3d(0.0, 1.0, 0.0), X);
      joints.add(joint1);
      RigidBody body1 = ScrewTools.addRigidBody("body1", joint1, new Matrix3d(), 2.0, new Vector3d(0.0, 1.0, 0.0));
      RevoluteJoint joint2 = ScrewTools.addRevoluteJoint("joint2", body1, new Vector3d(0.0, 1.0, 0.0), X);
      joints.add(joint2);
      ScrewTools.addRigidBody("body2", joint2, new Matrix3d(), 3.0, new Vector3d(0.0, 1.0, 0.0));

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpRandomChain(RigidBody rootBody)
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      Vector3d[] jointAxes =
      {
         X, X, Y, Z, X, Y, Z, Z
      };
      ScrewTestTools.createRandomChainRobot("", joints, rootBody, jointAxes, random);

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpRandomTree(RigidBody elevator)
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();

      Vector3d[] jointAxes1 = {X, Y, Z, Y};
      ScrewTestTools.createRandomChainRobot("chainA", joints, elevator, jointAxes1, random);

      Vector3d[] jointAxes2 = {Z, X, Y, X};
      ScrewTestTools.createRandomChainRobot("chainB", joints, elevator, jointAxes2, random);

      Vector3d[] jointAxes3 = {Y, Y, X};
      ScrewTestTools.createRandomChainRobot("chainC", joints, joints.get(joints.size() - 2).getPredecessor(), jointAxes3, random);

      return joints;
   }
}
