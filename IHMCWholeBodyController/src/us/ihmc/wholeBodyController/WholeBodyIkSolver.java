package us.ihmc.wholeBodyController;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang.mutable.MutableBoolean;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.hierarchicalKinematics.ForwardKinematicSolver;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalKinematicSolver;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyPose;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_COM;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_JointsPose;
import us.ihmc.utilities.hierarchicalKinematics.RobotModel;
import us.ihmc.utilities.hierarchicalKinematics.VectorXd;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.nativelibraries.NativeLibraryLoader;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

abstract public class WholeBodyIkSolver
{
   static{
      NativeLibraryLoader.loadLibrary("us.ihmc.utilities.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
   }

   public enum ComputeResult { FAILED_INVALID, FAILED_NOT_CONVERGED, SUCCEEDED };

   ///--------------- Constants -----------------------------------------
   static public final double IGNORE = -66666.666;


   static public enum ControlledDoF { DOF_NONE, DOF_3P, DOF_3P2R, DOF_3P3R }

   static public enum ComputeOption { RESEED, USE_ACTUAL_MODEL_JOINTS, USE_JOINTS_CACHE, RELAX };

   // I just like shorter names :/
   final private static RobotSide RIGHT = RobotSide.RIGHT;
   final private static RobotSide LEFT  = RobotSide.LEFT;

   ///--------------- Reference Frames and Transfoms --------------------------
   private final ReferenceFrames workingFrames;


   //  private final SideDependentList<ReferenceFrame> actualSoleFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> workingSoleFrames = new SideDependentList<ReferenceFrame>();

   //---------- robot models in different contexts and different formats ----------
   private final RobotModel        urdfModel;
   private final SDFFullRobotModel workingSdfModel;
   protected final HierarchicalKinematicSolver hierarchicalSolver;

   //---------------  tasks assigned to this solver --------------------------------
   public HierarchicalTask_COM        taskComPosition;
   public HierarchicalTask_JointsPose taskJointsPose;
   public final SideDependentList<HierarchicalTask_BodyPose> taskEndEffectorPose = new SideDependentList<HierarchicalTask_BodyPose>();
   public final HierarchicalTask_BodyPose taskLegPose;

   // ------------- Parameters that change the overall behavior ----------------------

   protected final SideDependentList<MutableBoolean> keepArmQuiet = new SideDependentList<MutableBoolean>();
   private final SideDependentList<ControlledDoF>  controlledDoF = new SideDependentList<ControlledDoF>();
   private boolean lockLegs = false;
   private boolean suggestKneeAngle = true;

   // --------------------- Miscellaneous -------------------------------------
   protected final HashMap<String, Integer> jointNamesToIndex = new HashMap<String, Integer>();
   protected final OneDoFJoint[] workingJointsInUrdfOrder;
   protected final SideDependentList<HashSet<Integer>> armJointIds = new SideDependentList<HashSet<Integer>>();
   protected final SideDependentList<HashSet<Integer>> legJointIds = new SideDependentList<HashSet<Integer>>();

   protected final DenseMatrix64F coupledJointWeights;
   protected final Vector64F      preferedJointPose;

   protected final Vector64F cachedAnglesQ;

   final private int numOfJoints;
   private HashSet<String> listOfVisibleAndEnableColliders;
   public int maxNumberOfAutomaticReseeds = 3;

   //------- Abstract method that provide robot-specific information and configuration  -------------

   abstract public String getGripperAttachmentLinkName(RobotSide side);
   abstract public String getGripperPalmLinkName(RobotSide side);
   abstract public String getFootLinkName(RobotSide side);

   abstract public int getNumberDoFperArm();
   abstract public int getNumberDoFperLeg();
   abstract public RobotSide getSideOfTheFootRoot();
   abstract public String getURDFConfigurationFileName()  throws IOException ;
   abstract protected HashSet<String> configureCollisionAvoidance(String urdfFileName )  throws Exception;
   abstract protected void configureTasks();

   abstract public ReferenceFrame  getRootFrame( SDFFullRobotModel model);

   //---------------------------------------------------------------------------------

   public void activateAutomaticPreferredHipHeight(boolean enable)
   {
      suggestKneeAngle = enable;
   }

   public ReferenceFrame getRootFrame(){
      return getRootFrame(workingSdfModel);
   }

   public boolean hasJoint(String jointName)
   {
      return (jointNamesToIndex.get(jointName) != null);
   }

   
   public boolean checkCollisions(SDFFullRobotModel modelToCheck)
   {
      HashMap<String,RigidBodyTransform> bodyTransforms = new HashMap<String,RigidBodyTransform>();
      for( OneDoFJoint joint : modelToCheck.getOneDoFJoints())
      {
         String bodyName     = joint.getSuccessor().getName();
         RigidBodyTransform bodyTransform = joint.getFrameAfterJoint().getTransformToWorldFrame();
         
         bodyTransforms.put(bodyName, bodyTransform);
      }
      
      return getHierarchicalSolver().collisionAvoidance.hasCollision( bodyTransforms );
   }
   
   /*
    * Verbosity levels:
    * 0: none
    * 1: show a resume of the errors.
    * 2: Lot of data...
    */

   public void setVerbosityLevel(int level)
   {
      hierarchicalSolver.setVerbosityLevel(level);
   }

   public RobotModel getUrdfRobotModel()
   {
      return urdfModel;
   }


   /* Note. The after/before direction is reversed between the SDF and HIK models for the right leg.
    * This method uses the direction specified by the WB (reversed).
    */
   public int getIdOfBodyAfterJoint(String jointname)
   {
      int jointId = urdfModel.getJointIndexByName(jointname);
      return  getIdOfBodyAfterJoint(jointId);
   }

   /* Note. The after/before direction is reversed between the SDF and HIK models for the right leg.
    * This method uses the direction specified by the WB (reversed).
    */
   public int getIdOfBodyAfterJoint(int jointId)
   {
      return  urdfModel.getChildBodyOfJoint(jointId);
   }

   public RigidBodyTransform getLocalBodyTransform(String bodyname )
   {
      int bodyid = getUrdfRobotModel().getBodyId( bodyname );
      return getLocalBodyTransform(bodyid);
   }

   public RigidBodyTransform getLocalBodyTransform(int bodyid )
   {
      RigidBodyTransform out = new RigidBodyTransform();
      VectorXd body_rot = new VectorXd(4);
      VectorXd body_pos = new VectorXd(3);
      getHierarchicalSolver().getForwardSolver().getBodyPose( bodyid, body_rot, body_pos);

      Quat4d quat = new Quat4d( body_rot.get(0), body_rot.get(1), body_rot.get(2), body_rot.get(3) );
      quat.inverse();

      out.setRotation( quat );
      out.setTranslation( new Vector3d( body_pos.get(0), body_pos.get(1), body_pos.get(2) ) ); 
      return out;
   }


   public ReferenceFrame getBodyReferenceFrame(String bodyname)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyname );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", 
            getRootFrame(workingSdfModel), localTransform);
      System.out.println( localTransform );
      return bodyFrame;
   }

   public ReferenceFrame getBodyReferenceFrame(int bodyId)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyId );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", 
            getRootFrame(workingSdfModel), localTransform);
      return bodyFrame;
   }   


   public HierarchicalKinematicSolver getHierarchicalSolver()
   {
      return hierarchicalSolver;
   }

   public int getNumberOfJoints()
   {
      return urdfModel.getNrOfJoints();
   }

   static 
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.utilities.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
   }

   public WholeBodyIkSolver( WholeBodyControllerParameters robotControllerParameters ) 
   {
      // load external library
      workingSdfModel  = robotControllerParameters.createFullRobotModel();
      workingFrames = new ReferenceFrames(workingSdfModel);

      //----------- STEP A: find, copy and load the urdf file -------------------------------- 

      // create a RobotModel using a special URDF
      urdfModel = new RobotModel();

      try{
         urdfModel.loadURDF( getURDFConfigurationFileName() , false);
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }

      numOfJoints = urdfModel.getNrOfJoints();

      workingJointsInUrdfOrder = new OneDoFJoint[numOfJoints];

      double EpsilonInRadiansForVerySmallAngles = 0.001;
      int NumberOfIterations = 60;

      hierarchicalSolver = new HierarchicalKinematicSolver(urdfModel, NumberOfIterations, EpsilonInRadiansForVerySmallAngles);
      hierarchicalSolver.setVerbosityLevel(0);

      cachedAnglesQ = new Vector64F(numOfJoints);

      for (RobotSide robotSide : RobotSide.values())
      {
         workingSoleFrames.set(robotSide, workingFrames.getSoleFrame(robotSide));
         armJointIds.set(robotSide, new HashSet<Integer>());
         legJointIds.set(robotSide, new HashSet<Integer>());
         controlledDoF.set(robotSide, ControlledDoF.DOF_NONE );
         keepArmQuiet.set(robotSide, new MutableBoolean(true));
      }

      for (int i = 0; i < numOfJoints; i++)
      {
         String joint_name_in_urdf = urdfModel.getActiveJointName(i);
         jointNamesToIndex.put(joint_name_in_urdf, i);
         OneDoFJoint joint = workingSdfModel.getOneDoFJointByName(joint_name_in_urdf);
         if (joint != null)
         {
            workingJointsInUrdfOrder[i] = joint;
         }
         else
         {
            System.err.println("WholeBodyIkSolver: Joint " + joint_name_in_urdf + " in urdf not found in sdf");
         }
      }

      ///----- Step B: allocate all the tasks and add them to the solver --------------

      ForwardKinematicSolver fk = hierarchicalSolver.getForwardSolver();

      taskComPosition = new HierarchicalTask_COM(fk);

      for (RobotSide robotSide : RobotSide.values())
      {
         int currentHandId = urdfModel.getBodyId(getGripperPalmLinkName(robotSide));

         taskEndEffectorPose.set(robotSide, new HierarchicalTask_BodyPose(fk, urdfModel, getGripperPalmLinkName(robotSide)));
         int [] indexes = new  int[ getNumberDoFperArm()];
         urdfModel.getParentJoints(currentHandId, getNumberDoFperArm() , indexes);
         for (int i: indexes)
         {
            armJointIds.get(robotSide).add( i );
         }
      }     

      {
         RobotSide otherFootSide = getSideOfTheFootRoot().getOppositeSide(); 
         int leftFootId = urdfModel.getBodyId(  getFootLinkName(otherFootSide ) );
         taskLegPose = new HierarchicalTask_BodyPose(fk,urdfModel,  getFootLinkName(otherFootSide ));

         int [] indexes = new  int[ getNumberDoFperArm()];
         urdfModel.getParentJoints(leftFootId, getNumberDoFperLeg(), indexes );
         for (int i: indexes)
         {
            legJointIds.get(otherFootSide).add( i );
         }
      }

      for (int i = 0; i < getNumberDoFperLeg(); i++)
      {
         legJointIds.get( getSideOfTheFootRoot()).add(i);
      }

      taskJointsPose = new HierarchicalTask_JointsPose(fk);

      taskLegPose.setName( "left Leg Pose");
      taskComPosition.setName( "COM Position");
      taskEndEffectorPose.get(RIGHT).setName( "Right Hand Pose" );
      taskEndEffectorPose.get(LEFT).setName( "Left Hand Pose" );
      taskJointsPose.setName( "Whole body Posture" );

      // the order is important!! first to be added have higher priority
      hierarchicalSolver.addTask(taskLegPose);
      hierarchicalSolver.addTask(taskComPosition);
      hierarchicalSolver.addTask(taskEndEffectorPose.get(RIGHT));
      hierarchicalSolver.addTask(taskEndEffectorPose.get(LEFT));
      hierarchicalSolver.addTask(taskJointsPose);

      preferedJointPose = new Vector64F(numOfJoints);
      preferedJointPose.zero();
      coupledJointWeights = new DenseMatrix64F(numOfJoints, numOfJoints);

      //code moved there for esthetic reasons :|
      configureTasks();
      try{
         listOfVisibleAndEnableColliders = configureCollisionAvoidance( getURDFConfigurationFileName() );
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }
   }

   public boolean isBodyIncludedInCollision(String bodyName)
   {
      return listOfVisibleAndEnableColliders.contains(bodyName);
   }



   // TODO: this is robot specific ! Move it to AtlasWholeBodyIk
   private void setPreferedKneeAngle()
   {
      if( suggestKneeAngle == false)
      {
         return;
      }
      HierarchicalTask_BodyPose taskL = taskEndEffectorPose.get(LEFT);
      HierarchicalTask_BodyPose taskR = taskEndEffectorPose.get(RIGHT);

      double zL = taskL.getTarget().get(2);
      double zR = taskR.getTarget().get(2);

      if(taskL.isEnabled() && !taskR.isEnabled() ) {
         zR = zL;
      }
      else  if( !taskL.isEnabled() && taskR.isEnabled() ) {
         zL = zR;
      }
      else if( !taskL.isEnabled() && !taskR.isEnabled() ) {
         zL = 1;
         zR = 1;
      }

      double preferedKneeAngle = 2.6 - Math.min(zL , zR);

      if( preferedKneeAngle < 0.4) preferedKneeAngle = 0.4;

      preferedJointPose.set(jointNamesToIndex.get("l_leg_kny"), preferedKneeAngle);
      preferedJointPose.set(jointNamesToIndex.get("r_leg_kny"), preferedKneeAngle);
      taskJointsPose.setTarget(preferedJointPose);
   }

   /*
    * We call "GripperAttachment" the location where the end effector would be if no gripper is present.
    * Its position corresponds with SDFFullRobotModel.getHandControlFrame
    */
   public void setGripperAttachmentTarget(SDFFullRobotModel actualRobotModel, RobotSide endEffectorSide, FramePose endEffectorPose)
   {
      PoseReferenceFrame endEffectorPoseReferenceFrame = new PoseReferenceFrame("endEffectorPoseReferenceFrame", endEffectorPose);
      taskEndEffectorPose.get(endEffectorSide).setBodyToControl( getGripperAttachmentLinkName(endEffectorSide));

      setEndEffectorTarget(actualRobotModel, endEffectorSide, endEffectorPoseReferenceFrame);
   }

   /*
    * We call "GripperPalm" the location where grasping shall be made.
    * Its position corresponds with the blue cylinder of the ModifiafleCylinderGrabber.
    */
   public void setGripperPalmTarget(SDFFullRobotModel actualRobotModel, RobotSide endEffectorSide, FramePose endEffectorPose)
   {
      PoseReferenceFrame endEffectorPoseReferenceFrame = new PoseReferenceFrame("endEffectorPoseReferenceFrame", endEffectorPose);
      taskEndEffectorPose.get(endEffectorSide).setBodyToControl( getGripperPalmLinkName(endEffectorSide));

      setEndEffectorTarget(actualRobotModel, endEffectorSide, endEffectorPoseReferenceFrame);
   }

   private void setEndEffectorTarget(SDFFullRobotModel actualRobotModel, RobotSide endEffectorSide, ReferenceFrame endEffectorPose)
   {
      movePelvisToHaveOverlappingFeet( actualRobotModel, workingSdfModel );

      final HierarchicalTask_BodyPose handTask = taskEndEffectorPose.get(endEffectorSide);

      RigidBodyTransform ee_transform;
      ee_transform = endEffectorPose.getTransformToDesiredFrame( getRootFrame( actualRobotModel ));

      keepArmQuiet.get(endEffectorSide).setValue( false );

      RobotSide steadySide = endEffectorSide.getOppositeSide();

      hierarchicalSolver.prioritizeTasks(taskEndEffectorPose.get(steadySide), handTask);

      Quat4d quat = new Quat4d();
      Vector3d pos = new Vector3d();
      ee_transform.get(quat, pos);

      handTask.setTarget( quat, pos); 
   }

   private void enforceControlledDoF( ComputeOption opt )
   {
      double rotationWeight = 0.5;

      for ( RobotSide side: RobotSide.values)
      {
         HierarchicalTask_BodyPose handTask = taskEndEffectorPose.get(side);

         switch( controlledDoF.get(side) )
         {
         case DOF_NONE:
            handTask.setEnabled(false);
            break;

         case DOF_3P:
            handTask.setEnabled(true);

            if( opt == null || opt != ComputeOption.RESEED)
               handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, 0.05, 0.05, 0.05) );
            else
               handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, 0.0, 0.0, 0.0) );

            handTask.setWeightError( new Vector64F(6, 1, 1, 1,  0,0,0) );
            break;

         case DOF_3P2R:
            handTask.setEnabled(true);
            handTask.setWeightError( new Vector64F(6, 1,1,1,  1,1,1 ) );

            Vector64F target = handTask.getTarget();
            Matrix3d rot = new Matrix3d();
            rot.set( new Quat4d( target.get(3),  target.get(4), target.get(5), target.get(6) ) );
            handTask.disableAxisInTaskSpace(rotationWeight, new Vector3d(rot.m02, rot.m12, rot.m22)); 

            break;
         case DOF_3P3R:
            handTask.setEnabled(true);
            handTask.setWeightError( new Vector64F(6, 1,1,1,  1,1,1 ) );
            handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, rotationWeight, rotationWeight, rotationWeight) ); 
            break;   
         }
      }
   }


   public void setPreferedJointPose(String joint_name, double q)
   {
      int index = jointNamesToIndex.get(joint_name);
      preferedJointPose.set(index, q);
      taskJointsPose.setTarget(preferedJointPose );
   }

   private void checkIfArmShallStayQuiet(Vector64F q_init)
   {
      DenseMatrix64F weights_joint = taskJointsPose.getWeightMatrixTaskSpace();

      for( RobotSide side: RobotSide.values())
      {
         for (int jointId: armJointIds.get(side))
         {
            if ( keepArmQuiet.get(side).isTrue() )
            {
               weights_joint.set(jointId, 0.3);
               preferedJointPose.set(jointId, q_init.get(jointId));
            }
            else
            {
             //  RobotModel model = this.getHierarchicalSolver().getRobotModel(); 
             //  double preferredAngle = 0.5*(model.q_min(jointId)+ model.q_max(jointId));
               double preferredAngle = q_init.get(jointId);
               
               weights_joint.set(jointId, 0.05);
               preferedJointPose.set(jointId,preferredAngle);
            }
         }
      }
      taskJointsPose.setWeightsTaskSpace(weights_joint);
      taskJointsPose.setTarget(preferedJointPose);
   }

   private void checkIfLegsNeedToBeLocked()
   {
      taskComPosition.setEnabled(!lockLegs);
      taskLegPose.setEnabled(!lockLegs);

      double val = lockLegs ? 0 : 1;
      
      Vector64F weightsJoints    =  taskJointsPose.getWeightsJointSpace();
      Vector64F weightsCollision =  getHierarchicalSolver().collisionAvoidance.getJointWeights();
      Vector64F weightsEE_Left   =  taskEndEffectorPose.get(LEFT).getWeightsJointSpace();
      Vector64F weightsEE_Right  =  taskEndEffectorPose.get(RIGHT).getWeightsJointSpace();
      
      for (RobotSide side: RobotSide.values)
      {
         for (int index : legJointIds.get(side) )
         {
            weightsEE_Left.set(index, val);
            weightsEE_Right.set(index, val);
            weightsJoints.set(index, val);
            weightsCollision.set(index, val);
         }
      }     
   }


   public ComputeResult compute(SDFFullRobotModel actualSdfModel,  Map<String, Double> anglesToUseAsInitialState,  Map<String, Double> outputAngles) throws Exception
   {
      for (int i = 0; i < numOfJoints; i++)
      { 
         String activeJointName = urdfModel.getActiveJointName(i);
         Double newValue =  anglesToUseAsInitialState.get( activeJointName );

         if( newValue != null) {
            cachedAnglesQ.set(i, newValue.doubleValue());
         }
      }

      ComputeResult ret = compute(actualSdfModel, null,  ComputeOption.USE_JOINTS_CACHE);

      if( ret != ComputeResult.FAILED_INVALID)
      {
         for (int i = 0; i < numOfJoints; i++)
         {
            double new_value = cachedAnglesQ.get(i);
            String jointName =  urdfModel.getActiveJointName(i);
            outputAngles.put(jointName, new Double(new_value) );
         }
      }
      return ret;
   }

   public abstract HashMap<String,Double> getSuggestedAnglesForReseed();


   public void reseedCachedModel()
   {
      Vector64F newQ = getHierarchicalSolver().getRandomQ();

      HashMap<String,Double> suggestedQ = getSuggestedAnglesForReseed();

      for( Map.Entry<String,Double> entry: suggestedQ.entrySet())
      {
         int index = jointNamesToIndex.get(entry.getKey());
         newQ.set( index,  entry.getValue() );
      }

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         boolean partOfQuietArm = false;
         boolean partOfQuietLeg = false;

         for(RobotSide robotSide: RobotSide.values())
         {
            if( keepArmQuiet.get(robotSide).isTrue() &&  armJointIds.get(robotSide).contains(i))
            {
               partOfQuietArm = true;
            }
            if( lockLegs &&  legJointIds.get(robotSide).contains(i))
            {
               partOfQuietLeg = true;
            }
         }

         if( !partOfQuietArm && !partOfQuietLeg) {
            cachedAnglesQ.set(i, newQ.get(i));
         }
      }
   }


   public ComputeResult compute(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack, ComputeOption opt) throws Exception
   {
      if( desiredRobotModelToPack == actualSdfModel)
      {
         throw new Exception(" the output of compute should not be a reference to actual_sdf_model");
      }

      switch(opt)
      {
      case RESEED:

         reseedCachedModel();         

         break;
      case USE_ACTUAL_MODEL_JOINTS:
         actualSdfModel.updateFrames();
         for (int i = 0; i < numOfJoints; i++)
         {
            OneDoFJoint joint = actualSdfModel.getOneDoFJointByName(urdfModel.getActiveJointName(i));
            cachedAnglesQ.set(i, joint.getQ());
         }
         break;

      case USE_JOINTS_CACHE:
         //do nothing, the cache is used by default.
         break;
      case RELAX:
         // 
         break;
      }

      enforceControlledDoF( opt );

      ComputeResult ret = computeImpl(actualSdfModel, desiredRobotModelToPack, true );

      int reseedLeft = maxNumberOfAutomaticReseeds;

      while( reseedLeft > 0 && ret != ComputeResult.SUCCEEDED)
      {
         reseedCachedModel();
         ret = computeImpl(actualSdfModel, desiredRobotModelToPack, true );
         reseedLeft--;
      }

      if(opt != ComputeOption.RESEED)
      {
         keepArmQuiet.get(LEFT).setValue(true);
         keepArmQuiet.get(RIGHT).setValue(true);
      }

      return ret;
   }

   private void adjustDesiredCOM(SDFFullRobotModel actualSdfModel )
   {
      ReferenceFrame leftSole = actualSdfModel.getSoleFrame(LEFT);
      ReferenceFrame rightSole = actualSdfModel.getSoleFrame(RIGHT);
      RigidBodyTransform rightToLeftFoot = leftSole.getTransformToDesiredFrame( rightSole );
      Vector3d diff = new Vector3d();
      rightToLeftFoot.getTranslation(diff); 

      taskComPosition.setTarget( new Vector64F(3, diff.x/2, diff.y/2, 0) );
   }
   
   public double calculateCenterOfMassError(SDFFullRobotModel actualSdfModel )
   {
      ReferenceFrame leftSole = actualSdfModel.getSoleFrame(LEFT);
      ReferenceFrame rightSole = actualSdfModel.getSoleFrame(RIGHT);
      
      Vector3d leftFoot = new Vector3d();
      Vector3d rightFoot = new Vector3d();
      
      leftSole.getTransformToWorldFrame().getTranslation(leftFoot);
      rightSole.getTransformToWorldFrame().getTranslation(rightFoot);
      
      Vector3d desiredCOM =  new Vector3d( 
            (leftFoot.getX() + rightFoot.getX())*0.5,
            (leftFoot.getY() + rightFoot.getY())*0.5, 
            0.0 );
      
      CenterOfMassCalculator comCalculator = new CenterOfMassCalculator(actualSdfModel.getElevator(), ReferenceFrame.getWorldFrame() );
      comCalculator.compute();
      FramePoint calculatedCOM = comCalculator.getCenterOfMass();

      double x1 = calculatedCOM.getPoint().getX();
      double y1 = calculatedCOM.getPoint().getY();
      
      double x2 = desiredCOM.getX();
      double y2 = desiredCOM.getY();
      
      double dist = Math.sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)  );
      return dist;
   }

   private void movePelvisToHaveOverlappingFeet(SDFFullRobotModel referenceModel, SDFFullRobotModel followerModel)
   {
      referenceModel.updateFrames();
      followerModel.updateFrames();

      ReferenceFrame referenceSoleFrame = referenceModel.getSoleFrame( getSideOfTheFootRoot() );
      ReferenceFrame followerSoleFrame  = followerModel.getSoleFrame( getSideOfTheFootRoot() );

      RigidBodyTransform soleWorldToFollow = new RigidBodyTransform();
      RigidBodyTransform soleToPelvis      = new RigidBodyTransform();
      RigidBodyTransform pelvisTransform   = new RigidBodyTransform();

      ReferenceFrame followerPelvisFrame = followerModel.getRootJoint().getFrameAfterJoint();

      followerPelvisFrame.getTransformToDesiredFrame(soleToPelvis,  followerSoleFrame);
      referenceSoleFrame.getTransformToDesiredFrame(soleWorldToFollow, ReferenceFrame.getWorldFrame());
      pelvisTransform.multiply( soleWorldToFollow, soleToPelvis);

      followerModel.getRootJoint().setPositionAndRotation(pelvisTransform);
      followerModel.updateFrames();
   }


   synchronized private ComputeResult computeImpl(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack, boolean continueUntilPoseConverged)
   {    
      movePelvisToHaveOverlappingFeet( actualSdfModel, workingSdfModel );

      int ret = -1;
      Vector64F q_out = new Vector64F(numOfJoints);

      try{
         adjustDesiredCOM(actualSdfModel);
         checkIfLegsNeedToBeLocked();
         setPreferedKneeAngle();
         adjustOtherFoot(actualSdfModel);

         checkIfArmShallStayQuiet(cachedAnglesQ);

         q_out.set(cachedAnglesQ);
         ret = hierarchicalSolver.solve(cachedAnglesQ, q_out,continueUntilPoseConverged);

      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      ComputeResult result ;

      if( ret == -2) 
         result = ComputeResult.FAILED_INVALID;
      else if( ret == -1)
         result = ComputeResult.FAILED_NOT_CONVERGED;
      else 
         result = ComputeResult.SUCCEEDED;

      //------------------------------------------
      if (result !=ComputeResult.FAILED_INVALID )
      {
         cachedAnglesQ.set(q_out);

         HashMap<String, Double> listOfNewJointAngles = new HashMap<String, Double>();

         for (int i = 0; i < workingJointsInUrdfOrder.length; i++)
         {
            if (workingJointsInUrdfOrder[i] != null)
            {
               String jointName =  workingJointsInUrdfOrder[i].getName();
               double Q = q_out.get(i);
               workingJointsInUrdfOrder[i].setQ( Q );
               listOfNewJointAngles.put( jointName, Q );
            }
         }

         movePelvisToHaveOverlappingFeet( actualSdfModel, workingSdfModel );

         if(desiredRobotModelToPack != null)
         {
            InverseDynamicsJointStateCopier desiredModelJointCopier = new InverseDynamicsJointStateCopier(
                  workingSdfModel.getElevator(), 
                  desiredRobotModelToPack.getElevator());

            desiredModelJointCopier.copy();
            desiredRobotModelToPack.updateFrames();
         }
      }

      if(desiredRobotModelToPack != null)
      {
         movePelvisToHaveOverlappingFeet( actualSdfModel, desiredRobotModelToPack );
      }

      return result;
   }

   public double getDesiredJointAngle(String jointName)
   {
      int index = jointNamesToIndex.get(jointName);
      return workingJointsInUrdfOrder[index].getQ();
   }

   private void adjustOtherFoot(SDFFullRobotModel actualSdfModel )
   {
      RigidBodyTransform foot_other_transform = new RigidBodyTransform();
      // note before and after... it is not a typo.
      ReferenceFrame leftAnkle = actualSdfModel.getOneDoFJointByName("l_leg_akx").getFrameAfterJoint();
      //TODO. there might be something wrong here
      leftAnkle.getTransformToDesiredFrame(foot_other_transform, getRootFrame(actualSdfModel) );

      Quat4d quat = new Quat4d();
      Vector3d pos = new Vector3d();

      foot_other_transform.get(quat, pos);
      taskLegPose.setTarget(quat, pos);
   }


   public ReferenceFrame getDesiredAfterJointFrame(String jointName, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();

      ReferenceFrame workingJointFrame = workingSdfModel.getOneDoFJointByName(jointName).getFrameAfterJoint();
      ReferenceFrame workingRootFrame  = getRootFrame();

      RigidBodyTransform rootToJoint  = workingJointFrame.getTransformToDesiredFrame( workingRootFrame );
      RigidBodyTransform parentToRoot = workingRootFrame.getTransformToDesiredFrame( parentFrame );

      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToJoint);

      return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(jointName, parentFrame, parentToBody);
   }

   public void updateWorkingModel( SDFFullRobotModel sdfModel)
   {
      workingSdfModel.copyAllJointsButKeepOneFootFixed(sdfModel.getOneDoFJoints(), getSideOfTheFootRoot());
      movePelvisToHaveOverlappingFeet( sdfModel, workingSdfModel );
   }

   public ReferenceFrame getDesiredGripperAttachmentFrame(RobotSide handSide, ReferenceFrame parentFrame)
   {
      return getDesiredBodyFrame(getGripperAttachmentLinkName(handSide), parentFrame);
   }

   public ReferenceFrame getDesiredGripperPalmFrame(RobotSide handSide, ReferenceFrame parentFrame)
   {
      return getDesiredBodyFrame(getGripperPalmLinkName(handSide), parentFrame);
   }

   public ReferenceFrame getDesiredBodyFrame(String name, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();

      RigidBodyTransform rootToBody   = getLocalBodyTransform(name);
      RigidBodyTransform parentToRoot =  getRootFrame().getTransformToDesiredFrame(parentFrame);

      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToBody);

      return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, parentFrame, parentToBody);
   }


   public ControlledDoF getNumberOfControlledDoF(RobotSide side)
   {
      return controlledDoF.get(side);
   }

   public void setNumberOfControlledDoF(RobotSide side, ControlledDoF dof)
   {
      controlledDoF.set(side, dof );
      enforceControlledDoF( null );
   }

   public boolean isLowerBodyLocked()
   {
      return lockLegs;
   }

   public void lockLowerBody(boolean lockLowerBody)
   {
      this.lockLegs = lockLowerBody;
   }


}