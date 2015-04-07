package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class NewPelvisPoseHistoryCorrection implements PelvisPoseHistoryCorrectionInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean ENABLE_ROTATION_CORRECTION = false;
   
   private static final double DEFAULT_BREAK_FREQUENCY = 0.6;

   private final YoVariableRegistry registry;
   
   private PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame pelvisReferenceFrame;
   private final ClippedSpeedOffsetErrorInterpolator offsetErrorInterpolator;
   private final OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseUpdater;
   
   private final double estimatorDT;
   private boolean sendCorrectionUpdate = false;

   private final DoubleYoVariable alphaFilterBreakFrequency;
   
   private final DoubleYoVariable confidenceFactor;
   
   private final RigidBodyTransform stateEstimatorPelvisTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();
   
   private final RigidBodyTransform totalErrorBetweenPelvisAndLocalizationTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorBetweenCorrectedAndLocalizationTransform = new RigidBodyTransform();
   private final Vector3d totalErrorTranslation = new Vector3d(); 
   private final Quat4d totalErrorRotation = new Quat4d(); 
   private final DoubleYoVariable totalErrorTranslation_X;
   private final DoubleYoVariable totalErrorTranslation_Y;
   private final DoubleYoVariable totalErrorTranslation_Z;
   private final double[] totalErrorYawPitchRoll = new double[3];
   private final DoubleYoVariable totalErrorRotation_Yaw;
   private final DoubleYoVariable totalErrorRotation_Pitch;
   private final DoubleYoVariable totalErrorRotation_Roll;
   
   private final IntegerYoVariable pelvisBufferSize;

   private final FramePose stateEstimatorInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoStateEstimatorInWorldFramePose;
   private final YoFramePose yoCorrectedPelvisPoseInWorldFrame;
   private final FramePose iterativeClosestPointInWorldFramePose = new FramePose(worldFrame);
   private final YoFramePose yoIterativeClosestPointPoseInWorldFrame;
   
   private final ReferenceFrame iterativeClosestPointReferenceFrame;
   private final FramePose correctedPelvisPoseInWorldFrame = new FramePose(worldFrame);
   
   private final BooleanYoVariable hasOneIcpPacketEverBeenReceived;
   
   private final Vector3d localizationTranslation = new Vector3d();
   private final Vector3d correctedPelvisTranslation = new Vector3d();
   private final Vector3d errorBetweenCorrectedAndLocalizationTransform_Translation = new Vector3d();
   
   private final FrameOrientation localizationOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation correctedPelvisOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation errorBetweenCorrectedAndLocalizationTransform_Rotation = new FrameOrientation(worldFrame);
   private final Quat4d errorBetweenCorrectedAndLocalizationQuaternion_Rotation = new Quat4d();
   
   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure, final double dt, YoVariableRegistry parentRegistry,
         int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, null);
   }

   public NewPelvisPoseHistoryCorrection(FullInverseDynamicsStructure inverseDynamicsStructure,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, final double dt, YoVariableRegistry parentRegistry, int pelvisBufferSize)
   {
      this(inverseDynamicsStructure.getRootJoint(), dt, parentRegistry, pelvisBufferSize, externalPelvisPoseSubscriber);
   }
   
   public NewPelvisPoseHistoryCorrection(SixDoFJoint sixDofJoint, final double estimatorDT, YoVariableRegistry parentRegistry, int pelvisBufferSize,
         PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.estimatorDT = estimatorDT;

      this.rootJoint = sixDofJoint;
      this.pelvisReferenceFrame = rootJoint.getFrameAfterJoint();
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
      this.registry = new YoVariableRegistry("newPelvisPoseHistoryCorrection");
      parentRegistry.addChild(registry);
      
      this.pelvisBufferSize = new IntegerYoVariable("pelvisBufferSize", registry);
      this.pelvisBufferSize.set(pelvisBufferSize);
      
      alphaFilterBreakFrequency = new DoubleYoVariable("alphaFilterBreakFrequency", registry);
      alphaFilterBreakFrequency.set(DEFAULT_BREAK_FREQUENCY);
      
      confidenceFactor = new DoubleYoVariable("PelvisErrorCorrectionConfidenceFactor", registry);
      
      offsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, pelvisReferenceFrame, alphaFilterBreakFrequency, this.estimatorDT, ENABLE_ROTATION_CORRECTION);
      outdatedPoseUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(pelvisBufferSize, pelvisReferenceFrame);
      
      iterativeClosestPointReferenceFrame = outdatedPoseUpdater.getOutdatedReferenceFrameToBeUpdated();
      
      //used only for feedback in SCS
      yoStateEstimatorInWorldFramePose = new YoFramePose("stateEstimatorInWorldFramePose", worldFrame, registry);
      yoCorrectedPelvisPoseInWorldFrame = new YoFramePose("correctedPelvisPoseInWorldFrame", worldFrame, registry);
      yoIterativeClosestPointPoseInWorldFrame = new YoFramePose("iterativeClosestPointPoseInWorldFrame", worldFrame, registry);
      
      totalErrorTranslation_X = new DoubleYoVariable("totalErrorTranslation_X", registry);
      totalErrorTranslation_Y = new DoubleYoVariable("totalErrorTranslation_Y", registry);
      totalErrorTranslation_Z = new DoubleYoVariable("totalErrorTranslation_Z", registry);
      totalErrorRotation_Yaw = new DoubleYoVariable("totalErrorRotation_Yaw", registry);
      totalErrorRotation_Pitch = new DoubleYoVariable("totalErrorRotation_Pitch", registry);
      totalErrorRotation_Roll = new DoubleYoVariable("totalErrorRotation_Roll", registry);
      
      hasOneIcpPacketEverBeenReceived = new BooleanYoVariable("hasOneIcpPacketEverBeenReceived", registry);
      hasOneIcpPacketEverBeenReceived.set(false);
   }
   
   public void doControl(long timestamp)
   {
      if (pelvisPoseCorrectionCommunicator != null)
      {
         pelvisReferenceFrame.update();
         checkForNewPacket();

         pelvisReferenceFrame.getTransformToDesiredFrame(stateEstimatorPelvisTransformInWorld, worldFrame);
         outdatedPoseUpdater.putUpToDateTransformInBuffer(stateEstimatorPelvisTransformInWorld, timestamp);
         
         offsetErrorInterpolator.interpolateError(correctedPelvisPoseInWorldFrame);
         /////for SCS feedback
         stateEstimatorInWorldFramePose.setPose(stateEstimatorPelvisTransformInWorld);
         yoStateEstimatorInWorldFramePose.set(stateEstimatorInWorldFramePose);
         /////
         
         updateCorrectedPelvis();
         pelvisReferenceFrame.update();
         checkForNeedToSendCorrectionUpdate();
      }
   }

   private void updateCorrectedPelvis()
   {
      if(!hasOneIcpPacketEverBeenReceived.getBooleanValue())
         correctedPelvisPoseInWorldFrame.setPose(stateEstimatorPelvisTransformInWorld);
      
      correctedPelvisPoseInWorldFrame.getPose(correctedPelvisTransformInWorldFrame);
      
      correctedPelvisTransformInWorldFrame.getTranslation(correctedPelvisTranslation);
      iterativeClosestPointInWorldFramePose.getPosition(localizationTranslation);
      
      iterativeClosestPointInWorldFramePose.getOrientationIncludingFrame(localizationOrientation);
      correctedPelvisOrientation.setIncludingFrame(worldFrame, correctedPelvisTransformInWorldFrame);
      
      errorBetweenCorrectedAndLocalizationTransform_Rotation.setOrientationFromOneToTwo(localizationOrientation, correctedPelvisOrientation);
      errorBetweenCorrectedAndLocalizationTransform_Rotation.getQuaternion(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);

      errorBetweenCorrectedAndLocalizationTransform_Translation.sub(localizationTranslation, correctedPelvisTranslation);
      
      ////// for SCS feedback
      yoCorrectedPelvisPoseInWorldFrame.set(correctedPelvisPoseInWorldFrame);
      //////
      errorBetweenCorrectedAndLocalizationTransform.setTranslation(errorBetweenCorrectedAndLocalizationTransform_Translation);
      errorBetweenCorrectedAndLocalizationTransform.setRotation(errorBetweenCorrectedAndLocalizationQuaternion_Rotation);
      
      rootJoint.setPositionAndRotation(correctedPelvisTransformInWorldFrame);
   }

   /**
    * poll for new packet, input for unit tests and real robot
    */
   private void checkForNewPacket()
   {
      if (pelvisPoseCorrectionCommunicator.hasNewPose())
      {
         processNewPacket();
         sendCorrectionUpdate = true;
      }
   }

   /**
    * pulls the corrected pose from the buffer, check that the nonprocessed buffer has
    * corresponding pelvis poses and calculates the total error
    */
   private void processNewPacket()
   {
      StampedPosePacket newPacket = pelvisPoseCorrectionCommunicator.getNewExternalPose();
      TimeStampedTransform3D timeStampedExternalPose = newPacket.getTransform();

      if (outdatedPoseUpdater.upToDateTimeStampedBufferIsInRange(timeStampedExternalPose.getTimeStamp()))
      {
         if(!hasOneIcpPacketEverBeenReceived.getBooleanValue())
            hasOneIcpPacketEverBeenReceived.set(true);
         double confidence = newPacket.getConfidenceFactor();
         confidence = MathTools.clipToMinMax(confidence, 0.0, 1.0);
         confidenceFactor.set(confidence);
         addNewExternalPose(timeStampedExternalPose);
      }
   }

   private void addNewExternalPose(TimeStampedTransform3D timeStampedExternalPose)
   {
      outdatedPoseUpdater.updateOutdatedTransform(timeStampedExternalPose);
      iterativeClosestPointReferenceFrame.update();
      iterativeClosestPointInWorldFramePose.setToZero(iterativeClosestPointReferenceFrame);
      iterativeClosestPointInWorldFramePose.changeFrame(worldFrame);
      ////for SCS feedback
      yoIterativeClosestPointPoseInWorldFrame.set(iterativeClosestPointInWorldFramePose);
      
//      iterativeClosestPointInWorldFramePose.getPose(totalErrorBetweenPelvisAndLocalizationTransform); // TODO verify that
      outdatedPoseUpdater.getTotalErrorTransform(totalErrorBetweenPelvisAndLocalizationTransform);
      
      ////for SCS feedback
      totalErrorBetweenPelvisAndLocalizationTransform.getTranslation(totalErrorTranslation);
      totalErrorTranslation_X.set(totalErrorTranslation.getX());
      totalErrorTranslation_Y.set(totalErrorTranslation.getY());
      totalErrorTranslation_Z.set(totalErrorTranslation.getZ());
      totalErrorBetweenPelvisAndLocalizationTransform.getRotation(totalErrorRotation);
      RotationFunctions.setYawPitchRollBasedOnQuaternion(totalErrorYawPitchRoll, totalErrorRotation);
      totalErrorRotation_Yaw.set(totalErrorYawPitchRoll[0]);
      totalErrorRotation_Pitch.set(totalErrorYawPitchRoll[1]);
      totalErrorRotation_Roll.set(totalErrorYawPitchRoll[2]);
      /////
      
      offsetErrorInterpolator.setInterpolatorInputs(correctedPelvisPoseInWorldFrame, iterativeClosestPointInWorldFramePose, confidenceFactor.getDoubleValue());
   }
   
   private void checkForNeedToSendCorrectionUpdate()
   {
      if (sendCorrectionUpdate)
      {
         sendCorrectionUpdatePacket();
         sendCorrectionUpdate = false;
      }
   }
   
   private void sendCorrectionUpdatePacket()
   {
      PelvisPoseErrorPacket pelvisPoseErrorPacket = new PelvisPoseErrorPacket(totalErrorBetweenPelvisAndLocalizationTransform, errorBetweenCorrectedAndLocalizationTransform);
      pelvisPoseCorrectionCommunicator.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
   }
   
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.pelvisPoseCorrectionCommunicator = externalPelvisPoseSubscriber;
   }
}