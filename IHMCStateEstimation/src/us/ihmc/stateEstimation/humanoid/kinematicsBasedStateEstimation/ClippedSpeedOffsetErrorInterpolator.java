package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class ClippedSpeedOffsetErrorInterpolator
{
   private static final double MAX_TRANSLATIONAL_CORRECTION_SPEED = 0.05;
   private static final double MAX_ROTATIONAL_CORRECTION_SPEED = 0.05;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanYoVariable isRotationCorrectionEnabled;

   private final YoVariableRegistry registry;

   private final BooleanYoVariable hasBeenCalled;

   private final DoubleYoVariable alphaFilterBreakFrequency;
   private final DoubleYoVariable dt;

   private final Vector3d startTranslation = new Vector3d();
   private final Vector3d interpolatedTranslation = new Vector3d();
   private final Vector3d goalTranslation = new Vector3d();
   private final RigidBodyTransform interpolatedTransform_Translation = new RigidBodyTransform();
   
   private final FrameOrientation startRotation = new FrameOrientation(worldFrame);
   private final Quat4d startRotation_quat = new Quat4d();
   private final Quat4d interpolatedRotation = new Quat4d();
   private final FrameOrientation interpolatedRotationFrameOrientation = new FrameOrientation(worldFrame);
   private final FrameOrientation goalRotation = new FrameOrientation(worldFrame);
   private final Quat4d goalRotation_quat = new Quat4d();
   private final RigidBodyTransform interpolatedTransform_Rotation = new RigidBodyTransform();
   
   private final RigidBodyTransform interpolatedTransform = new RigidBodyTransform(); 
   
   private final AlphaFilteredYoVariable alphaFilter;
   private final DoubleYoVariable alphaFilter_AlphaValue;
   private final DoubleYoVariable alphaFilter_PositionValue;
   private final DoubleYoVariable cLippedAlphaFilterValue;
   private final DoubleYoVariable previousClippedAlphaFilterValue;

   private final DoubleYoVariable maxTranslationalCorrectionVelocity;
   private final DoubleYoVariable maxRotationalCorrectionVelocity;

   private final FramePose startOffsetErrorPose = new FramePose(worldFrame);
   private final FramePose goalOffsetErrorPose = new FramePose(worldFrame);
   private final Vector3d distanceToTravelVector = new Vector3d();
   private final DoubleYoVariable distanceToTravel;
   private final FrameOrientation rotationToTravel = new FrameOrientation(worldFrame);

   private final DoubleYoVariable angleToTravel;

   private final DoubleYoVariable translationalSpeedForGivenDistanceToTravel;
   private final DoubleYoVariable rotationalSpeedForGivenAngleToTravel;

   private final DoubleYoVariable temporaryTranslationAlphaClipped;
   private final DoubleYoVariable temporaryRotationAlphaClipped;

   private final ReferenceFrame referenceFrameToBeCorrectedTransform;
   private final RigidBodyTransform referenceFrameToBeCorrectedTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform referenceFrameToBeCorrectedTransform_Rotation = new RigidBodyTransform();
   
   private final Vector3d referenceFrameToBeCorrected_Translation = new Vector3d();
   private final Vector3d correctionToBeApplied_Translation = new Vector3d();
   private final Quat4d referenceFrameToBeCorrected_Rotation = new Quat4d();
   private final FrameOrientation correctionToBeAppliedFrameOrientation_Rotation = new FrameOrientation(worldFrame);

   private final Vector3d previousReferenceFrameToBeCorrected_Translation = new Vector3d(); 
   private final Vector3d pelvisTranslationBetweenTwoTicks = new Vector3d();
   private final FrameOrientation referenceFrameToBeCorrectedFrameOrientation_Rotation = new FrameOrientation(worldFrame);
   private final FrameOrientation previousReferenceFrameToBeCorrectedFrameOrientation_Rotation = new FrameOrientation(worldFrame);
   private final FrameOrientation pelvisRotationBetweenTwoTicks = new FrameOrientation(worldFrame);
   
   private final AxisAngle4d axisAngletoTravel = new AxisAngle4d();
   
   //for feedBack in scs
   private final YoFramePose yoStartOffsetErrorPose;
   private final YoFramePose yoGoalOffsetErrorPose;
   private final YoFramePose yoInterpolatedOffset;
   
   public ClippedSpeedOffsetErrorInterpolator(YoVariableRegistry parentRegistry, ReferenceFrame referenceFrame, DoubleYoVariable alphaFilterBreakFrequency,
         double dt, boolean correctRotation)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.alphaFilterBreakFrequency = alphaFilterBreakFrequency;
      this.dt = new DoubleYoVariable("dt", registry);
      this.dt.set(dt);

      this.referenceFrameToBeCorrectedTransform = referenceFrame;

      isRotationCorrectionEnabled = new BooleanYoVariable("isRotationCorrectionEnabled", registry);
      isRotationCorrectionEnabled.set(correctRotation);

      hasBeenCalled = new BooleanYoVariable("hasbeenCalled", registry);
      hasBeenCalled.set(false);

      alphaFilter_AlphaValue = new DoubleYoVariable("alphaFilter_AlphaValue", registry);
      alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFilterBreakFrequency.getDoubleValue(),
            this.dt.getDoubleValue()));
      alphaFilter_PositionValue = new DoubleYoVariable("alphaFilter_PositionValue", registry);
      alphaFilter_PositionValue.set(0.0);
      alphaFilter = new AlphaFilteredYoVariable("alphaFilter", registry, alphaFilter_AlphaValue, alphaFilter_PositionValue);

      cLippedAlphaFilterValue = new DoubleYoVariable("cLippedAlphaFilterValue", registry);
      cLippedAlphaFilterValue.set(0.0);
      previousClippedAlphaFilterValue = new DoubleYoVariable("previousClippedAlphaFilterValue", registry);
      previousClippedAlphaFilterValue.set(0.0);

      maxTranslationalCorrectionVelocity = new DoubleYoVariable("maxTranslationalCorrectionVelocity", parentRegistry);
      maxTranslationalCorrectionVelocity.set(MAX_TRANSLATIONAL_CORRECTION_SPEED * this.dt.getDoubleValue());
      maxRotationalCorrectionVelocity = new DoubleYoVariable("maxRotationalCorrectionVelocity", parentRegistry);
      maxRotationalCorrectionVelocity.set(MAX_ROTATIONAL_CORRECTION_SPEED * this.dt.getDoubleValue());

      distanceToTravel = new DoubleYoVariable("distanceToTravel", registry);
      distanceToTravel.set(0.0);
      angleToTravel = new DoubleYoVariable("angleToTravel", registry);
      angleToTravel.set(0.0);

      translationalSpeedForGivenDistanceToTravel = new DoubleYoVariable("translationalSpeedForGivenDistanceToTravel", registry);
      rotationalSpeedForGivenAngleToTravel = new DoubleYoVariable("rotationalSpeedForGivenAngleToTravel", registry);

      temporaryTranslationAlphaClipped = new DoubleYoVariable("temporaryTranslationAlphaClipped", registry);
      temporaryRotationAlphaClipped = new DoubleYoVariable("temporaryRotationAlphaClipped", registry);

      // for feedback in SCS
      yoStartOffsetErrorPose = new YoFramePose("yoStartOffsetErrorPose", worldFrame, registry);
      yoGoalOffsetErrorPose = new YoFramePose("yoGoalOffsetErrorPose", worldFrame, registry);
      yoInterpolatedOffset = new YoFramePose("yoInterpolatedOffset", worldFrame, registry);
   }

   public void setInterpolatorInputs(FramePose startOffsetError, FramePose goalOffsetError, double alphaFilterPosition)
   {
      startOffsetErrorPose.setPoseIncludingFrame(startOffsetError);
      goalOffsetErrorPose.setPoseIncludingFrame(goalOffsetError);
      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         startOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
         goalOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
      }

      //scs feedback only
      yoStartOffsetErrorPose.set(startOffsetErrorPose);
      yoGoalOffsetErrorPose.set(goalOffsetErrorPose);
      ///////////

      startOffsetErrorPose.getPosition(startTranslation);
      startOffsetErrorPose.getOrientationIncludingFrame(startRotation);
      
      goalOffsetErrorPose.getPosition(goalTranslation);
      goalOffsetErrorPose.getOrientationIncludingFrame(goalRotation);
      
      alphaFilter_PositionValue.set(alphaFilterPosition);
      alphaFilter.set(0.0);
      previousClippedAlphaFilterValue.set(0.0);
      cLippedAlphaFilterValue.set(0.0);

      updateMaxAlphaVariationSpeed();

      hasBeenCalled.set(false);
   }

   private void updateMaxAlphaVariationSpeed()
   {
      distanceToTravelVector.sub(goalTranslation, startTranslation);
      
      distanceToTravel.set(distanceToTravelVector.length());
      translationalSpeedForGivenDistanceToTravel.set(distanceToTravel.getDoubleValue() / dt.getDoubleValue());
      
      rotationToTravel.setOrientationFromOneToTwo(startRotation, goalRotation);
      rotationToTravel.getAxisAngle(axisAngletoTravel);
      
      angleToTravel.set(axisAngletoTravel.getAngle());
      rotationalSpeedForGivenAngleToTravel.set(angleToTravel.getDoubleValue() / dt.getDoubleValue());
   }

   public void interpolateError(FramePose offsetPoseToPack)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         alphaFilter.update(0.0);
         hasBeenCalled.set(true);
      }
      else
      {
         alphaFilter.update();
      }
      
      //translation
      if (translationalSpeedForGivenDistanceToTravel.getDoubleValue() * alphaFilter.getDoubleValue() <= translationalSpeedForGivenDistanceToTravel
            .getDoubleValue()
            * previousClippedAlphaFilterValue.getDoubleValue()
            + (maxTranslationalCorrectionVelocity.getDoubleValue() / distanceToTravel.getDoubleValue()))
      {
         temporaryTranslationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryTranslationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxTranslationalCorrectionVelocity.getDoubleValue() / distanceToTravel.getDoubleValue()));
      }

      //rotation
      if (rotationalSpeedForGivenAngleToTravel.getDoubleValue() * alphaFilter.getDoubleValue() <= rotationalSpeedForGivenAngleToTravel.getDoubleValue()
            * previousClippedAlphaFilterValue.getDoubleValue() + (maxRotationalCorrectionVelocity.getDoubleValue() / angleToTravel.getDoubleValue()))
      {
         temporaryRotationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryRotationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxRotationalCorrectionVelocity.getDoubleValue() / angleToTravel.getDoubleValue()));
      }

      //chose the smaller alpha (so the slower correction) between translation and rotation
      cLippedAlphaFilterValue.set(Math.min(temporaryTranslationAlphaClipped.getDoubleValue(), temporaryRotationAlphaClipped.getDoubleValue()));
      previousClippedAlphaFilterValue.set(cLippedAlphaFilterValue.getDoubleValue());

      referenceFrameToBeCorrectedTransform.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Translation, worldFrame);
      referenceFrameToBeCorrectedTransform_Translation.setRotationToIdentity();
      referenceFrameToBeCorrectedTransform.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Rotation, worldFrame);
      referenceFrameToBeCorrectedTransform_Rotation.zeroTranslation();
      
      referenceFrameToBeCorrectedTransform_Translation.getTranslation(referenceFrameToBeCorrected_Translation);
      referenceFrameToBeCorrectedTransform_Rotation.getRotation(referenceFrameToBeCorrected_Rotation);
      referenceFrameToBeCorrectedFrameOrientation_Rotation.set(referenceFrameToBeCorrected_Rotation);
      
      pelvisTranslationBetweenTwoTicks.sub(referenceFrameToBeCorrected_Translation, previousReferenceFrameToBeCorrected_Translation);
      pelvisRotationBetweenTwoTicks.setOrientationFromOneToTwo(referenceFrameToBeCorrectedFrameOrientation_Rotation, previousReferenceFrameToBeCorrectedFrameOrientation_Rotation);
      RigidBodyTransform pelvisRotationBetweenTwoTicksTransform = new RigidBodyTransform(); 
      pelvisRotationBetweenTwoTicks.getTransform3D(pelvisRotationBetweenTwoTicksTransform);
      
      startTranslation.add(pelvisTranslationBetweenTwoTicks);
      goalTranslation.add(pelvisTranslationBetweenTwoTicks);
      
      if(isRotationCorrectionEnabled.getBooleanValue())
      {
         startRotation.applyTransform(pelvisRotationBetweenTwoTicksTransform);
         startRotation.getQuaternion(startRotation_quat);
         goalRotation.applyTransform(pelvisRotationBetweenTwoTicksTransform);
         goalRotation.getQuaternion(goalRotation_quat);
      }
      else
      {
         startRotation_quat.set(referenceFrameToBeCorrected_Rotation);
         goalRotation_quat.set(referenceFrameToBeCorrected_Rotation);
      }
      
      interpolatedTranslation.interpolate(startTranslation, goalTranslation, cLippedAlphaFilterValue.getDoubleValue());
      interpolatedRotation.interpolate(startRotation_quat, goalRotation_quat, cLippedAlphaFilterValue.getDoubleValue());

      interpolatedRotationFrameOrientation.set(interpolatedRotation);
      
      
      correctionToBeApplied_Translation.sub(referenceFrameToBeCorrected_Translation, interpolatedTranslation);
      correctionToBeApplied_Translation.negate();
      correctionToBeAppliedFrameOrientation_Rotation.setOrientationFromOneToTwo(interpolatedRotationFrameOrientation, referenceFrameToBeCorrectedFrameOrientation_Rotation);
      
      interpolatedTransform_Translation.setTranslationAndIdentityRotation(correctionToBeApplied_Translation);
      correctionToBeAppliedFrameOrientation_Rotation.getTransform3D(interpolatedTransform_Rotation);
      
      interpolatedTransform.setIdentity();
      interpolatedTransform.multiply(referenceFrameToBeCorrectedTransform_Translation);
      interpolatedTransform.multiply(interpolatedTransform_Translation);
      interpolatedTransform.multiply(referenceFrameToBeCorrectedTransform_Rotation);
      interpolatedTransform.multiply(interpolatedTransform_Rotation);
      
      previousReferenceFrameToBeCorrected_Translation.set(referenceFrameToBeCorrected_Translation);
      previousReferenceFrameToBeCorrectedFrameOrientation_Rotation.set(referenceFrameToBeCorrected_Rotation);
      
      offsetPoseToPack.setPose(interpolatedTransform);

      //scs feedback only
      yoInterpolatedOffset.set(offsetPoseToPack);
   }
}