package us.ihmc.quadrupedRobotics.footstepChooser;

import us.ihmc.quadrupedRobotics.parameters.SwingTargetGeneratorParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class MidFootZUpSwingTargetGenerator implements SwingTargetGenerator
{
   private final double MINIMUM_VELOCITY_FOR_FULL_SKEW = 0.1;
   public static double MINIMUM_DISTANCE_FROM_SAMESIDE_FOOT = 0.04;
   public static double DEFAULT_STRIDE_LENGTH = 1.373;//0.34;
   public static double DEFAULT_STANCE_WIDTH = 0.36922 * 2;//0.24;
   public static double DEFAULT_MAX_SKEW = 0.1;
   public static double DEFAULT_MAX_YAW = 0.25;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonQuadrupedReferenceFrames referenceFrames;
   private final DoubleYoVariable minimumVelocityForFullSkew = new DoubleYoVariable("minimumVelocityForFullSkew", registry);
   private final DoubleYoVariable strideLength = new DoubleYoVariable("strideLength", registry);
   private final DoubleYoVariable stanceWidth = new DoubleYoVariable("stanceWidth", registry);
   private final DoubleYoVariable maxForwardSkew = new DoubleYoVariable("maxForwardSkew", registry);
   private final DoubleYoVariable maxLateralSkew = new DoubleYoVariable("maxLateralSkew", registry);
   private final DoubleYoVariable maxYawPerStep = new DoubleYoVariable("maxYawPerStep", registry);
   private final DoubleYoVariable minimumDistanceFromSameSideFoot = new DoubleYoVariable("minimumDistanceFromSameSideFoot", registry);
   private final DoubleYoVariable xOffsetFromCenterOfHips = new DoubleYoVariable("xOffsetFromCenterOfHips", registry);
   private final DoubleYoVariable yOffsetFromCenterOfHips = new DoubleYoVariable("yOffsetFromCenterOfHips", registry);
   private final QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final FramePoint centroid = new FramePoint(ReferenceFrame.getWorldFrame());

   private final FramePoint swingLegHipPitchPoint = new FramePoint();
   private final FrameOrientation swingLegHipRollOrientation = new FrameOrientation();

   private final FramePoint desiredSwingFootPositionFromHalfStride = new FramePoint();
   private final FramePoint desiredSwingFootPositionFromOppositeSideFoot = new FramePoint();

   public MidFootZUpSwingTargetGenerator(SwingTargetGeneratorParameters footStepParameters, CommonQuadrupedReferenceFrames referenceFrames,
         YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
      
      if (footStepParameters != null)
      {
         minimumDistanceFromSameSideFoot.set(footStepParameters.getMinimumDistanceFromSameSideFoot());
         minimumVelocityForFullSkew.set(footStepParameters.getMinimumVelocityForFullSkew());
         strideLength.set(footStepParameters.getStanceLength());
         stanceWidth.set(footStepParameters.getStanceWidth());
         maxForwardSkew.set(footStepParameters.getMaxForwardSkew());
         maxLateralSkew.set(footStepParameters.getMaxLateralSkew());
         maxYawPerStep.set(footStepParameters.getMaxYawPerStep());
      }
      else
      {
         minimumDistanceFromSameSideFoot.set(MINIMUM_DISTANCE_FROM_SAMESIDE_FOOT);
         minimumVelocityForFullSkew.set(MINIMUM_VELOCITY_FOR_FULL_SKEW);
         strideLength.set(DEFAULT_STRIDE_LENGTH);
         stanceWidth.set(DEFAULT_STANCE_WIDTH);
         maxForwardSkew.set(DEFAULT_MAX_SKEW);
         maxYawPerStep.set(DEFAULT_MAX_YAW);
      }
   }

   @Override
   public void getSwingTarget(RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack, double desiredYawRate)
   {
      updateFeetPositions();
      RobotSide oppositeSide = swingLeg.getOppositeSide();
      ReferenceFrame oppositeSideZUpFrame = referenceFrames.getSideDependentMidFeetZUpFrame(oppositeSide);
      calculateSwingTarget(supportPolygon, oppositeSideZUpFrame, swingLeg, desiredBodyVelocity, desiredYawRate, swingTargetToPack);
   }

   private TranslationReferenceFrame hindFoot = new TranslationReferenceFrame("backFoot", ReferenceFrame.getWorldFrame());
   private TranslationReferenceFrame frontFoot = new TranslationReferenceFrame("frontFoot", ReferenceFrame.getWorldFrame());
   private MidFrameZUpFrame midFeetZUpFrame = new MidFrameZUpFrame("MidFeetZUpFrame", ReferenceFrame.getWorldFrame(), hindFoot, frontFoot);
   
   @Override
   public void getSwingTarget(QuadrupedSupportPolygon footPostions, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, FramePoint swingTargetToPack,
         double desiredYawRate)
   {
      FramePoint across = footPostions.getFootstep(swingLeg.getAcrossBodyQuadrant());
      FramePoint diag = footPostions.getFootstep(swingLeg.getDiagonalOppositeQuadrant());

      if(swingLeg.isQuadrantInFront())
      {
         frontFoot.updateTranslation(across);
         hindFoot.updateTranslation(diag);
      }
      else
      {
         frontFoot.updateTranslation(diag);
         hindFoot.updateTranslation(across);
      }
      midFeetZUpFrame.update();
      
      calculateSwingTarget(footPostions, midFeetZUpFrame, swingLeg, desiredBodyVelocity, desiredYawRate, swingTargetToPack);
   }

   private void calculateSwingTarget(QuadrupedSupportPolygon supportPolygon, ReferenceFrame oppositeSideZUpFrame, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double desiredYawRate,
         FramePoint swingTargetToPack)
   {
      //calculate hipPitchHeight
      swingLegHipPitchPoint.setToZero(referenceFrames.getHipPitchFrame(swingLeg));
      swingLegHipPitchPoint.changeFrame(ReferenceFrame.getWorldFrame());
      double swingLegHipPitchHeight = swingLegHipPitchPoint.getZ();

      //calculate hip Roll
      swingLegHipRollOrientation.setToZero(referenceFrames.getHipRollFrame(swingLeg));
      swingLegHipRollOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double stepDistanceRemovedBecauseOfRoll = referenceFrames.getLegLength(swingLeg) * Math.sin(Math.abs(swingLegHipRollOrientation.getRoll()));

      double maxStepDistance;
      double maxStepDistanceWithNoRoll = Math.sqrt(Math.pow(referenceFrames.getLegLength(swingLeg), 2) - Math.pow(swingLegHipPitchHeight, 2));
      if (Double.isNaN(maxStepDistanceWithNoRoll) || Double.isNaN(stepDistanceRemovedBecauseOfRoll))
      {
         maxStepDistance = 0.0;
      }
      else
      {
         maxStepDistance = maxStepDistanceWithNoRoll - stepDistanceRemovedBecauseOfRoll;
         maxStepDistance = Math.max(maxStepDistance, 0.0);
      }

      double deltaYaw = MathTools.clipToMinMax(desiredYawRate, maxYawPerStep.getDoubleValue());

      RobotQuadrant sameSideQuadrant = swingLeg.getSameSideQuadrant();
      RobotQuadrant sameEndQuadrant = swingLeg.getAcrossBodyQuadrant();

      FramePoint footPositionSameSideOppositeEnd =new FramePoint(supportPolygon.getFootstep(sameSideQuadrant));
      FramePoint footPositionOppositeSideSameEnd = new FramePoint(supportPolygon.getFootstep(sameEndQuadrant));

      //midZUpFrame is oriented so X is perpendicular to the two same side feet, Y pointing backward
      determineFootPositionFromHalfStride(supportPolygon, swingLeg, desiredBodyVelocity, maxStepDistance, deltaYaw, footPositionSameSideOppositeEnd,
            footPositionSameSideOppositeEnd, oppositeSideZUpFrame);

      determineFootPositionFromOppositeSideFoot(supportPolygon, swingLeg, desiredBodyVelocity, maxStepDistance, deltaYaw, footPositionSameSideOppositeEnd,
            footPositionOppositeSideSameEnd, oppositeSideZUpFrame);
      
      //pack the destination with 20% of the position from halfStride and 80% of the position from the opposite side foot
      desiredSwingFootPositionFromHalfStride.scale(0.2);
      desiredSwingFootPositionFromOppositeSideFoot.scale(0.8);
      
      swingTargetToPack.set(desiredSwingFootPositionFromHalfStride);
      swingTargetToPack.add(desiredSwingFootPositionFromOppositeSideFoot);
      swingTargetToPack.add(xOffsetFromCenterOfHips.getDoubleValue(), yOffsetFromCenterOfHips.getDoubleValue(), 0.0);
      
      //for debug
      //      swingTargetToPack.set(desiredSwingFootPositionFromHalfStride);
      //      swingTargetToPack.set(desiredSwingFootPositionFromOppositeSideFoot);
   }

   private void determineFootPositionFromHalfStride(QuadrupedSupportPolygon supportPolygon, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double maxStepDistance, double deltaYaw,
          FramePoint footPositionSameSideOppositeEnd, FramePoint footPositionOppositeSideSameEnd, ReferenceFrame oppositeSideZUpFrame)
   {
      RobotSide swingSide = swingLeg.getSide();
      RobotEnd robotEnd = swingLeg.getEnd();

      //check the difference in orientation between the oppositeSideZUpFrame and the bodyZUpFrame
      double orientationDeltaWithBody = calculateOppositeSideOrientationWithRespectToBody(oppositeSideZUpFrame);
      
      //handle forward backward placement
      desiredSwingFootPositionFromHalfStride.setToZero(oppositeSideZUpFrame);
      double halfStrideLength = 0.5 * strideLength.getDoubleValue();
      double clippedSkew = MathTools.clipToMinMax(maxForwardSkew.getDoubleValue(), 0.0, halfStrideLength);
      double clippedSkewScalar = MathTools.clipToMinMax(desiredBodyVelocity.getX() / minimumVelocityForFullSkew.getDoubleValue(), 1.0);
      double amountToSkew = clippedSkewScalar * clippedSkew;
      amountToSkew = MathTools.clipToMinMax(amountToSkew, maxStepDistance);
      double newY = robotEnd.negateIfFrontEnd(halfStrideLength) - amountToSkew;
      desiredSwingFootPositionFromHalfStride.setY(newY);

      //handle left right placement
      double halfStanceWidth = 0.5 * stanceWidth.getDoubleValue();
      double lateralClippedSkew = MathTools.clipToMinMax(maxLateralSkew.getDoubleValue(), 0.0, halfStanceWidth);
      double lateralClippedSkewScalar = MathTools.clipToMinMax(desiredBodyVelocity.getY() / minimumVelocityForFullSkew.getDoubleValue(), 1.0);
      double lateralAmountToSkew = lateralClippedSkewScalar * lateralClippedSkew;
      lateralAmountToSkew = MathTools.clipToMinMax(lateralAmountToSkew, maxStepDistance);
      double newX = swingSide.negateIfRightSide(stanceWidth.getDoubleValue()) + lateralAmountToSkew;
      desiredSwingFootPositionFromHalfStride.setX(newX);
      
      // maintain minimumDistanceFromSameSideFoot inline
      footPositionSameSideOppositeEnd.changeFrame(oppositeSideZUpFrame);
      double minimumRadiusFromSameSideFoot = minimumDistanceFromSameSideFoot.getDoubleValue();

      boolean footIsForwardOfOtherFoot = desiredSwingFootPositionFromHalfStride.getY() < footPositionSameSideOppositeEnd.getY();
      boolean footIsBehindOtherFoot = desiredSwingFootPositionFromHalfStride.getY() > footPositionSameSideOppositeEnd.getY();
      boolean footIsCloseToOtherFoot = desiredSwingFootPositionFromHalfStride.distance(footPositionSameSideOppositeEnd) < minimumRadiusFromSameSideFoot;

      if ((robotEnd.equals(RobotEnd.HIND) && footIsForwardOfOtherFoot) || (robotEnd.equals(RobotEnd.FRONT) && footIsBehindOtherFoot) || footIsCloseToOtherFoot)
      {
         desiredSwingFootPositionFromHalfStride.setY(footPositionSameSideOppositeEnd.getY());
         desiredSwingFootPositionFromHalfStride.add(0.0, robotEnd.negateIfFrontEnd(minimumRadiusFromSameSideFoot), 0.0);
      }

      //compensate the angular diplacement if the footsepts of the side used as reference is not aligned with the body
      desiredSwingFootPositionFromHalfStride.set(desiredSwingFootPositionFromHalfStride.yawAboutPoint(footPositionOppositeSideSameEnd, - orientationDeltaWithBody));
      
      desiredSwingFootPositionFromHalfStride.changeFrame(ReferenceFrame.getWorldFrame());

      //rotate the foot about the centroid of the predicted foot polygon
      supportPolygon.setFootstep(swingLeg, desiredSwingFootPositionFromHalfStride);
      supportPolygon.getCentroid(centroid);
      desiredSwingFootPositionFromHalfStride.set(desiredSwingFootPositionFromHalfStride.yawAboutPoint(centroid, deltaYaw));
   }

   private void determineFootPositionFromOppositeSideFoot(QuadrupedSupportPolygon supportPolygon, RobotQuadrant swingLeg, FrameVector desiredBodyVelocity, double maxStepDistance, double deltaYaw,
         FramePoint footPositionSameSideOppositeEnd, FramePoint footPositionOppositeSideSameEnd, ReferenceFrame oppositeSideZUpFrame)
   {
      RobotSide swingSide = swingLeg.getSide();
      
      //check the difference in orientation between the oppositeSideZUpFrame and the bodyZUpFrame
      double orientationDeltaWithBody = calculateOppositeSideOrientationWithRespectToBody(oppositeSideZUpFrame);
      
      desiredSwingFootPositionFromOppositeSideFoot.setToZero(oppositeSideZUpFrame);
      double halfStrideLength = 0.5 * strideLength.getDoubleValue();
      double clippedSkew = MathTools.clipToMinMax(maxForwardSkew.getDoubleValue(), 0.0, halfStrideLength);
      double clippedSkewScalar = MathTools.clipToMinMax(desiredBodyVelocity.getX() / minimumVelocityForFullSkew.getDoubleValue(), 1.0);
      double amountToSkew = clippedSkewScalar * clippedSkew;
      amountToSkew = MathTools.clipToMinMax(amountToSkew, maxStepDistance);
      
      footPositionOppositeSideSameEnd.changeFrame(oppositeSideZUpFrame);      
      
      double newY = footPositionOppositeSideSameEnd.getY() - amountToSkew;

      // maintain minimumDistanceFromSameSideFoot inline
      footPositionSameSideOppositeEnd.changeFrame(oppositeSideZUpFrame);
      if(swingLeg.isQuadrantInHind() && (footPositionSameSideOppositeEnd.getY() + minimumDistanceFromSameSideFoot.getDoubleValue() > newY ))
         newY = footPositionSameSideOppositeEnd.getY() + minimumDistanceFromSameSideFoot.getDoubleValue();
      
      desiredSwingFootPositionFromOppositeSideFoot.setY(newY);
      
      //handle left right placement
      double halfStanceWidth = 0.5 * stanceWidth.getDoubleValue();
      double lateralClippedSkew = MathTools.clipToMinMax(maxLateralSkew.getDoubleValue(), 0.0, halfStanceWidth);
      double lateralClippedSkewScalar = MathTools.clipToMinMax(desiredBodyVelocity.getY() / minimumVelocityForFullSkew.getDoubleValue(), 1.0);
      double lateralAmountToSkew = lateralClippedSkewScalar * lateralClippedSkew;
      lateralAmountToSkew = MathTools.clipToMinMax(lateralAmountToSkew, maxStepDistance);
      
      double newX = swingSide.negateIfRightSide(stanceWidth.getDoubleValue()) + lateralAmountToSkew;
      desiredSwingFootPositionFromOppositeSideFoot.setX(newX);
      
      //compensate the angular diplacement if the footsepts of the side used as reference is not aligned with the body
      desiredSwingFootPositionFromOppositeSideFoot.set(desiredSwingFootPositionFromOppositeSideFoot.yawAboutPoint(footPositionOppositeSideSameEnd, - orientationDeltaWithBody));
      
      desiredSwingFootPositionFromOppositeSideFoot.changeFrame(ReferenceFrame.getWorldFrame());
      
      //rotate the foot about the centroid of the predicted foot polygon
      supportPolygon.setFootstep(swingLeg, desiredSwingFootPositionFromOppositeSideFoot);
      supportPolygon.getCentroid(centroid);
      desiredSwingFootPositionFromOppositeSideFoot.set(desiredSwingFootPositionFromOppositeSideFoot.yawAboutPoint(centroid, deltaYaw));
   }

   private double calculateOppositeSideOrientationWithRespectToBody(ReferenceFrame oppositeSideZUpFrame)
   {
      FrameOrientation2d oppositeSideOrientation = new FrameOrientation2d(oppositeSideZUpFrame);
      oppositeSideOrientation.changeFrame(referenceFrames.getBodyZUpFrame());
      double orientationDeltaWithBody = oppositeSideOrientation.getYaw() - Math.PI / 2.0;
      return orientationDeltaWithBody;
   }

   private void updateFeetPositions()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint footPosition = supportPolygon.getFootstep(robotQuadrant);
         if(footPosition == null)
         {
            footPosition = new FramePoint(ReferenceFrame.getWorldFrame());
         }
         
         footPosition.setToZero(referenceFrames.getFootFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(robotQuadrant, footPosition);
      }
   }
}