package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

/** {@inheritDoc} */
public class AtlasCapturePointPlannerParameters implements CapturePointPlannerParameters
{
   private boolean runningOnRealRobot;
   private final boolean useTwoCMPsPerSupport;
   private final boolean useNewICPPlanner;

   public AtlasCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      useTwoCMPsPerSupport = true;
      useNewICPPlanner = true;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return runningOnRealRobot ? 2.0 : 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getSingleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.7;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
   {
      // Using higher order for the trajectory degrade the pseudo CMP trajectory (going a lot outside the support polygon) when using two CMPs per support.
      return useTwoCMPsPerSupport ? 4 : 5;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToStop()
   {
      return 2;
   }

   /** {@inheritDoc} */
   @Override
   public double getIsDoneTimeThreshold()
   {
      return -1e-4;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getFreezeTimeFactor()
   {
      return 0.0; //0.9;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return 0.0; // 0.025; // 0.035;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getDoTimeFreezing()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getDoFootSlipCompensation()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaDeltaFootPositionForFootslipCompensation()
   {
      return 0.65;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPInsideOffset()
   {
      return -0.005; //0.006;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPInsideOffset()
   {
      return 0.025;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTerribleHackToReduceICPVelocityAtTheEndOfTransfer()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useNewICPPlanner()
   {
      return useNewICPPlanner;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return useTwoCMPsPerSupport;
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeSpentOnExitCMPInPercentOfStepTime()
   {
      return 0.50;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxEntryCMPForwardOffset()
   {
      return 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinEntryCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxExitCMPForwardOffset()
   {
      return 0.08;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinExitCMPForwardOffset()
   {
      return -0.04;
   }

   /** {@inheritDoc} */
   @Override
   public double getCMPSafeDistanceAwayFromSupportEdges()
   {
      return 0.01;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthToCMPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useExitCMPOnToesForSteppingDown()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return AtlasPhysicalProperties.footLengthForControl;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepHeightThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return 0.10;
   }

   /** {@inheritDoc} */
   @Override
   public double getCMPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinTimeToSpendOnExitCMPInSingleSupport()
   {
      return 0.0;
   }
}
