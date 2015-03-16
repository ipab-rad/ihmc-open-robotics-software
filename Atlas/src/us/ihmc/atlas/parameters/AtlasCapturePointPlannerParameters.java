package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

public class AtlasCapturePointPlannerParameters implements CapturePointPlannerParameters
{
	private boolean runningOnRealRobot;

	public AtlasCapturePointPlannerParameters(boolean runningOnRealRobot)
	{
		this.runningOnRealRobot = runningOnRealRobot;
	}
	
	@Override
	public double getDoubleSupportInitialTransferDuration()
	{
		return runningOnRealRobot ? 2.0 : 1.0; 
	}

	@Override
	public double getDoubleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.25;
	}

   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.1;
   }

	@Override
	public double getSingleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.7;
	}

	@Override
	public int getNumberOfFootstepsToConsider()
	{
		return 3;
	}

	@Override
	public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
	{
		return 5;
	}

	@Override
	public int getNumberOfFootstepsToStop()
	{
		return 2;
	}

	@Override
	public double getIsDoneTimeThreshold()
	{
		return -1e-4;
	}

   @Override
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   @Override
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return 0.025; //0.035;
   }

   @Override
   public boolean getDoTimeFreezing()
   {
      return true;
   }

   @Override
   public boolean getDoFootSlipCompensation()
   {
      return true;
   }

   @Override
   public double getAlphaDeltaFootPositionForFootslipCompensation()
   {
      return 0.65;
   }

	@Override
	public double getCapturePointInFromFootCenterDistance()
	{
		return -0.005; //0.006;
	}

	@Override
	public double getCapturePointForwardFromFootCenterDistance()
	{
		return 0.0;
	}

	@Override
	public double getMaxAllowedErrorWithoutPartialTimeFreeze()
	{
		return 0.03;
	}
}