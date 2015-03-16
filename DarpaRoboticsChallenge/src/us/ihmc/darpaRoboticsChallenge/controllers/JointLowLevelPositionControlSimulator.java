package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class JointLowLevelPositionControlSimulator implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final PIDController jointController;
   
   private final double controlDT;
   private final OneDegreeOfFreedomJoint simulatedJoint;
   private final OneDoFJoint highLevelControllerOutputJoint;

   public JointLowLevelPositionControlSimulator(OneDegreeOfFreedomJoint simulatedJoint, OneDoFJoint highLevelControllerOutputJoint, double controlDT)
   {
      registry = new YoVariableRegistry(simulatedJoint.getName() + name);
      this.controlDT = controlDT;
      jointController = new PIDController(simulatedJoint.getName() + "PositionControllerSimulator", registry);

      this.simulatedJoint = simulatedJoint;
      this.highLevelControllerOutputJoint = highLevelControllerOutputJoint;

      
     /* double subtreeMass = TotalMassCalculator.computeSubTreeMass(highLevelControllerOutputJoint.getSuccessor());
      jointController.setProportionalGain(50.0 * subtreeMass);
      jointController.setIntegralGain(35.0 * subtreeMass);
      jointController.setMaxIntegralError(0.3);
      jointController.setDerivativeGain(7.0 * subtreeMass);*/
      
      jointController.setProportionalGain(5000.0);
      jointController.setIntegralGain(1000.0*50.0);
      jointController.setMaxIntegralError(0.2);
      jointController.setDerivativeGain(200.0);
      jointController.setMaximumOutputLimit( 400.0 );
   }

   public void doControl()
   {
      if (highLevelControllerOutputJoint.isUnderPositionControl())
      {
         double currentPosition = simulatedJoint.getQ().getDoubleValue();
         double desiredPosition = highLevelControllerOutputJoint.getqDesired();
         double currentRate = simulatedJoint.getQD().getDoubleValue();
         double desiredRate = highLevelControllerOutputJoint.getQdDesired();
         double desiredTau = jointController.compute(currentPosition, desiredPosition, currentRate, desiredRate, controlDT);
         simulatedJoint.setTau(desiredTau);
      }
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }
}