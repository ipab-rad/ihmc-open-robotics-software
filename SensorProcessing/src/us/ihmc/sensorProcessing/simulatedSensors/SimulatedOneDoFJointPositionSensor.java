package us.ihmc.sensorProcessing.simulatedSensors;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOneDoFJointPositionSensor extends SimulatedSensor<MutableDouble>
{
   private final ControlFlowOutputPort<Double> jointPositionOutputPort = createOutputPort();
   private final OneDegreeOfFreedomJoint joint;
   private final MutableDouble jointPosition = new MutableDouble();

   public SimulatedOneDoFJointPositionSensor(String name, OneDegreeOfFreedomJoint joint)
   {
      this.joint = joint;
   }

   public void startComputation()
   {
      jointPosition.setValue(joint.getQ().getDoubleValue());
      corrupt(jointPosition);
      jointPositionOutputPort.setData(jointPosition.doubleValue());
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Double> getJointPositionOutputPort()
   {
      return jointPositionOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}