package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;

public interface SensorReader
{
   public abstract void read();
   
   public abstract SensorOutputMapReadOnly getSensorOutputMapReadOnly();
}