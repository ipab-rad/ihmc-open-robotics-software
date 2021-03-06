package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;

import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLArray;
import com.jmatio.types.MLDouble;
import com.jmatio.types.MLInt32;
import com.jmatio.types.MLInt64;
import com.jmatio.types.MLNumericArray;

import us.ihmc.robotDataCommunication.logger.util.CustomProgressMonitor;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.dataBuffer.DataEntry;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;

public class YoVariableExporter extends YoVariableLogReader
{
   private final StandardSimulationGUI gui;
   private final List<YoVariable<?>> variables;

   public YoVariableExporter(SimulationConstructionSet scs, File logDirectory, LogProperties logProperties, List<YoVariable<?>> variables)
   {
      super(logDirectory, logProperties);
      this.gui = scs.getGUI();
      this.variables = variables;
   }

   public void exportGraphs(File file, long start, long end)
   {
      CustomProgressMonitor monitor = new CustomProgressMonitor("Export data to Matlab", "Reading variable data", 0, 100);

      
      if (!initialize())
      {
         return;
      }

      try
      {
         int startPosition = getPosition(start);
         int endPosition = getPosition(end);
         int elements = endPosition - startPosition + 1;

         // Time element
         MLInt64 timestamp = new MLInt64("timestamp", new int[] { elements, 1 });
         MLDouble robotTime = new MLDouble("robotTime", new int[] { elements, 1 });
         
         ArrayList<DataHolder<?>> dataHolders = new ArrayList<>();
         
         for (YoGraph graph : gui.getGraphArrayPanel().getGraphsOnThisPanel())
         {
            for (DataEntry entry : graph.getEntriesOnThisGraph())
            {
               YoVariable<?> variable = entry.getVariable();
               int offset = variables.indexOf(variable);
               if (offset == -1)
               {
                  System.err.println("Cannot export variable " + variable.getName() + " as it is calculated by the visualizer.");
                  continue;
               }
               int dataBufferOffset = offset + 1;
               dataHolders.add(createDataHolder(dataBufferOffset, elements, variable));
            }
         }
         
         int step = elements / 90;
         
         long firstTimestamp = -1;
         for (int i = startPosition; i <= endPosition; i++)
         {
            if((i - startPosition) % step == 0) 
            {
               monitor.setProgress((i - startPosition) / step);
            }
            
            ByteBuffer data = readData(i);
            LongBuffer dataAsLong = data.asLongBuffer();

            
            long entryTimestamp = dataAsLong.get();
            
            if(firstTimestamp == -1)
            {
               firstTimestamp = entryTimestamp;
            }
            
            timestamp.setReal(entryTimestamp, i - startPosition);
            robotTime.setReal(TimeTools.nanoSecondstoSeconds(entryTimestamp - firstTimestamp), i - startPosition);
            
            for (int dh = 0; dh < dataHolders.size(); dh++)
            {
               DataHolder<?> dataHolder = dataHolders.get(dh);
               dataHolder.addEntry(dataAsLong);
            }
         }
         
         monitor.setNote("Writing data to disk");
         ArrayList<MLArray>  matlabData = new ArrayList<>();
         
         matlabData.add(timestamp);
         matlabData.add(robotTime);
         for (int dh = 0; dh < dataHolders.size(); dh++)
         {
            matlabData.add(dataHolders.get(dh).getData());
         }
         
         new MatFileWriter(file, matlabData);
         
         monitor.close();

      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private DataHolder<?> createDataHolder(int offset, int elements, YoVariable<?> variable)
   {
      int[] dims = { elements, 1 };
      String name =  variable.getName();
      if (variable instanceof EnumYoVariable<?>)
      {
         return new DataHolder<Integer>(offset, new MLInt32(name, dims))
         {

            @Override
            public void set(long entryAsLong)
            {
               set((Integer) ((int) entryAsLong));
            }

         };
      }
      else if (variable instanceof LongYoVariable)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {

            @Override
            public void set(long entryAsLong)
            {
               set((Long) entryAsLong);
            }

         };
      }
      else if (variable instanceof IntegerYoVariable)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {

            @Override
            public void set(long entryAsLong)
            {
               set((Long) entryAsLong);
            }

         };
      }
      else if (variable instanceof DoubleYoVariable)
      {
         return new DataHolder<Double>(offset, new MLDouble(name, dims))
         {

            @Override
            public void set(long entryAsLong)
            {
               set((Double) Double.longBitsToDouble(entryAsLong));
            }

         };
      }
      else if (variable instanceof BooleanYoVariable)
      {
         return new DataHolder<Integer>(offset, new MLInt32(name, dims))
         {

            @Override
            public void set(long entryAsLong)
            {
               set((Integer) (entryAsLong == 1L ? 1 : 0));
            }

         };
      }
      else
      {
         throw new RuntimeException("Unknown YoVariable type " + variable.getClass().getSimpleName());
      }
   }

   private abstract class DataHolder<T extends Number>
   {
      private final MLNumericArray<T> data;
      private final int offset;

      private int index = 0;

      private DataHolder(int offset, MLNumericArray<T> data)
      {
         this.data = data;
         this.offset = offset;
      }

      public void addEntry(LongBuffer dataAsLong)
      {
         long entryAsLong = dataAsLong.get(offset);
         set(entryAsLong);
         index++;
      }

      public abstract void set(long entryAsLong);

      protected void set(T entryAsDataType)
      {
         data.setReal(entryAsDataType, index);
      }

      public MLNumericArray<T> getData()
      {
         return data;
      }
   }

}
