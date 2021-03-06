package us.ihmc.simulationconstructionset.joystick;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class EnumYoVariableDependentJoystickInputManager<T>
{
   private final int pollIntervalMillis = 20;
   private final Joystick joystick;
   private final HashMap<Enum<?>, ArrayList<JoystickEventListener>> eventListeners = new HashMap<>();
   private final EnumYoVariable<?> enumYoVariable;
   private final T[] enumValues;
   
   public EnumYoVariableDependentJoystickInputManager(final EnumYoVariable<?> enumYoVariable, Class<T> enumType) throws IOException
   {
      this.enumValues = enumType.getEnumConstants();
      this.enumYoVariable = enumYoVariable;
      
      joystick = new Joystick();
      joystick.setPollInterval(pollIntervalMillis);
      
      enumYoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            updateListeners(enumYoVariable);
         }
      });
   }
   
   public Joystick getJoystick()
   {
      return joystick;
   }
   
   public void initialize()
   {
      updateListeners(enumYoVariable);
   }
   
   private void updateListeners(final EnumYoVariable<?> enumYoVariable)
   {
      joystick.clearEventListeners();
      T enumValue = enumValues[enumYoVariable.getOrdinal()];
      if(eventListeners.containsKey(enumValue))
      {
         for(JoystickEventListener eventListener : eventListeners.get(enumValue))
         {
            joystick.addJoystickEventListener(eventListener);
         }
      }
   }
   
   public void disableJoystick()
   {
      joystick.clearEventListeners();
   }
   
   public void addJoystickMapping(EnumDependentJoystickMapping joystickMap)
   {
      eventListeners.put(joystickMap.getEnum(), joystickMap.getEventListeners());
   }
}
