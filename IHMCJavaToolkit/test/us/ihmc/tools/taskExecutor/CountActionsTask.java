package us.ihmc.tools.taskExecutor;

import us.ihmc.tools.taskExecutor.Task;

public class CountActionsTask implements Task
{
   private int numberOfTimesTransitionIntoActionWasCalled = 0;
   private int numberOfTimesDoActionWasCalled = 0;
   private int numberOfTimesTransitionOutOfActionWasCalled = 0;

   private final int numberOfDoActionsBeforeDone;

   public CountActionsTask(int numberOfDoActionsBeforeDone)
   {
      this.numberOfDoActionsBeforeDone = numberOfDoActionsBeforeDone;
   }

   public void doTransitionIntoAction()
   {
      numberOfTimesTransitionIntoActionWasCalled++;
   }

   public void doAction()
   {
      numberOfTimesDoActionWasCalled++;
   }

   public void doTransitionOutOfAction()
   {
      numberOfTimesTransitionOutOfActionWasCalled++;
   }

   public boolean isDone()
   {
      return (numberOfTimesDoActionWasCalled >= numberOfDoActionsBeforeDone);
   }

   public boolean checkNumberOfCalls(int expectedTransitionIntoCalls, int expectedDoActionCalls, int expectedTransactionOutOfCalls)
   {
      if (this.numberOfTimesTransitionIntoActionWasCalled != expectedTransitionIntoCalls)
         return false;

      if (this.numberOfTimesDoActionWasCalled != expectedDoActionCalls)
         return false;

      if (this.numberOfTimesTransitionOutOfActionWasCalled != expectedTransactionOutOfCalls)
         return false;

      return true;

   }

   @Override
   public void pause()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void resume()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void stop()
   {
      // TODO Auto-generated method stub
      
   }

}
