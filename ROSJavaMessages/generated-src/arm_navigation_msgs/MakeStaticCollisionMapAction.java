package arm_navigation_msgs;

public interface MakeStaticCollisionMapAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MakeStaticCollisionMapAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nMakeStaticCollisionMapActionGoal action_goal\nMakeStaticCollisionMapActionResult action_result\nMakeStaticCollisionMapActionFeedback action_feedback\n";
  arm_navigation_msgs.MakeStaticCollisionMapActionGoal getActionGoal();
  void setActionGoal(arm_navigation_msgs.MakeStaticCollisionMapActionGoal value);
  arm_navigation_msgs.MakeStaticCollisionMapActionResult getActionResult();
  void setActionResult(arm_navigation_msgs.MakeStaticCollisionMapActionResult value);
  arm_navigation_msgs.MakeStaticCollisionMapActionFeedback getActionFeedback();
  void setActionFeedback(arm_navigation_msgs.MakeStaticCollisionMapActionFeedback value);
}