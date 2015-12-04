package tf2_msgs;

public interface LookupTransformAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf2_msgs/LookupTransformAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nLookupTransformActionGoal action_goal\nLookupTransformActionResult action_result\nLookupTransformActionFeedback action_feedback\n";
  tf2_msgs.LookupTransformActionGoal getActionGoal();
  void setActionGoal(tf2_msgs.LookupTransformActionGoal value);
  tf2_msgs.LookupTransformActionResult getActionResult();
  void setActionResult(tf2_msgs.LookupTransformActionResult value);
  tf2_msgs.LookupTransformActionFeedback getActionFeedback();
  void setActionFeedback(tf2_msgs.LookupTransformActionFeedback value);
}