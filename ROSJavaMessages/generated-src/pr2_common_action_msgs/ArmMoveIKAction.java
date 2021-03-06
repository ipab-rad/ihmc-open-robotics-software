package pr2_common_action_msgs;

public interface ArmMoveIKAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_common_action_msgs/ArmMoveIKAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nArmMoveIKActionGoal action_goal\nArmMoveIKActionResult action_result\nArmMoveIKActionFeedback action_feedback\n";
  pr2_common_action_msgs.ArmMoveIKActionGoal getActionGoal();
  void setActionGoal(pr2_common_action_msgs.ArmMoveIKActionGoal value);
  pr2_common_action_msgs.ArmMoveIKActionResult getActionResult();
  void setActionResult(pr2_common_action_msgs.ArmMoveIKActionResult value);
  pr2_common_action_msgs.ArmMoveIKActionFeedback getActionFeedback();
  void setActionFeedback(pr2_common_action_msgs.ArmMoveIKActionFeedback value);
}
