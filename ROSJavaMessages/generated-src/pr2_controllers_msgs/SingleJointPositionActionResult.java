package pr2_controllers_msgs;

public interface SingleJointPositionActionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_controllers_msgs/SingleJointPositionActionResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nSingleJointPositionResult result\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  pr2_controllers_msgs.SingleJointPositionResult getResult();
  void setResult(pr2_controllers_msgs.SingleJointPositionResult value);
}
