package pr2_mechanism_msgs;

public interface SwitchControllerActionFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/SwitchControllerActionFeedback";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nSwitchControllerFeedback feedback\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  pr2_mechanism_msgs.SwitchControllerFeedback getFeedback();
  void setFeedback(pr2_mechanism_msgs.SwitchControllerFeedback value);
}
