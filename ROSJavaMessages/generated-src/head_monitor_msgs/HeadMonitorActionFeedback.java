package head_monitor_msgs;

public interface HeadMonitorActionFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "head_monitor_msgs/HeadMonitorActionFeedback";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nHeadMonitorFeedback feedback\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  head_monitor_msgs.HeadMonitorFeedback getFeedback();
  void setFeedback(head_monitor_msgs.HeadMonitorFeedback value);
}
