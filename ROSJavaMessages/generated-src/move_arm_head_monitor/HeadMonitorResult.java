package move_arm_head_monitor;

public interface HeadMonitorResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "move_arm_head_monitor/HeadMonitorResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n#result definition\nactionlib_msgs/GoalStatus resultStatus\n";
  actionlib_msgs.GoalStatus getResultStatus();
  void setResultStatus(actionlib_msgs.GoalStatus value);
}
