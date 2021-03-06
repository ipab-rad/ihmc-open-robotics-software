package move_arm_head_monitor;

public interface HeadMonitorActionGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "move_arm_head_monitor/HeadMonitorActionGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalID goal_id\nHeadMonitorGoal goal\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalID getGoalId();
  void setGoalId(actionlib_msgs.GoalID value);
  move_arm_head_monitor.HeadMonitorGoal getGoal();
  void setGoal(move_arm_head_monitor.HeadMonitorGoal value);
}
