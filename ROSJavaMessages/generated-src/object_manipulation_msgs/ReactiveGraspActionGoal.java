package object_manipulation_msgs;

public interface ReactiveGraspActionGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ReactiveGraspActionGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalID goal_id\nReactiveGraspGoal goal\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalID getGoalId();
  void setGoalId(actionlib_msgs.GoalID value);
  object_manipulation_msgs.ReactiveGraspGoal getGoal();
  void setGoal(object_manipulation_msgs.ReactiveGraspGoal value);
}
