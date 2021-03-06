package object_manipulation_msgs;

public interface GraspHandPostureExecutionActionFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspHandPostureExecutionActionFeedback";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nGraspHandPostureExecutionFeedback feedback\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  object_manipulation_msgs.GraspHandPostureExecutionFeedback getFeedback();
  void setFeedback(object_manipulation_msgs.GraspHandPostureExecutionFeedback value);
}
