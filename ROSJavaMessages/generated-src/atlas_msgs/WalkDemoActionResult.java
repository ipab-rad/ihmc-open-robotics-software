package atlas_msgs;

public interface WalkDemoActionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/WalkDemoActionResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nWalkDemoResult result\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  atlas_msgs.WalkDemoResult getResult();
  void setResult(atlas_msgs.WalkDemoResult value);
}
