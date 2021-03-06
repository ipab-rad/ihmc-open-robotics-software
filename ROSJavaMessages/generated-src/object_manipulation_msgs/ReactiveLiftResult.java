package object_manipulation_msgs;

public interface ReactiveLiftResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ReactiveLiftResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\n# The result of the lift attempt\nManipulationResult manipulation_result\n\n";
  object_manipulation_msgs.ManipulationResult getManipulationResult();
  void setManipulationResult(object_manipulation_msgs.ManipulationResult value);
}
