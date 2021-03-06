package object_manipulation_msgs;

public interface PickupResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/PickupResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\n# The overall result of the pickup attempt\nManipulationResult manipulation_result\n\n# The performed grasp, if attempt was successful\nGrasp grasp\n\n# the complete list of attempted grasp, in the order in which they have been attempted\n# the successful one should be the last one in this list\nGrasp[] attempted_grasps\n\n# the outcomes of the attempted grasps, in the same order as attempted_grasps\nGraspResult[] attempted_grasp_results\n\n";
  object_manipulation_msgs.ManipulationResult getManipulationResult();
  void setManipulationResult(object_manipulation_msgs.ManipulationResult value);
  object_manipulation_msgs.Grasp getGrasp();
  void setGrasp(object_manipulation_msgs.Grasp value);
  java.util.List<object_manipulation_msgs.Grasp> getAttemptedGrasps();
  void setAttemptedGrasps(java.util.List<object_manipulation_msgs.Grasp> value);
  java.util.List<object_manipulation_msgs.GraspResult> getAttemptedGraspResults();
  void setAttemptedGraspResults(java.util.List<object_manipulation_msgs.GraspResult> value);
}
