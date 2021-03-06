package move_arm_head_monitor;

public interface HeadLookGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "move_arm_head_monitor/HeadLookGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n#goal definition\ntime target_time\nstring target_link\nfloat32 target_x\nfloat32 target_y\nfloat32 target_z\n";
  org.ros.message.Time getTargetTime();
  void setTargetTime(org.ros.message.Time value);
  java.lang.String getTargetLink();
  void setTargetLink(java.lang.String value);
  float getTargetX();
  void setTargetX(float value);
  float getTargetY();
  void setTargetY(float value);
  float getTargetZ();
  void setTargetZ(float value);
}
