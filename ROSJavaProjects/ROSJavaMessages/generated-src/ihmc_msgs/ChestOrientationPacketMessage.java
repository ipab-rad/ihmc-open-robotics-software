package ihmc_msgs;

public interface ChestOrientationPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ChestOrientationPacketMessage";
  static final java.lang.String _DEFINITION = "## ChestOrientationPacketMessage\n# This message sets the orientation of the robot\'s chest in world coordinates.\n\ngeometry_msgs/Quaternion orientation\n\n# trajectoryTime specifies how fast or how slow to move to the desired pose\nfloat64 trajectoryTime\n\n# toHomePosition can be used to move the chest back to its default starting position\nbool toHomeOrientation\n\nint8 destination\n\n\n";
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getTrajectoryTime();
  void setTrajectoryTime(double value);
  boolean getToHomeOrientation();
  void setToHomeOrientation(boolean value);
  byte getDestination();
  void setDestination(byte value);
}