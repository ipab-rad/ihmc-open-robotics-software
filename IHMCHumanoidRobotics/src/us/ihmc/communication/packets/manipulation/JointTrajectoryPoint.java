package us.ihmc.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.MathTools;


@ClassDocumentation("Sub-Packet for a point in an arm joint trajectory. It works similar to the\n"
                                  + "trajectory_msgs/JointTrajectoryPoint message.")
public class JointTrajectoryPoint extends IHMCRosApiPacket<JointTrajectoryPoint>
{
   @FieldDocumentation("Arm joint angles for this waypoint in order. For Atlas the controller assumes\n"
                                     + "joint angles will be given in the following order: shoulder pitch, shoulder\n"
                                     + "roll, elbow pitch, elbow roll, wrist pitch, wrist roll")
   public double[] positions;
   @FieldDocumentation("Arm joint angular velocities for this waypoint in order.")
   public double[] velocities;
   @FieldDocumentation("Time at which the waypoint is reached after the controller recieved the packet.\n"
                                     + "A value of zero is not allowed.")
   public double time;
   
   public JointTrajectoryPoint()
   {
   }
   
   public JointTrajectoryPoint(JointTrajectoryPoint jointTrajectoryPoint)
   {
      if (jointTrajectoryPoint.positions != null)
            this.positions = ArrayTools.copyArray(jointTrajectoryPoint.positions);
      
      if (jointTrajectoryPoint.velocities != null)
         this.velocities = ArrayTools.copyArray(jointTrajectoryPoint.velocities);
      
      this.time = jointTrajectoryPoint.time;
   }
   
   public JointTrajectoryPoint(double[] positions, double velocities[], double time)
   {
      this.positions = positions;
      this.velocities = velocities;
      this.time = time;
   }
   
   public JointTrajectoryPoint(int armJoints)
   {
      this.positions = new double[armJoints];
      this.velocities = new double[armJoints];
      this.time = 0.0;
   }
   
   @Override
   public boolean epsilonEquals(JointTrajectoryPoint other, double epsilon)
   {
      if (this.positions.length != other.positions.length || this.velocities.length != other.velocities.length)
      {
         return false;
      }
      
      for (int i = 0; i < this.positions.length; i++)
      {
         if (!MathTools.epsilonEquals(this.positions[i], other.positions[i], epsilon))
         {
            return false;
         }
      }
      
      for (int i = 0; i < this.velocities.length; i++)
      {
         if (!MathTools.epsilonEquals(this.velocities[i], other.velocities[i], epsilon))
         {
            return false;
         }
      }
      
      if (!MathTools.epsilonEquals(this.time, other.time, epsilon))
      {
         return false;
      }
      
      return true;
   }
   
   public JointTrajectoryPoint(Random random)
   {
      int numberOfJoints = random.nextInt(10);
      positions = new double[numberOfJoints];
      velocities = new double[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         positions[i] = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
         velocities[i] = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      }
      time = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
   }
}