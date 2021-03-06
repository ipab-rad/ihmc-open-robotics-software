package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;
import java.io.Serializable;

/**
 * One of the main goals of this class is to check, at runtime, that operations on tuples occur within the same Frame.
 * This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public abstract class FrameTuple<T extends Tuple3d> extends ReferenceFrameHolder implements Serializable
{
   private static final long serialVersionUID = 3894861900288076730L;

   private static final boolean DEBUG = false;

   protected ReferenceFrame referenceFrame;
   protected final T tuple;
   protected String name;

   public FrameTuple(ReferenceFrame referenceFrame, T tuple, String name)
   {
      if (DEBUG)
      {
         if (referenceFrame == null)
         {
            String errorMsg = "FrameTuple: created a " + "className" + " with a null reference frame.";
            System.err.println(errorMsg);
         }
      }

      this.referenceFrame = referenceFrame;
      this.tuple = tuple;
      this.name = name;
   }

   public final void setName(String name)
   {
      this.name = name;
   }

   public final String getName()
   {
      return name;
   }

   public final void set(double x, double y, double z)
   {
      tuple.x = x;
      tuple.y = y;
      tuple.z = z;
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this.referenceFrame = referenceFrame;
      set(x, y, z);
   }

   public final void set(Tuple3d tuple)
   {
      this.tuple.set(tuple);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3d tuple)
   {
      this.referenceFrame = referenceFrame;
      set(tuple);
   }

   /**
    * Set the x and y components of this frameTuple to frameTuple2d.x and frameTuple2d.y respectively, and sets the z component to zero.
    * Changes the referenceFrame of this frameTuple to frameTuple2d.getReferenceFrame().
    * @param frameTuple2d
    * @throws ReferenceFrameMismatchException
    */
   public void setXYIncludingFrame(FrameTuple2d<?> frameTuple2d)
   {
      this.referenceFrame = frameTuple2d.getReferenceFrame();
      setXY(frameTuple2d);
   }

   /**
    * 
    * @throws ReferenceFrameMismatchException
    */
   public final void set(FrameTuple<?> frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      set(frameTuple.tuple);
   }

   public final void setIncludingFrame(FrameTuple<?> frameTuple)
   {
      setIncludingFrame(frameTuple.referenceFrame, frameTuple.tuple);
   }

   public final void setX(double x)
   {
      tuple.x = x;
   }

   public final void setY(double y)
   {
      tuple.y = y;
   }

   public final void setZ(double z)
   {
      tuple.z = z;
   }

   public final void set(Direction direction, double value)
   {
      MathTools.set(tuple, direction, value);
   }

   /**
    * Set the x and y components of this frameTuple to tuple2d.x and tuple2d.y respectively, and sets the z component to zero.
    * @param tuple2d
    */
   public void setXY(Tuple2d tuple2d)
   {
      this.tuple.setX(tuple2d.getX());
      this.tuple.setY(tuple2d.getY());
      this.tuple.setZ(0.0);
   }

   /**
    * Set the x and y components of this frameTuple to frameTuple2d.x and frameTuple2d.y respectively, and sets the z component to zero.
    * @param frameTuple2d
    * @throws ReferenceFrameMismatchException
    */
   public void setXY(FrameTuple2d<?> frameTuple2d)
   {
      checkReferenceFrameMatch(frameTuple2d);
      setXY(frameTuple2d.tuple);
   }

   public final double get(Direction direction)
   {
      return MathTools.get(tuple, direction);
   }

   public final void scale(double scaleFactor)
   {
      tuple.scale(scaleFactor);
   }

   public final void scale(double scaleXFactor, double scaleYFactor, double scaleZFactor)
   {
      tuple.x *= scaleXFactor;
      tuple.y *= scaleYFactor;
      tuple.z *= scaleZFactor;
   }

   public final double getX()
   {
      return tuple.x;
   }

   public final double getY()
   {
      return tuple.y;
   }

   public final double getZ()
   {
      return tuple.z;
   }

   /**
    * Returns a Point3d copy of the tuple in this FrameTuple.
    *
    * @return Point3d
    */
   public final Point3d getPointCopy()
   {
      return new Point3d(tuple);
   }

   /**
    * Returns a Vector3d copy of the tuple in this FrameTuple.
    *
    * @return Vector3d
    */
   public final Vector3d getVectorCopy()
   {
      return new Vector3d(this.tuple);
   }

   public final void get(Tuple3d tuple3dToPack)
   {
      tuple3dToPack.set(tuple);
   }

   public final void get(Tuple3f tuple3fToPack)
   {
      tuple3fToPack.set(tuple);
   }

   public final void setToZero()
   {
      tuple.set(0.0, 0.0, 0.0);
   }

   public final void setToZero(ReferenceFrame referenceFrame)
   {
      setToZero();
      this.referenceFrame = referenceFrame;
   }

   public final void setToNaN()
   {
      this.tuple.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public final void setToNaN(ReferenceFrame referenceFrame)
   {
      this.tuple.set(Double.NaN, Double.NaN, Double.NaN);
      this.referenceFrame = referenceFrame;
   }

   public final void checkForNaN()
   {
      if (containsNaN())
         throw new RuntimeException(getClass().getSimpleName() + " " + this + " has a NaN!");
   }

   public final boolean containsNaN()
   {
      return Double.isNaN(tuple.x) || Double.isNaN(tuple.y) || Double.isNaN(tuple.z);
   }
   
   public final boolean containsInfinity()
   {
      return Double.isInfinite(tuple.x) || Double.isInfinite(tuple.y) || Double.isInfinite(tuple.z);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 (this = scaleFactor * tuple1).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    */
   public final void scale(double scaleFactor, Tuple3d tuple1)
   {
      tuple.scale(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds tuple2 (this = scaleFactor * tuple1 + tuple2).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    * @param tuple2 Tuple3d
    */
   public final void scaleAdd(double scaleFactor, Tuple3d tuple1, Tuple3d tuple2)
   {
      tuple.scaleAdd(scaleFactor, tuple1, tuple2);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 and then adds the scalar multiplication of tuple2 (this = scaleFactor1 * tuple1 + scaleFactor2 * tuple2).
    *
    * @param scaleFactor1 double
    * @param frameTuple1 Tuple3d
    * @param scaleFactor2 double
    * @param frameTuple2 Tuple3d
    */
   public final void scaleAdd(double scaleFactor1, Tuple3d tuple1, double scaleFactor2, Tuple3d tuple2)
   {
      tuple.x = scaleFactor1 * tuple1.x + scaleFactor2 * tuple2.x;
      tuple.y = scaleFactor1 * tuple1.y + scaleFactor2 * tuple2.y;
      tuple.z = scaleFactor1 * tuple1.z + scaleFactor2 * tuple2.z;
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds tuple1 (this = scaleFactor * this + tuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    */
   public final void scaleAdd(double scaleFactor, Tuple3d tuple1)
   {
      tuple.scaleAdd(scaleFactor, tuple1);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 (this = scaleFactor * frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scale(double scaleFactor, FrameTuple<?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scale(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1 (this = scaleFactor * this + frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple<?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      scaleAdd(scaleFactor, frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then adds frameTuple2 (this = scaleFactor * frameTuple1 + frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?>
    * @param frameTuple2 FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      scaleAdd(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this frameTuple to the scalar multiplication of frameTuple1 and then subs frameTuple2 (this = scaleFactor * frameTuple1 - frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?>
    * @param frameTuple2 FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleSub(double scaleFactor, FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);

      set(frameTuple1);
      scale(scaleFactor);
      sub(frameTuple2);
   }

   /**  
    * Sets the value of this tuple to the sum of itself and tuple1.
    * @param tuple1 the other Tuple3d
    */
   public final void add(Tuple3d tuple1)
   {
      tuple.add(tuple1);
   }

   /**  
    * Sets the value of this tuple to the sum of itself and tuple1.
    * @param tuple1 the other Tuple3d
    */
   public final void add(double x, double y, double z)
   {
      tuple.x += x;
      tuple.y += y;
      tuple.z += z;
   }

   /**
    * Sets the value of this tuple to the sum of tuple1 and tuple2 (this = tuple1 + tuple2).
    * @param tuple1 the first Tuple3d
    * @param tuple2 the second Tuple3d
    */
   public final void add(Tuple3d tuple1, Tuple3d tuple2)
   {
      tuple.add(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the sum of itself and frameTuple1 (this += frameTuple1).
    * Checks if reference frames match.
    * @param frameTuple1 the other Tuple3d
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple<?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      add(frameTuple1.tuple);
   }

   /**
    * Sets the value of this frameTuple to the sum of frameTuple1 and frameTuple2 (this = frameTuple1 + frameTuple2).
    * @param frameTuple1 the first FrameTuple<?>
    * @param frameTuple2 the second FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void add(FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      add(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**  
    * Sets the value of this tuple to the difference of itself and tuple1 (this -= tuple1).
    * @param tuple1 the other Tuple3d
    */
   public final void sub(double x, double y, double z)
   {
      tuple.x -= x;
      tuple.y -= y;
      tuple.z -= z;
   }

   /**  
    * Sets the value of this tuple to the difference of itself and tuple1 (this -= tuple1).
    * @param tuple1 the other Tuple3d
    */
   public final void sub(Tuple3d tuple1)
   {
      tuple.sub(tuple1);
   }

   /**  
    * Sets the value of this tuple to the difference of tuple1 and tuple2 (this = tuple1 - tuple2).
    * @param tuple1 the first Tuple3d
    * @param tuple2 the second Tuple3d
    */
   public final void sub(Tuple3d tuple1, Tuple3d tuple2)
   {
      tuple.sub(tuple1, tuple2);
   }

   /**  
    * Sets the value of this frameTuple to the difference of itself and frameTuple1 (this -= frameTuple1).
    * @param frameTuple1 the first FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple<?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      sub(frameTuple1.tuple);
   }

   /**  
    * Sets the value of this frameTuple to the difference of frameTuple1 and frameTuple2 (this = frameTuple1 - frameTuple2).
    * @param frameTuple1 the first FrameTuple<?>
    * @param frameTuple2 the second FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void sub(FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      sub(frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of tuple1 minus tuple2 (this = scaleFactor * ( tuple1 - tuple2 ) ).
    *
    * @param scaleFactor double
    * @param tuple1 Tuple3d
    * @param tuple2 Tuple3d
    */
   public final void subAndScale(double scaleFactor, Tuple3d tuple1, Tuple3d tuple2)
   {
      sub(tuple1, tuple2);
      scale(scaleFactor);
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of frameTuple1 minus frameTuple2 (this = scaleFactor * ( frameTuple1 - frameTuple2 ) ).
    *
    * @param scaleFactor double
    * @param frameTuple1 the first FrameTuple<?>
    * @param frameTuple2 the second FrameTuple<?>
    * @throws ReferenceFrameMismatchException
    */
   public final void subAndScale(double scaleFactor, FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      subAndScale(scaleFactor, frameTuple1.tuple, frameTuple2.tuple);
   }

   /**
     *  Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:  this = (1-alpha) * tuple1 + alpha * tuple2.
     *  @param t1  the first tuple
     *  @param t2  the second tuple  
     *  @param alpha  the alpha interpolation parameter  
    */
   public final void interpolate(Tuple3d tuple1, Tuple3d tuple2, double alpha)
   {
      tuple.interpolate(tuple1, tuple2, alpha);
   }

   /**
     *  Linearly interpolates between tuples tuple1 and tuple2 and places the result into this tuple:  this = (1-alpha) * tuple1 + alpha * tuple2.
     *  @param t1  the first tuple
     *  @param t2  the second tuple  
     *  @param alpha  the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(FrameTuple<?> frameTuple1, FrameTuple<?> frameTuple2, double alpha)
   {
      frameTuple1.checkReferenceFrameMatch(frameTuple2);

      interpolate(frameTuple1.tuple, frameTuple2.tuple, alpha);
      referenceFrame = frameTuple1.getReferenceFrame();
   }

   public final void packMatrix(DenseMatrix64F matrix, int startRow)
   {
      MatrixTools.setDenseMatrixFromTuple3d(matrix, tuple, startRow, 0);
   }

   public final void clipToMinMax(double minValue, double maxValue)
   {
      this.tuple.clamp(minValue, maxValue);
   }

   public final void negate()
   {
      tuple.negate();
   }

   public final void absolute()
   {
      tuple.absolute();
   }

   /**
     * Returns true if the L-infinite distance between this tuple and tuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-z2)].
    * @param tuple1 Tuple3d
    * @param threshold double
    */
   public final boolean epsilonEquals(Tuple3d tuple1, double threshold)
   {
      if(tuple1 == null)
      {
         return false;
      }
      
      return tuple.epsilonEquals(tuple1, threshold);
   }

   /**
     * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-z2)].
    * @param frameTuple1 FrameTuple<?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple<?> frameTuple1, double threshold)
   {
      if(frameTuple1 == null)
      {
         return false;
      }
      
      checkReferenceFrameMatch(frameTuple1);

      return epsilonEquals(frameTuple1.tuple, threshold);
   }

   /**
     * Returns true if the L-infinite distance between this tuple and tuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-0)].
    * @param tuple1 Tuple3d
    * @param threshold double
    */
   public final boolean epsilonEquals(Tuple2d tuple1, double threshold)
   {
      if(tuple1 == null)
      {
         return false;
      }
      
      double diff;

      diff = tuple.x - tuple1.x;
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      diff = tuple.y - tuple1.y;
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      diff = tuple.z;
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > threshold)
         return false;

      return true;
   }

   /**
     * Returns true if the L-infinite distance between this frameTuple and frameTuple1 is less than or equal to the epsilon parameter, otherwise returns false.
     * The L-infinite distance is equal to MAX[abs(x1-x2), abs(y1-y2), abs(z1-0)].
    * @param frameTuple1 FrameTuple<?>
    * @param threshold double
    * @throws ReferenceFrameMismatchException
    */
   public final boolean epsilonEquals(FrameTuple2d<?> frameTuple2d, double threshold)
   {
      if(frameTuple2d == null)
      {
         return false;
      }
      
      checkReferenceFrameMatch(frameTuple2d);

      return epsilonEquals(frameTuple2d.tuple, threshold);
   }

   /**
    * Returns this FrameTuple's ReferenceFrame.
    * @return ReferenceFrame
    */
   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);

   public abstract FrameTuple<T> changeFrameUsingTransformCopy(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame);

   public abstract void applyTransform(RigidBodyTransform transform);

   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public final double[] toArray()
   {
      return new double[] { tuple.x, tuple.y, tuple.z };
   }

   /**
    * toString
    * <p/>
    * String representation of a FrameVector (x,y,z):reference frame name
    *
    * @return String
    */
   @Override
   public final String toString()
   {
      return "" + tuple + "-" + referenceFrame.getName();
   }
}
