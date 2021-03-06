package us.ihmc.quadrupedRobotics.supportPolygon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.tools.testing.TestPlanTarget.Fast;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import org.junit.Test;

import com.google.caliper.Benchmark;
import com.google.caliper.api.VmOptions;
import com.google.caliper.runner.CaliperMain;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.RunnableThatThrows;

@DeployableTestClass(targets = Fast)
@VmOptions("-XX:-TieredCompilation")
public class QuadrupedSupportPolygonTest
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testConstructorsGettersAndSetters()
   {
      QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
      
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      quadrupedSupportPolygon = new QuadrupedSupportPolygon(framePoints);
      quadrupedSupportPolygon = new QuadrupedSupportPolygon(quadrupedSupportPolygon);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         quadrupedSupportPolygon.setFootstep(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      quadrupedSupportPolygon.printOutPolygon(getClass().getMethods()[0].getName());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3d tuple3dToPack = new Point3d();
         quadrupedSupportPolygon.getFootstep(robotQuadrant).get(tuple3dToPack);
         assertEquals("Point not equal", footPoints.get(robotQuadrant), tuple3dToPack);
      }
      
      assertEquals("RefFrame getter not work", ReferenceFrame.getWorldFrame(), quadrupedSupportPolygon.getReferenceFrame());
      
      System.out.println(quadrupedSupportPolygon.toString());
      
      quadrupedSupportPolygon.changeFrame(ReferenceFrame.getWorldFrame());
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetEdgeNormals()
   {
      QuadrupedSupportPolygon quadrupedSupportPolygon = createSimplePolygon();
      
      FrameVector2d[] normalsToPack = new FrameVector2d[quadrupedSupportPolygon.size()];
      for (int i = 0; i < normalsToPack.length; i++)
      {
         normalsToPack[i] = new FrameVector2d(quadrupedSupportPolygon.getReferenceFrame());
      }
      
      quadrupedSupportPolygon.getEdgeNormals2d(normalsToPack);
      
      assertEquals("Does not equal", new FrameVector2d(WORLD, 0.0, 1.0), normalsToPack[0]);
      assertEquals("Does not equal", new FrameVector2d(WORLD, 1.0, 0.0), normalsToPack[1]);
      assertEquals("Does not equal", new FrameVector2d(WORLD, 0.0, -1.0), normalsToPack[2]);
      assertEquals("Does not equal", new FrameVector2d(WORLD, -1.0, 0.0), normalsToPack[3]);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetLegPairs()
   {
      final QuadrupedSupportPolygon quadrupedSupportPolygon = createSimplePolygon();
      
      RobotQuadrant[][] legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 4", 4, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.FRONT_LEFT);
      
      legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 3", 3, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      
      legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 1", 1, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.HIND_LEFT);
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            quadrupedSupportPolygon.getLegPairs();
         }
      });
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPitch()
   {
      QuadrupedSupportPolygon pitchedUpPolygon = createPitchedUpPolygon();
      assertEquals("not 45 degrees", -Math.PI / 4, pitchedUpPolygon.getPitchInRadians(), 1e-7);
      
      QuadrupedSupportPolygon pitchedDownPolygon = createPitchedDownPolygon();
      assertEquals("not -45 degrees", Math.PI / 4, pitchedDownPolygon.getPitchInRadians(), 1e-7);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testDistanceInside()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      assertEquals("not 0.0 inside", 0.0, simplePolygon.distanceInside2d(new FramePoint(WORLD, 0.0,  0.0, 0.0)), 1e-7);
      assertEquals("not 0.25 inside", 0.25, simplePolygon.distanceInside2d(new FramePoint(WORLD, 0.25,  0.5, 0.0)), 1e-7);
      assertTrue("should be inside", simplePolygon.isInside(new FramePoint(WORLD, 0.25,  0.5, 0.0)));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSizeMethods()
   {
      QuadrupedSupportPolygon variableSizedPolygon = create3LegPolygon();
      assertEquals("not size 3", 3, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 2", 2, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 1", 1, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 0", 0, variableSizedPolygon.size());
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPolygonOrdering()
   {
      QuadrupedSupportPolygon outOfOrderPolygon = createOutOfOrderPolygon();
      
      int i = 0;
      for (RobotQuadrant robotQuadrant : outOfOrderPolygon.getSupportingQuadrantsInOrder())
      {
         if (i == 0)
            assertEquals("wrong quad", RobotQuadrant.FRONT_LEFT, robotQuadrant);
         else if (i == 1)
            assertEquals("wrong quad", RobotQuadrant.FRONT_RIGHT, robotQuadrant);
         else if (i == 2)
            assertEquals("wrong quad", RobotQuadrant.HIND_RIGHT, robotQuadrant);
         else
            assertEquals("wrong quad", RobotQuadrant.HIND_LEFT, robotQuadrant);
         i++;
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCentroid()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint centroid = new FramePoint();
      simplePolygon.getCentroid2d(centroid);
      FramePoint expected = new FramePoint(WORLD, 0.5, 0.5, 0.0);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
      simplePolygon.getCentroid2d(centroid);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testInCircle()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint inCircle = new FramePoint();
      double radius = simplePolygon.getInCircle2d(inCircle);
      FramePoint expected = new FramePoint(WORLD, 0.5, 0.5, 0.0);
      assertTrue("not equal expected " + expected + " actual " + inCircle, expected.epsilonEquals(inCircle, 1e-7));
      assertEquals("not correct radius", 0.5, radius, 1e-7);
   }
   
   @Benchmark
   public void timeComputePolygon(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      
      FrameVector2d[] normalsToPack = new FrameVector2d[createSimplePolygon.size()];
      for (int j = 0; j < normalsToPack.length; j++)
      {
         normalsToPack[j] = new FrameVector2d();
      }
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getEdgeNormals2d(normalsToPack);
      }
   }
   
   @Benchmark
   public void benchmarkCentroid(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint framePoint = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getCentroid2d(framePoint);
      }
   }
   
   @Benchmark
   public void benchmarkCentroidOpt(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint framePoint = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getCentroid2d(framePoint);
      }
   }
   
   @Benchmark
   public void benchmarkInCirclePoint(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint intersectionToPack = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getInCircle2d(intersectionToPack);
      }
   }

   private QuadrupedSupportPolygon createSimplePolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createOutOfOrderPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon create3LegPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createPitchedUpPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 1.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 1.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createPitchedDownPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 2.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(0.0, 2.0, 2.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(-2.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(-2.0, 2.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   public static void main(String[] args)
   {
      CaliperMain.main(QuadrupedSupportPolygonTest.class, args);
   }
}
