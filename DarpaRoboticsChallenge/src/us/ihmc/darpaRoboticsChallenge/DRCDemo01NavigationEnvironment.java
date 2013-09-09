package us.ihmc.darpaRoboticsChallenge;

import java.net.URL;
import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceTexture;
import us.ihmc.utilities.math.geometry.Box3d;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;
import com.yobotics.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;

public class DRCDemo01NavigationEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject combinedTerrainObject;

   private final Random random = new Random(1989L);

   private static final double ROCKS_START_Y = 3.5;
   private static final double ROCK_PATH_LENGTH = 8.0;

   private static final int NUM_ROCKS = 80;
   private static final double MAX_ROCK_CENTROID_HEIGHT = 0.2;
   private static final int POINTS_PER_ROCK = 21;

   // chance unevenness of rocks
   private static final double MAX_ABS_XY_NORMAL_VALUE = 0.0;
   private static final double ROCK_FIELD_WIDTH = 2.0;
   private static final double ROCK_BOUNDING_BOX_WIDTH = 0.5;

   private static final boolean FULLY_RANDOM = true;    // Will do a neat grid if set to false;
   private static final int ROCKS_PER_ROW = 4;
   private static final boolean DIFFICULT_STEPPING_STONES = false;    // for path 8, if true creates an extension to the path with harder steps

   private static final AppearanceDefinition cinderBlockAppearance = YoAppearance.DarkGray();
   private static final double cinderBlockLength = 0.40;    // 40 cm (approx 16 in, just less than 16in)
   private static final double cinderBlockWidth = cinderBlockLength / 2;
   private static final double cinderBlockHeight = 0.15;    // 15 cm (approx 6 in, less than 6 in, but consistent with other cm measurements)
   private static final double cinderBlockTiltDegrees = 15;
   private static final double cinderBlockTiltRadians = Math.toRadians(cinderBlockTiltDegrees);

   enum BLOCKTYPE {FLAT, FLATSKEW, UPRIGHTSKEW, ANGLED};

   private boolean addLimboBar = false;

   // private static final double FLOOR_THICKNESS = 0.001;

   public DRCDemo01NavigationEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject("Rocks with a wall");

      setUpPath1Rocks();
      setUpPath2SmallCones();
      setUpPath3RampsWithLargeBlocks();
      setUpPath4DRCTrialsTrainingWalkingCourse();

      setUpPath5NarrowDoor();
      setUpPath6Barriers();
      setUpPath7Stairs();
      setUpPath8RampsWithSteppingStones();

      setUpGround();

      conditionallyAddLimboBar();
   }

   private void setUpPath1Rocks()
   {
      addRocks();
   }

   private void setUpPath2SmallCones()
   {
      int numCones = 4;
      float initialOffset = 2.952f;
      float coneSeparation = 1.5f;
      float coneColorSeparateion = 0.1f;

      for (int i = 0; i < numCones; i++)
      {
         AppearanceDefinition cone1;
         AppearanceDefinition cone2;
         if (i % 2 == 0)
         {
            cone1 = YoAppearance.Green();
            cone2 = YoAppearance.Red();
         }
         else
         {
            cone1 = YoAppearance.Red();
            cone2 = YoAppearance.Green();
         }

         setUpCone(initialOffset + (i * coneSeparation) + coneColorSeparateion, -(initialOffset + (i * coneSeparation)), .25, .25, 0.5, cone1);
         setUpCone(initialOffset + (i * coneSeparation), -(initialOffset + (i * coneSeparation) + coneColorSeparateion), .25, .25, 0.45, cone2);
      }

   }

   private void setUpPath3RampsWithLargeBlocks()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      // float rampHeight = 0.3f;

      float rampHeight = 0.625f;

      setUpRamp(5.0f, 0.0f, 2.0f, 3.0f, rampHeight, color);
      setUpWall(new double[] {7.0f, 0.0f}, .5f, 1.0f, rampHeight, 0, color);

      setUpWall(new double[] {7.75f, 0.0f}, 2f, .5f, rampHeight, 0, color);
      setUpWall(new double[] {8.5f, 0f}, .5f, .75f, rampHeight - 0.1, 0, color);

      setUpWall(new double[] {8.5f, .75f}, .5f, .75f, rampHeight, 0, color);

      setUpWall(new double[] {8.5f, -0.66f}, .25f, 1f, rampHeight, 0, color);

      setUpWall(new double[] {8.5f, -1.045f}, .25f, 1f, rampHeight, 0, color);



      setUpWall(new double[] {9.25f, 0f}, 2.0f, 0.5f, rampHeight, 0, color);
      setUpRamp(11f, 0f, 2.0f, -3.0f, rampHeight, color);

      // Do this for a long ramp for testing:
      // rampHeight = 1.0f;
      // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);
   }

   private void setUpPath4DRCTrialsTrainingWalkingCourse()
   {
      double courseAngle = 45.0;
      double startDistance = 4.0;
      AppearanceDefinition color = YoAppearance.Gray();

      final double sectionLength = 2.4384;    // 8 ft

      // need basics:
      // basic ramp
      // basic block (height parameter: # block layers, 1-2 typ)
      // square block (two basic blocks side by side, height parameter: # block layers, 0-4 typ)
      // diagonal block-flat (# of square block base supports, 1 typ, e.g. 1 square block under)
      // diagonal block-upright (# of square block base supports, 0 and 1 typ)
      // slanted block (square block on ramp, with # of square block support layers: 0-3 typ)


      // 1. Flat terrain: Pavers and Astroturf. Do nothing, but space out others farther.

      // 2. Ramps (Pitch Ramps 15degrees)
      int numberOfRamps=2;
      setUpMultipleUpDownRamps(courseAngle, startDistance, numberOfRamps, sectionLength, color);
      
      // 3. Tripping Hazards
      // Diagonal 2x4s and 4x4s
      // From the picture layout:
      // first half has 2x4s(1.5x3.5) flat at 45deg angles spaced about 2ft apart (horiz and vert) (5 total)
      // second half has 4x4s(3.5x3.5) at 45 deg angles spaced 4ft apart (3 total)
      // I chose to do a worse case scenario where 2x4 and 4x4 are actually 2x4 and 4x4 (not standard sizes)
      int[] numberOfStepOvers = {5, 3};
      startDistance = setUpTripHazards(courseAngle, startDistance, numberOfStepOvers, sectionLength, color);

      // 4. Hurdles
      // 15cm (6 in) and 30 cm (12 in)
      // From the picture layout:
      // 1st section: midway (centered at 2ft) blocks layed straight across, 6 on bottom layer, 3 on top directly aligned on others. In other layout, 5 on top not directly over bottom.
      // 2nd section: midway (centered at 2ft from start of second section, 45deg zig zag pattern, two high. 8 on bottom, 4 directly on top or 7 overlapped.
      startDistance += sectionLength + sectionLength / 4;
      setUpStraightHurdles(courseAngle, startDistance, new int[] {6, 5});

      startDistance += sectionLength / 2;
      setUpZigZagHurdles(courseAngle, startDistance, new int[] {8, 7});

      startDistance += sectionLength / 4;

      // 5. Footfalls and Holes
      //    80 cm (32 in) and 40 cm (16 in) squares
      // 6. Ascend Flat Top Steps
      // 7. Descend Flat Top Steps
      // 8. Ascend Pitch/Roll 15 deg Top Steps
      // 9. Descend Pitch/Roll 15 deg Top Steps
      // TODO: Finish course
      setUpCinderBlockField(courseAngle, startDistance);

      // 10. Step-Over Obstacles

   }

   private void setUpCinderBlockField(double courseAngle, double startDistance)
   {
      int nBlocksWide = 6;
      int nBlocksLong = 31;
    
      double[][] blockAngle = new double[nBlocksLong][nBlocksWide];
      int[][] blockHeight = new int[nBlocksLong][nBlocksWide];
      BLOCKTYPE[][] blockType = new BLOCKTYPE[nBlocksLong][nBlocksWide];
      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            blockHeight[i][j] = -1;    // (int) Math.round(Math.random()*4-1);
            blockAngle[i][j] = 0;    // (int) Math.round(Math.random()*3)*45;
            blockType[i][j] = BLOCKTYPE.FLAT;
         }
      }

      blockHeight = new int[][]
      {
         {
            0, 0, -1, -1, 0, 0
         },    // 5. Footfalls and Holes
         {
            0, 0, -1, -1, 0, 0
         },
         {
            -1, -1, 0, 0, -1, -1
         },
         {
            -1, 0, 0, 0, 0, -1
         },
         {
            0, -1, 0, -1, 0, 0
         },
         {
            1, 0, -1, 0, -1, 0
         },
         {
            0, 1, 0, -1, 0, -1
         },    // 6.7. Ascend/Descend Flat Top Steps
         {
            2, 0, 1, 0, 1, -1
         },
         {
            3, 2, 0, 1, 0, 1
         },
         {
            2, 3, 2, 0, 1, -1
         },
         {
            1, 2, 3, 2, 0, 1
         },
         {
            1, 1, 2, 3, 2, 0
         },
         {
            1, 1, 1, 2, 3, 2
         },
         {
            0, 0, 1, 1, 2, 3
         },
         {
            0, 0, 0, 1, 1, 2
         },
         {
            0, 0, 0, 0, 1, 1
         },
         {
            0, 0, 0, 0, 0, 1
         },
         {
            0, 0, 0, 0, 0, 0
         },
         {
            0, 0, 0, 0, 0, 0
         },    // 8.9. Ascend/descend Pitch/Roll 15 deg Top Steps
         {
            1, 0, 1, 0, 1, 0
         },    // 1 angled...
         {
            0, 1, 0, 1, 0, 1
         },
         {
            0, 0, 0, 0, 1, 2
         },
         {
            0, 0, 0, 1, 2, 3
         },
         {
            0, 0, 1, 2, 3, 2
         },
         {
            0, 1, 2, 3, 2, 1
         },
         {
            1, 2, 3, 2, 1, 0
         },
         {
            2, 3, 2, 1, 0, 0
         },
         {
            3, 2, 1, 0, 0, 0
         },
         {
            2, 1, 0, 0, 0, 0
         },
         {
            1, 0, 0, 0, 0, 0
         },
         {
            0, 0, 0, 0, 0, 0
         }
      };

      int[] full90Diags =
      {
         -1, -3, -5, 1, 7, 8, 9, 13, 15, 17, 19, 21, 23
      };
      int[] alternating0_90DiagsAndUprightSkewed =
      {
         10, 12, 14, 16, 18, 20, 22
      };

      
      for (int i = 0; i < full90Diags.length; i++)
      {
         for (int j = Math.max(0, -full90Diags[i]); j < nBlocksWide; j++)
         {
            int col = j;
            int row = full90Diags[i] + col;
            if (row < nBlocksLong)
               blockAngle[row][col] = 90;
         }
      }

      for (int i = 0; i < alternating0_90DiagsAndUprightSkewed.length; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            int col = j;
            int row = alternating0_90DiagsAndUprightSkewed[i] + col;
            blockType[row][col] = BLOCKTYPE.UPRIGHTSKEW;
            if (j % 2 == 1)
               blockAngle[row][col] = 90;
         }
      }

      final int flatSkewedRow=19;
      for(int col=0;col<nBlocksWide-1;col++)
      {
         boolean evenCol = col % 2 == 0;
         int row=flatSkewedRow + (evenCol?0:1);
         blockType[row][col] = BLOCKTYPE.FLATSKEW;
         if(evenCol)
            blockAngle[row][col] = 90;
         else
            blockAngle[row][col] = 0;
      }

      final int NORTH = -90;
      final int SOUTH = 90;
      final int WEST = 0;
      final int EAST = 180;
      final int startAngled = 19;
      for (int i = startAngled; i < nBlocksLong; i++)
      {
         for (int j = Math.max(0, startAngled + (nBlocksWide - 1) - i); j < nBlocksWide; j++)
         {
            boolean evenRow = (i - startAngled) % 2 == 0;
            boolean evenCol = j % 2 == 0;
            blockType[i][j] = BLOCKTYPE.ANGLED;
            if (evenRow)
            {
               if (evenCol)
                  blockAngle[i][j] = WEST;
               else
                  blockAngle[i][j] = NORTH;
            }
            else
            {
               if (evenCol)
                  blockAngle[i][j] = SOUTH;
               else
                  blockAngle[i][j] = EAST;
            }
         }
      }


      startDistance += cinderBlockLength / 2;

      for (int i = 0; i < nBlocksLong; i++)
      {
         for (int j = 0; j < nBlocksWide; j++)
         {
            double xCenter = startDistance + i * cinderBlockLength;
            double yCenter = (nBlocksWide * cinderBlockLength) / 2 - j * cinderBlockLength;
            double[] point = {xCenter, yCenter};
            double[] rotatedPoint = rotateAroundOrigin(point, courseAngle);
            int h = blockHeight[i][j];
            double deg = blockAngle[i][j] + courseAngle;
            switch (blockType[i][j])
            {
               case FLAT :
                  setUpCinderBlockSquare(rotatedPoint, h, deg);

                  break;

               case FLATSKEW :
                  setUpFlatSkewedBlockSquare(rotatedPoint, h, deg);

                  break;

               case UPRIGHTSKEW :
                  setUpSkewedUprightBlockSquare(rotatedPoint, h, deg);

                  break;

               case ANGLED :
                  setUpRampBlock(rotatedPoint, h, deg);

                  break;
            }
         }
      }
   }

   private void setUpMultipleUpDownRamps(double courseAngle, double startDistance, int numberOfRamps, final double sectionLength, AppearanceDefinition color)
   {
      for (int i = 1; i <= 2*numberOfRamps; i = i + 2)
      {
         double rampLength = sectionLength / (numberOfRamps*2);
         double rampAngle = Math.toRadians(15);
         double rampHeight = rampLength * Math.tan(rampAngle);
         double rampCenter = startDistance + rampLength * (i - 1) + rampLength / 2;
         double[] newPoint = rotateAroundOrigin(new double[] {rampCenter, 0}, courseAngle);
         setUpRotatedRamp(newPoint[0], newPoint[1], sectionLength, rampLength, rampHeight, courseAngle, color);

         double rampDownCenter = startDistance + rampLength * (i) + rampLength / 2;
         newPoint = rotateAroundOrigin(new double[] {rampDownCenter, 0}, courseAngle);
         setUpRotatedRamp(newPoint[0], newPoint[1], sectionLength, -rampLength, rampHeight, courseAngle, color);
      }
   }

   private double setUpTripHazards(double courseAngle, double startDistance, int[] numberOfStepOvers, final double sectionLength, AppearanceDefinition color)
   {
      double[] stepHeight = {0.0508, 0.1016};
      double[] stepWidth = {0.1016, 0.1016};
      double[] degreesOffset = {45, -45};

      startDistance += sectionLength;

      for (int i = 0; i < numberOfStepOvers.length; i++)
      {
         for (int j = 0; j < numberOfStepOvers[i]; j++)
         {
            double stepLength;
            if (Math.abs(degreesOffset[i]) < Math.toDegrees(Math.atan(sectionLength / (sectionLength / 2))))
               stepLength = (sectionLength / 2) / Math.cos(Math.toRadians(degreesOffset[i]));
            else
               stepLength = (sectionLength) / Math.sin(Math.toRadians(degreesOffset[i]));
            double[] point = {startDistance + sectionLength / 4 + sectionLength / 2 * i, -sectionLength / 2 + j * sectionLength / (numberOfStepOvers[i] - 1)};
            double[] newPoint = rotateAroundOrigin(point, courseAngle);
            setUpWall(newPoint, stepWidth[i], stepLength, stepHeight[i], courseAngle + degreesOffset[i], color);
         }
      }
      return startDistance;
   }

   private void setUpStraightHurdles(double courseAngle, double startDistance, int[] numberStraightHurdles)
   {
      for (int i = 0; i < numberStraightHurdles.length; i++)
      {
         for (int j = 0; j < numberStraightHurdles[i]; j++)
         {
            double[] point = {startDistance, -(numberStraightHurdles[i] * cinderBlockLength) / 2 + j * cinderBlockLength};
            double[] newPoint = rotateAroundOrigin(point, courseAngle);
            setUpCinderBlock(newPoint, i, courseAngle + 90);
         }
      }
   }

   private void setUpZigZagHurdles(double courseAngle, double startDistance, int[] numberZigZagHurdles)
   {
      double xOffset = cinderBlockLength / 4 * Math.cos(Math.toRadians(45));
      double yOffset = cinderBlockLength * Math.cos(Math.toRadians(45));

      for (int i = 0; i < numberZigZagHurdles.length; i++)
      {
         int start45sign=Math.round((float) numberZigZagHurdles[i]/2.0+.25)%2==0?1:-1;//start45 when n=3,4,7,8,11,12,...
         int startXsign=Math.round(((float) numberZigZagHurdles[i]+1.)/2.0+.25)%2==0?-1:1;//start x+ when n=1, 4,5, 8,9, ...
         for (int j = 0; j < numberZigZagHurdles[i]; j++)
         {
            int evenBlockSign = (j % 2 == 0) ? 1 : -1;
            double signedXOffset = xOffset * evenBlockSign * startXsign;
            double signedAngleOffset = 45 * evenBlockSign * start45sign;
            double[] point = {startDistance + signedXOffset, ((numberZigZagHurdles[i] - 1) * yOffset) / 2 - j * yOffset};
            double[] newPoint = rotateAroundOrigin(point, courseAngle);
            setUpCinderBlock(newPoint, i, courseAngle + signedAngleOffset);
         }
      }
   }

   private void setUpRampBlock(double[] point, int h, double deg)
   {
      setUpRampBlock(point[0], point[1], h, deg);
   }

   private void setUpSkewedUprightBlockSquare(double[] point, int h, double deg)
   {
      setUpSkewedUprightBlockSquare(point[0], point[1], h, deg);
   }

   private void setUpFlatSkewedBlockSquare(double[] point, int h, double deg)
   {
      setUpFlatSkewedBlockSquare(point[0], point[1], h, deg);
   }

   private void setUpCinderBlockSquare(double[] point, int h, double deg)
   {
      setUpCinderBlockSquare(point[0], point[1], h, deg);
   }

   private void setUpPath5NarrowDoor()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      // angled Door
      // door1
      setUpWall(new double[] {0.769f, -9.293f}, 0.973f, 0.157f, 2.5f, -115.0f, color);

      // door2
      setUpWall(new double[] {-.642f, -8.635f}, 0.973f, 0.157f, 2.54f, -115.0f, color);

      // box2
      setUpWall(new double[] {-0.485f, -6.573f}, 0.5f, 0.5f, 1.0f, -45, color);

      // box1
      setUpWall(new double[] {0.515f, -4.972f}, 0.5f, 0.5f, 1.0f, -110.0f, color);

   }

   private void setUpPath6Barriers()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();
      double courseAngle = -135.0;
      int numberOfStepOvers = 8;
      double heightIncrease = 0.05;
      double startDistance = 4.0;
      double spacing = 1.0;

      double barrierWidth = 3.0;
      double platformWidth = 0.8;

      for (int i = 0; i < numberOfStepOvers; i++)
      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * spacing), 0}, courseAngle);
         setUpWall(newPoint, barrierWidth, 0.15, heightIncrease * (i + 1), courseAngle, color);
      }

      for (int i = 0; i < numberOfStepOvers; i++)
      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * spacing), (barrierWidth - platformWidth) / 2.0 + 0.001}, courseAngle);
         setUpWall(newPoint, platformWidth, 0.4 * spacing, heightIncrease * (i + 1) + 0.001, courseAngle, color);
      }

   }

   private void setUpPath7Stairs()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();
      double courseAngle = 135;
      int numberOfSteps = 3;
      double rise = 0.2;
      double startDistance = 4.0;
      double run = 0.4;

      for (int i = 0; i < numberOfSteps; i++)
      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * run), 0}, courseAngle);
         setUpWall(newPoint, 3.0, run, rise * (i + 1), courseAngle, color);
      }

      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (numberOfSteps * run), 0}, courseAngle);
         setUpWall(newPoint, 3.0, run, rise * (numberOfSteps - 1 + 1), courseAngle, color);
      }

      for (int i = 1; i < numberOfSteps + 1; i++)
      {
         double offset = numberOfSteps * run;
         double[] newPoint = rotateAroundOrigin(new double[] {offset + startDistance + (i * run), 0}, courseAngle);
         setUpWall(newPoint, 3.0, run, rise * (-i + numberOfSteps + 1), courseAngle, color);
      }
   }

   private void setUpPath8RampsWithSteppingStones()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      float rampHeight = 0.3f;

      // ramp up and landing
      setUpRamp(-5.0f, 0.0f, 3.0f, -3.0f, rampHeight, color);
      setUpWall(new double[] {-7.0f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

      // simple stepping stones, centered at x=-0.75m
      setUpWall(new double[] {-7.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-8.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-9.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);

      // qualification stepping stones, centered along x=0.75m
      setUpWall(new double[] {-8.0f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-8.5f, 0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(new double[] {-9.3f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);

      // middle landing
      setUpWall(new double[] {-10.5f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

      if (DIFFICULT_STEPPING_STONES)
      {
         // more difficult stepping stones
         setUpWall(new double[] {-11.6f, -0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(new double[] {-12.2f, 0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(new double[] {-13.1f, 0.15f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(new double[] {-14f, 0.95f}, 0.5f, 0.5f, rampHeight, 0, color);

         // landing and ramp down
         setUpWall(new double[] {-15.5f, 0.5f}, 2.0f, 1.0f, rampHeight, 0, color);
         setUpRamp(-17.5f, 0.5f, 2.0f, 3.0f, rampHeight, color);
      }
      else
      {
         setUpRamp(-12.5f, 0.0f, 3.0f, 3.0f, rampHeight, color);
      }

      // Do this for a long ramp for testing:
      // rampHeight = 1.0f;
      // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);
   }

   private void setUpGround()
   {
      // AppearanceDefinition app = YoAppearance.Gray();

      // center
      // setUpCone(0, 0, 1.5, 1.5, FLOOR_THICKNESS, app);

      // filler

      // setUpCone(0, 0, 12.5, 14.5, 0.0001, app);

      // setUpCone(0, 0, 10, 12, 0.005, YoAppearance.Brown());

      URL fileURL = DRCDemo01NavigationEnvironment.class.getResource("Textures/ground2.png");
      YoAppearanceTexture texture = new YoAppearanceTexture(fileURL);

      Transform3D location = new Transform3D();
      location.setTranslation(new Vector3d(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 200, 200, 0.75), YoAppearance.DarkGray());
      combinedTerrainObject.addTerrainObject(newBox2);

   }

   private void conditionallyAddLimboBar()
   {
      if (addLimboBar)
      {
         double height = 1;
         double width = 1.5;
         AppearanceDefinition color = YoAppearance.DarkGray();

         setUpWall(new double[] {1, width / 2}, 0.125, 0.125, height, 0, color);
         setUpWall(new double[] {1, -width / 2}, 0.125, 0.125, height, 0, color);

         combinedTerrainObject.getLinkGraphics().translate(0, width / 2, height);
         combinedTerrainObject.getLinkGraphics().addCube(0.125, width, 0.125, color);
         combinedTerrainObject.getLinkGraphics().translate(0, -width / 2, -height);
      }
   }

   private void addRocks()
   {
      for (int i = 0; i < NUM_ROCKS; i++)
      {
         double centroidHeight = random.nextDouble() * MAX_ROCK_CENTROID_HEIGHT;
         Vector3d normal = generateRandomUpFacingNormal();

         double[] approximateCentroid = generateRandomApproximateCentroid(i);

         double[][] vertices = generateRandomRockVertices(approximateCentroid[0], approximateCentroid[1]);

         addRock(normal, centroidHeight, vertices);
      }
   }

   private double[] rotateAroundOrigin(double[] xy, double angdeg)
   {
      double x = xy[0];
      double y = xy[1];
      double[] newPoint = new double[2];
      double angRad = Math.toRadians(angdeg);
      newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
      newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);

      return newPoint;
   }

   private double[] generateRandomApproximateCentroid(int position)
   {
      double[] approximateCentroid = new double[2];

      if (FULLY_RANDOM)
      {
         approximateCentroid[0] = random.nextDouble() * ROCK_FIELD_WIDTH - ROCK_FIELD_WIDTH / 2.0;
         approximateCentroid[1] = random.nextDouble() * ROCK_PATH_LENGTH + ROCKS_START_Y;

      }
      else
      {
         int row = position / ROCKS_PER_ROW;
         int rows = NUM_ROCKS / ROCKS_PER_ROW;
         double distancePerRow = ROCK_PATH_LENGTH / ((double) rows - 1);
         approximateCentroid[1] = ROCKS_START_Y + distancePerRow * row;

         int positionOnRow = position - row * ROCKS_PER_ROW;
         approximateCentroid[0] = ROCK_FIELD_WIDTH * ((double) positionOnRow) / ((double) ROCKS_PER_ROW) - ROCK_FIELD_WIDTH / 2.0;
      }

      return approximateCentroid;
   }

   private Vector3d generateRandomUpFacingNormal()
   {
      double normalX = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      double normalY = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      Vector3d normal = new Vector3d(normalX, normalY, 1.0);

      return normal;
   }

   private double[][] generateRandomRockVertices(double approximateCentroidX, double approximateCentroidY)
   {
      double[][] vertices = new double[POINTS_PER_ROCK][2];

      for (int j = 0; j < POINTS_PER_ROCK; j++)
      {
         vertices[j][0] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidX - ROCK_BOUNDING_BOX_WIDTH / 2.0;
         vertices[j][1] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidY - ROCK_BOUNDING_BOX_WIDTH / 2.0;
      }

      return vertices;
   }

   private void addRock(Vector3d normal, double centroidHeight, double[][] vertices)
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      ArrayList<Point2d> vertexPoints = new ArrayList<Point2d>();

      for (double[] point : vertices)
      {
         Point2d point2d = new Point2d(point);
         vertexPoints.add(point2d);
      }

      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
      RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, color);
      this.combinedTerrainObject.addTerrainObject(rock);
   }

   private void setUpWall(double[] xy, double width, double length, double height, double yawDegrees, AppearanceDefinition app)
   {
      double x = xy[0];
      double y = xy[1];
      Transform3D location = new Transform3D();
      location.rotZ(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpCone(double x, double y, double bottomWidth, double topWidth, double height, AppearanceDefinition app)
   {
      combinedTerrainObject.addCone(x, y, bottomWidth, topWidth, height, app);
   }

   private void setUpRamp(double x, double y, double width, double length, double height, AppearanceDefinition app)
   {
      combinedTerrainObject.addRamp(x - length / 2, y - width / 2, x + length / 2, y + width / 2, height, app);
   }

   private void setUpRotatedRamp(double xCenter, double yCenter, double width, double run, double rise, double yawDegreesAboutCenter, AppearanceDefinition app)
   {
      combinedTerrainObject.addRotatedRamp(xCenter - run / 2, yCenter - width / 2, xCenter + run / 2, yCenter + width / 2, rise, yawDegreesAboutCenter, app);
   }

   // need basics:
   // basic ramp
   // basic block (height parameter: # block layers, 1-2 typ)
   // square block (two basic blocks side by side, height parameter: # block layers, 0-4 typ)
   // diagonal block-flat (# of square block base supports, 1 typ, e.g. 1 square block under)
   // diagonal block-upright (# of square block base supports, 0 and 1 typ)
   // slanted block (square block on ramp, with # of square block support layers: 0-3 typ)

   private void setUpCinderBlock(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double[] centerPoint = {xCenter, yCenter};
      setUpCinderBlock(centerPoint, numberFlatSupports, yawDegrees);
   }

   private void setUpCinderBlock(double[] centerPoint, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      AppearanceDefinition app = cinderBlockAppearance;

      double xCenter = centerPoint[0];
      double yCenter = centerPoint[1];

      // wall
      Transform3D location = new Transform3D();
      location.rotZ(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(xCenter, yCenter, cinderBlockHeight / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth, cinderBlockHeight), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpSlopedCinderBlock(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      AppearanceDefinition app = cinderBlockAppearance;

      Transform3D location = new Transform3D();
      location.rotZ(Math.toRadians(yawDegrees));

      Transform3D tilt = new Transform3D();
      tilt.rotY(-cinderBlockTiltRadians);
      location.mul(tilt);

      double zCenter = (cinderBlockHeight * Math.cos(cinderBlockTiltRadians) + cinderBlockLength * Math.sin(cinderBlockTiltRadians)) / 2;
      location.setTranslation(new Vector3d(xCenter, yCenter, zCenter + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth, cinderBlockHeight), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpCinderBlockSquare(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      double xOffset = 0, yOffset = cinderBlockWidth / 2;
      double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);

      setUpCinderBlock(xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
      setUpCinderBlock(xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);

      if (numberFlatSupports > 0)
         setUpCinderBlockSquare(xCenter, yCenter, numberFlatSupports - 1, yawDegrees + 90);
   }

   private void setUpFlatSkewedBlockSquare(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      setUpCinderBlockSquare(xCenter, yCenter, numberFlatSupports - 1, yawDegrees);
      setUpCinderBlock(xCenter, yCenter, numberFlatSupports, yawDegrees - 45);
   }

   private void setUpCinderBlockUpright(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      AppearanceDefinition app = cinderBlockAppearance;

      // wall
      Transform3D location = new Transform3D();
      Transform3D setUpright = new Transform3D();

      location.rotZ(Math.toRadians(yawDegrees));
      setUpright.rotX(Math.toRadians(90));
      location.mul(setUpright);

      location.setTranslation(new Vector3d(xCenter, yCenter, cinderBlockWidth / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, cinderBlockLength, cinderBlockWidth, cinderBlockHeight), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpSkewedUprightBlockSquare(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      setUpCinderBlockSquare(xCenter, yCenter, numberFlatSupports - 1, yawDegrees);
      setUpCinderBlockUpright(xCenter, yCenter, numberFlatSupports, yawDegrees - 45);
   }

   private void setUpRampBlock(double xCenter, double yCenter, int numberFlatSupports, double yawDegrees)
   {
      if (numberFlatSupports < 0)
         return;

      setUpCinderBlockSquare(xCenter, yCenter, numberFlatSupports - 1, yawDegrees);


      // double rampRun = cinderBlockLength * Math.cos(cinderBlockTiltRadians);
      double rampRise = cinderBlockLength * Math.sin(cinderBlockTiltRadians);

      // double xRampOffset = (cinderBlockLength - rampRun) / 2, yRampOffset = 0;
      // double[] xyRampRotatedOffset = rotateAroundOrigin(xRampOffset, yRampOffset, yawDegrees);
      // double xRampCenter = xCenter + xyRampRotatedOffset[0];
      // double yRampCenter = yCenter + xyRampRotatedOffset[1];
      // TO DO: set ramp elevation
      // setUpRotatedRamp(xRampCenter, yRampCenter, 2 * cinderBlockWidth, rampRun, rampRise, yawDegrees, cinderBlockAppearance);

      // Create a block instead. Ramp3d only has collision surface of ramp itself. And setUpRotateRamp() can't change height yet.
      // A block however has all the needed collision surfaces, even if it overlaps the block above it.
      Transform3D blockSupportLocation = new Transform3D();
      blockSupportLocation.rotZ(Math.toRadians(yawDegrees));
      double[] xySupportRotatedOffset = rotateAroundOrigin(new double[] {(cinderBlockLength - rampRise) / 2, 0}, yawDegrees);
      blockSupportLocation.setTranslation(new Vector3d(xCenter + xySupportRotatedOffset[0], yCenter + xySupportRotatedOffset[1],
              rampRise / 2 + numberFlatSupports * cinderBlockHeight));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(blockSupportLocation, rampRise, cinderBlockLength, rampRise),
                                            cinderBlockAppearance);
      combinedTerrainObject.addTerrainObject(newBox);


      double xOffset = 0, yOffset = cinderBlockWidth / 2;
      double[] xyRotated1 = rotateAroundOrigin(new double[] {xOffset, yOffset}, yawDegrees);
      double[] xyRotated2 = rotateAroundOrigin(new double[] {xOffset, -yOffset}, yawDegrees);
      setUpSlopedCinderBlock(xCenter + xyRotated1[0], yCenter + xyRotated1[1], numberFlatSupports, yawDegrees);
      setUpSlopedCinderBlock(xCenter + xyRotated2[0], yCenter + xyRotated2[1], numberFlatSupports, yawDegrees);
   }


   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public ArrayList<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub

   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
      // TODO Auto-generated method stub

   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

}
