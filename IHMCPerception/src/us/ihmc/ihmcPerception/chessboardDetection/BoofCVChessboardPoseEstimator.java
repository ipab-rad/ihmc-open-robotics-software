package us.ihmc.ihmcPerception.chessboardDetection;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.point.Vector3D_F64;
//import georegression.struct.se.Se3_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class BoofCVChessboardPoseEstimator
{
   final boolean VISUALIZE = true;

   IntrinsicParameters intrinsicParameters = null;
   FiducialDetector<ImageFloat32> detector;
   double gridWidth;

   public BoofCVChessboardPoseEstimator(int rows, int cols, double gridWidth)
   {
      ConfigChessboard config;
      this.gridWidth=gridWidth;
      config = new ConfigChessboard(cols, rows, gridWidth);
      detector = FactoryFiducial.calibChessboard(config, ImageFloat32.class);
   }
   
   public void setIntrinsicParameter(IntrinsicParameters parameter)
   {
      this.intrinsicParameters = new IntrinsicParameters(parameter);
   }

   private void setDefaultIntrinsicParameter(int height, int width, double fovYinRadian)
   {
      double f = height /2.0 / Math.tan(fovYinRadian/2.0);
      intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = width;
      intrinsicParameters.height = height;
      intrinsicParameters.cx = width / 2;
      intrinsicParameters.cy = height / 2;
      intrinsicParameters.fx = f;
      intrinsicParameters.fy = f;
   }
   public RigidBodyTransform detect(BufferedImage image)
   {
      if(intrinsicParameters==null)
         setDefaultIntrinsicParameter(image.getHeight(), image.getWidth(), Math.PI/4);
      ImageFloat32 gray = new ImageFloat32(image.getWidth(), image.getHeight());
      ConvertBufferedImage.convertFrom(image, gray);
      detector.setIntrinsic(intrinsicParameters);
      detector.detect(gray);


      // display the results
      if (detector.totalFound() == 0)
      {
         return null;
      }
      else
      {
         int closestIndex = -1;
         double closestDistance = Double.MAX_VALUE;
         Se3_F64 targetToSensor = new Se3_F64();
         for (int i = 0; i < detector.totalFound(); i++)
         {
            detector.getFiducialToCamera(i, targetToSensor);
            double dist = targetToSensor.getT().norm();
            if (dist < closestDistance)
            {
               closestIndex = i;
               closestDistance = dist;
            }
         }
         detector.getFiducialToCamera(closestIndex, targetToSensor);
         Vector3D_F64 translation = targetToSensor.getT();
         Matrix3d rotation = new Matrix3d(targetToSensor.getR().data);
         

         RigidBodyTransform transform=new RigidBodyTransform(rotation, new Vector3d(translation.x, translation.y, translation.z));
         return transform;
      }

   }
   
   public void drawBox(BufferedImage image, RigidBodyTransform transform, double scale)
   {
      DenseMatrix64F rotation =new DenseMatrix64F(3,3);
      Vector3d translation = new Vector3d();
      transform.getRotation(rotation);
      transform.getTranslation(translation);
      Se3_F64 targetToSensor = new Se3_F64(rotation,new Vector3D_F64(translation.x, translation.y, translation.z));

      Graphics2D g2 = image.createGraphics();
      g2.setStroke(new BasicStroke(4));
      g2.setColor(java.awt.Color.RED);
      VisualizeFiducial.drawCube(targetToSensor, this.intrinsicParameters, scale, 2, g2);
      g2.finalize();

      
   }
}
