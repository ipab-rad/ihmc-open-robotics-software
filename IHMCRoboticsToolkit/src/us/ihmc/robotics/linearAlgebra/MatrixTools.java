package us.ihmc.robotics.linearAlgebra;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixIO;

import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class MatrixTools
{
   public final static Matrix3d IDENTITY = new Matrix3d();

   static
   {
      IDENTITY.setIdentity();
   }

   /**
    * Sets all the entries of a matrix to NaN
    *
    * @param matrix
    */
   public static void setToNaN(DenseMatrix64F matrix)
   {
      setToValue(matrix, Double.NaN);
   }

   /**
    * Sets all entries of a matrix to zero
    *
    * @param matrix
    */
   public static void setToZero(DenseMatrix64F matrix)
   {
      setToValue(matrix, 0.0);
   }

   private static void setToValue(DenseMatrix64F matrix, double val)
   {
      for (int i = 0; i < matrix.numRows; i++)
      {
         for (int j = 0; j < matrix.numCols; j++)
         {
            matrix.unsafe_set(i, j, val);
         }
      }
   }

   /**
    * This method tries to be smart about converting the various yaml fields to DenseMatrix64F
    * @param val
    * @param fieldName
    * @param object
    * 
    *   Map<String, Object> object = (Map<String, Object>) yaml.load(input);
    *   yamlFieldToMatrix(beq,"beq",object);
    */
   public static DenseMatrix64F yamlFieldToMatrix(DenseMatrix64F val, String fieldName, Map<String, Object> object)
   {
      if (val == null)
         val = new DenseMatrix64F(1, 1);
      if (object.get(fieldName) instanceof ArrayList<?>)
      {
         ArrayList<?> arrayList = (ArrayList<?>) object.get(fieldName);
         try
         {
            if (arrayList.get(0) instanceof ArrayList)
            {
               @SuppressWarnings("unchecked")
               ArrayList<ArrayList<Double>> tmp2DArrayList = (ArrayList<ArrayList<Double>>) arrayList;
               // 2D
               val.reshape(tmp2DArrayList.size(), tmp2DArrayList.get(0).size());
               for (int i = 0; i < tmp2DArrayList.size(); i++)
               {
                  for (int j = 0; j < tmp2DArrayList.get(0).size(); j++)
                  {
                     val.set(i, j, tmp2DArrayList.get(i).get(j));
                  }
               }
            }
            else
            {
               @SuppressWarnings("unchecked")
               ArrayList<Double> tmp1DArrayList = (ArrayList<Double>) arrayList;
               // 1D
               val.reshape(1, arrayList.size());
               for (int i = 0; i < tmp1DArrayList.size(); i++)
               {
                  val.set(0, i, tmp1DArrayList.get(i));
               }
            }
         }
         catch (Exception e)
         {
            //Field is empty
            val = null;
         }
      }
      else if (object.get(fieldName) instanceof Double)
      {
         val.reshape(1, 1);
         Double tmpDouble = (Double) object.get(fieldName);

         val.set(0, 0, tmpDouble);
      }
      else
      {
         throw new RuntimeException("Unsupported data type:" + object.get(fieldName).getClass());
      }
      return val;
   }

   public static boolean isEmptyMatrix(DenseMatrix64F m)
   {
      if (m == null)
         throw new RuntimeException("Matrix is null");
      if (m.numCols == 0 && m.numRows == 0)
         return true;
      else
         return false;
   }

   /** Same as CommonOps.mult but allow a,b,c to be zero rol/col matrices
    *  c = a * b;
    */

   public static void multAllowEmptyMatrix(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numRows != c.numRows || b.numCols != c.numCols)
         throw new RuntimeException("matrix c dimension should be " + a.numRows + " x " + b.numCols);

      if (a.numCols != b.numRows)
         throw new RuntimeException("a, b matrices are not compatible, check their dimensions");

      if (a.numCols == 0 && b.numRows == 0)
         c.zero();
      else
         CommonOps.mult(a, b, c);
   }

   /**
    *  same as CommonOps.multAdd but allow a,b,c to be empty matrices
    */

   public static void multAddAllowEmptyMatrix(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numRows != c.numRows || b.numCols != c.numCols)
         throw new RuntimeException("matrix c dimension should be " + a.numRows + " x " + b.numCols);

      if (a.numCols != b.numRows)
         throw new RuntimeException("a, b matrices are not compatible, check their dimensions");

      if (a.numRows != 0 && b.numCols != 0)
         CommonOps.multAdd(a, b, c);
   }

   /**
    *  fill column col of matrix m with value val
    */
   public static void fillColumn(DenseMatrix64F m, int col, double val)
   {
      for (int i = 0; i < m.numRows; i++)
         m.set(i, col, val);
   }

   /**
    *  set matrixToSet from vectorElements
    */

   public static void setVector(DenseMatrix64F vectorToSet, double[] vectorElements)
   {
      if (vectorToSet.numCols != 1)
         throw new RuntimeException("first argument has to be column vector");
      for (int i = 0; i < vectorElements.length; i++)
      {
         vectorToSet.set(i, 0, vectorElements[i]);
      }
   }

   /**
    *  return a newVector based on vectorElements 
    */
   public static DenseMatrix64F createVector(double[] vectorElements)
   {
      DenseMatrix64F ret = new DenseMatrix64F(vectorElements.length, 1);
      setVector(ret, vectorElements);

      return ret;
   }

   /**
    *  find the index of the first largest element in a vector
    *  in the range of [startIndex, endIndex), not including the endIndex
    */
   public static int findMaxElementIndex(double m[], int startIndex, int endIndex)
   {
      double maxVal = Double.NEGATIVE_INFINITY;
      int maxIndex = -1;
      for (int i = startIndex; i < endIndex; i++)
      {
         if (m[i] > maxVal)
         {
            maxVal = m[i];
            maxIndex = i;
         }
      }
      return maxIndex;
   }

   /**
    * AtGA = A' * G * A
    * 
    * if AtGA is null, a new Matrix will be allocated and returned
    */
   public static DenseMatrix64F multQuad(DenseMatrix64F A, DenseMatrix64F G, DenseMatrix64F AtGA)
   {
      DenseMatrix64F out;
      if (AtGA == null)
      {
         out = new DenseMatrix64F(A.numCols, A.numCols);
      }
      else
      {
         out = AtGA;
      }

      DenseMatrix64F tmp = new DenseMatrix64F(A.numCols, G.numCols);
      CommonOps.multTransA(A, G, tmp);
      CommonOps.mult(tmp, A, out);
      return out;
   }

   public static double multMatrixRowVector(DenseMatrix64F a, int row, DenseMatrix64F b)
   {
      if (a.numCols != b.numRows)
         throw new RuntimeException("a.numCols should be equal to b.numRows");
      if (b.numCols != 1)
         throw new RuntimeException("b should be a column vector");

      int rowHeadIndex = a.getIndex(row, 0);
      double total = 0;
      for (int i = 0; i < a.numCols; i++)
      {
         total += a.get(rowHeadIndex + i) * b.get(i);
      }

      return total;

   }

   static enum Relation
   {
      LESSTHAN, EQUAL, GREATERTHAN
   };

   public static DenseMatrix64F compare(DenseMatrix64F a, Relation r, DenseMatrix64F b, double eps)
   {
      assert (a.numCols == b.numCols && a.numRows == b.numRows);
      DenseMatrix64F result = new DenseMatrix64F(a.numRows, a.numCols);
      for (int i = 0; i < a.numCols; i++)
         for (int j = 0; j < a.numRows; j++)
         {

            double testVal = a.get(i, j) - b.get(i, j);
            switch (r)
            {
            case LESSTHAN:
               result.unsafe_set(i, j, testVal < 0 ? 1 : 0);
               break;
            case EQUAL:
               result.unsafe_set(i, j, Math.abs(testVal) < eps ? 1 : 0);
               break;
            case GREATERTHAN:
               result.unsafe_set(i, j, testVal > 0 ? 1 : 0);
               break;
            default:
               throw new RuntimeException("shoun't be here");
            }
         }
      return result;
   }

   /**
    * Set a column of a Matrix to an Array
    *
    * @param matrix Matrix to set
    * @param column Column
    * @param columnValues
    */
   public static void setMatrixColumnFromArray(DenseMatrix64F matrix, int column, double[] columnValues)
   {
      for (int i = 0; i < matrix.numRows; i++)
      {
         matrix.unsafe_set(i, column, columnValues[i]);
      }
   }

   /**
    * Set a column of a Matrix to an Array
    *
    * @param matrix Matrix to set
    * @param column Column
    * @param columnValues
    */
   public static void setMatrixColumnFromArray(DenseMatrix64F matrix, int column, double[] columnValues, int startRow)
   {
      for (int i = startRow; i < columnValues.length; i++)
      {
         matrix.unsafe_set(i, column, columnValues[i]);
      }
   }

   public static void setMatrixFromOneBasedArray(DenseMatrix64F ret, double[] oneBasedArray)
   {
      for (int i = 0; i < oneBasedArray.length - 1; i++)
      {
         ret.set(i, 0, oneBasedArray[i + 1]);
      }
   }

   public static void setOneBasedArrayFromMatrix(double[] oneBasedArrayToPack, DenseMatrix64F matrix)
   {
      int index = 1;
      for (int i = 0; i < matrix.getNumRows(); i++)
      {
         oneBasedArrayToPack[index++] = matrix.get(i, 0);
      }
   }

   /**
    *
    * Differentiates a row vector by subtracting the previous element from the current element
    *
    * @param vectorToDiff  Row vector
    * @param startRow   Row to start at
    * @param numberOfRows  Rows to differentiate
    * @param vectorToPack  Result row vector
    */
   public static void diff(DenseMatrix64F vectorToDiff, int startRow, int numberOfRows, DenseMatrix64F vectorToPack)
   {
      for (int i = 1; i < numberOfRows; i++)
      {
         vectorToPack.unsafe_set(i - 1, 0, vectorToDiff.unsafe_get(startRow + i, 0) - vectorToDiff.unsafe_get(startRow + i - 1, 0));
      }
   }

   /**
    * Differentiates an array by subtracting the previous element from the current element
    *
    * @param vectorToDiff
    * @param vectorToPack  Result row vector
    */
   public static void diff(double[] vectorToDiff, DenseMatrix64F vectorToPack)
   {
      for (int i = 1; i < vectorToDiff.length; i++)
      {
         vectorToPack.unsafe_set(i - 1, 0, vectorToDiff[i] - vectorToDiff[i - 1]);
      }
   }

   public static void numericallyDifferentiate(DenseMatrix64F derivativeToPack, DenseMatrix64F previousMatrixToUpdate, DenseMatrix64F newMatrix, double dt)
   {
      derivativeToPack.set(newMatrix);
      CommonOps.subtractEquals(derivativeToPack, previousMatrixToUpdate);
      CommonOps.scale(1.0 / dt, derivativeToPack);
      previousMatrixToUpdate.set(newMatrix);
   }

   /**
    *
    * Sets a block of a matrix
    *
    * @param dest Set a block of this matrix
    * @param destStartRow Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src Get a block of this matrix
    * @param srcStartRow Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale Scale the block from otherMatrix by this value
    *
    */
   public static void setMatrixBlock(DenseMatrix64F dest, int destStartRow, int destStartColumn, DenseMatrix64F src, int srcStartRow, int srcStartColumn,
         int numberOfRows, int numberOfColumns, double scale)
   {
      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j, scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   /**
    * Sets matrixToPack to the entries of input in the specified rows and columns
    *
    * @param matrixToPack matrix to pack
    * @param input input matrix
    * @param rows rows of input matrix to use in setting matrix to pack
    * @param columns columns of input matrix to use in setting matrix to pack
    */
   public static void getMatrixBlock(DenseMatrix64F matrixToPack, DenseMatrix64F input, int[] rows, int[] columns)
   {
      if ((rows.length != matrixToPack.getNumRows()) || (columns.length != matrixToPack.getNumCols()))
      {
         throw new RuntimeException("The size of matrixToPack is not rows.length * columns.length");
      }

      int newI = 0;
      for (int i : rows)
      {
         int newJ = 0;
         for (int j : columns)
         {
            matrixToPack.set(newI, newJ, input.get(i, j));
            newJ++;
         }

         newI++;
      }
   }

   /**
    *
    * Adds to a block of a matrix
    *
    * @param dest Add to a block of this matrix
    * @param destStartRow Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src Get a block of this matrix
    * @param srcStartRow Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale Scale the block from otherMatrix by this value
    *
    */
   public static void addMatrixBlock(DenseMatrix64F dest, int destStartRow, int destStartColumn, DenseMatrix64F src, int srcStartRow, int srcStartColumn,
         int numberOfRows, int numberOfColumns, double scale)
   {
      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j,
                  dest.unsafe_get(destStartRow + i, destStartColumn + j) + scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   public static void extractColumns(DenseMatrix64F source, int[] srcColumns, DenseMatrix64F dest, int destStartColumn)
   {
      for (int i : srcColumns)
      {
         CommonOps.extract(source, 0, source.getNumRows(), i, i + 1, dest, 0, destStartColumn);
         destStartColumn++;
      }
   }
   
   public static void extractColumns(DenseMatrix64F source, TIntArrayList srcColumns, DenseMatrix64F dest, int destStartColumn)
   {
      for (int i = 0; i < srcColumns.size(); i++)
      {
         int index = srcColumns.get(i);
         CommonOps.extract(source, 0, source.getNumRows(), index, index + 1, dest, 0, destStartColumn);
         destStartColumn++;
      }
   }

   public static void linspace(double[] vector, double d1, double d2, int n)
   {
      // double[] vector = new double[n];

      for (int i = 0; i < n; i++)
      {
         vector[i] = d1 + i * ((d2 - d1) / ((double) (n - 1)));
      }

      // return vector;

   }

   public static double[] linspace(double d1, double d2, int n)
   {
      double[] vector = new double[n];
      linspace(vector, d1, d2, n);

      return vector;

   }

   public static DenseMatrix64F flipUpDown(DenseMatrix64F matrix)
   {
      DenseMatrix64F ret = matrix.copy();
      int nRows = ret.getNumRows();
      int nColumns = ret.getNumCols();

      for (int i = 0; i < nRows; i++)
      {
         int newRowNumber = nRows - i - 1;
         for (int j = 0; j < nColumns; j++)
         {
            ret.set(newRowNumber, j, matrix.get(i, j));
         }
      }

      return ret;
   }

   public static void denseMatrixToMatrix3d(DenseMatrix64F denseMatrix, Matrix3d matrixToPack, int startRow, int startColumn)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrixToPack.setElement(row, column, denseMatrix.get(row + startRow, column + startColumn));
         }
      }
   }

   public static void matrix3DToDenseMatrix(Matrix3d matrix, DenseMatrix64F denseMatrixToPack, int startRow, int startColumn)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            denseMatrixToPack.unsafe_set(row + startRow, column + startColumn, matrix.getElement(row, column));
         }
      }
   }

   public static void denseMatrixToVector3d(DenseMatrix64F denseMatrix, Tuple3d vectorToPack, int startRow, int startColumn)
   {
      vectorToPack.setX(denseMatrix.get(startRow + 0, startColumn));
      vectorToPack.setY(denseMatrix.get(startRow + 1, startColumn));
      vectorToPack.setZ(denseMatrix.get(startRow + 2, startColumn));
   }

   public static DenseMatrix64F mult(DenseMatrix64F A, DenseMatrix64F B)
   {
      DenseMatrix64F C = new DenseMatrix64F(A.getNumRows(), B.getNumCols());
      CommonOps.mult(A, B, C);

      return C;
   }

   public static void addDiagonal(DenseMatrix64F matrix, double scalar)
   {
      int n = Math.max(matrix.getNumRows(), matrix.getNumCols());
      for (int i = 0; i < n; i++)
      {
         matrix.add(i, i, scalar);
      }
   }

   /**
    * Set diagonal elements of matrix to diagValues
    *
    * @param matrix
    * @param diagValues
    */
   public static void setMatrixDiag(Matrix3d matrix, double[] diagValues)
   {
      for (int i = 0; i < 3; i++)
      {
         matrix.setElement(i, i, diagValues[i]);
      }
   }

   /**
    * Returns the resulting matrix from vector1*transpose(vector2)
    *
    * @param vector1
    * @param vector2
    */
   public static Matrix3d vectorTimesVectorTranspose(Vector3d vector1, Vector3d vector2)
   {
      Matrix3d matrix = new Matrix3d();

      double[] array1 = new double[3];
      double[] array2 = new double[3];

      vector1.get(array1);
      vector2.get(array2);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrix.setElement(row, column, array1[row] * array2[column]);
         }
      }

      return matrix;
   }

   /**
    * Returns the resulting matrix from vector*transpose(vector)
    *
    * @param vector
    */
   public static Matrix3d vectorTimesVectorTranspose(Vector3d vector)
   {
      return vectorTimesVectorTranspose(vector, vector);
   }

   public static void vectorToSkewSymmetricMatrix(DenseMatrix64F matrixToPack, Tuple3d tuple)
   {
      matrixToPack.set(0, 0, 0.0);
      matrixToPack.set(0, 1, -tuple.getZ());
      matrixToPack.set(0, 2, tuple.getY());

      matrixToPack.set(1, 0, tuple.getZ());
      matrixToPack.set(1, 1, 0.0);
      matrixToPack.set(1, 2, -tuple.getX());

      matrixToPack.set(2, 0, -tuple.getY());
      matrixToPack.set(2, 1, tuple.getX());
      matrixToPack.set(2, 2, 0.0);
   }

   /**
    * Converts a vector to tilde form (matrix implementation of cross product)
    */
   public static void toTildeForm(Matrix3d matrixToPack, Tuple3d p)
   {
      matrixToPack.setElement(0, 0, 0.0);
      matrixToPack.setElement(0, 1, -p.getZ());
      matrixToPack.setElement(0, 2, p.getY());

      matrixToPack.setElement(1, 0, p.getZ());
      matrixToPack.setElement(1, 1, 0.0);
      matrixToPack.setElement(1, 2, -p.getX());

      matrixToPack.setElement(2, 0, -p.getY());
      matrixToPack.setElement(2, 1, p.getX());
      matrixToPack.setElement(2, 2, 0.0);
   }

   /*
    * M = \tilde{a} * \tilde{b}
    */
   public static void setTildeTimesTilde(Matrix3d M, Tuple3d a, Tuple3d b)
   {
      double axbx = a.x * b.x;
      double ayby = a.y * b.y;
      double azbz = a.z * b.z;

      M.m00 = -azbz - ayby;
      M.m01 = a.y * b.x;
      M.m02 = a.z * b.x;

      M.m10 = a.x * b.y;
      M.m11 = -axbx - azbz;
      M.m12 = a.z * b.y;

      M.m20 = a.x * b.z;
      M.m21 = a.y * b.z;
      M.m22 = -axbx - ayby;
   }

   public static void setDenseMatrixFromMatrix3d(int startRow, int startColumn, Matrix3d fromMatrix3d, DenseMatrix64F denseMatrix)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            denseMatrix.set(i + startRow, j + startColumn, fromMatrix3d.getElement(i, j));
         }
      }
   }

   public static void setMatrixColumnToVector(int columnIndex, DenseMatrix64F Matrix, DenseMatrix64F vector)
   {
      for (int i = 0; i < Matrix.getNumRows(); i++)
      {
         Matrix.set(i, columnIndex, vector.get(i, columnIndex));
      }
   }

   public static void setMatrixRowToVector(int rowIndex, DenseMatrix64F Matrix, DenseMatrix64F vector)
   {
      for (int i = 0; i < Matrix.getNumCols(); i++)
      {
         Matrix.set(rowIndex, i, vector.get(i));
      }
   }

   public static void tranformSe3IntoTransform3D(Se3_F64 from, RigidBodyTransform to)
   {

      DenseMatrix64F R = from.getR();
      Vector3D_F64 T = from.getT();

      Matrix3d Rd = new Matrix3d();
      denseMatrixToMatrix3d(R, Rd, 0, 0);

      to.setRotation(Rd);
      to.setTranslation(new Vector3d(T.x, T.y, T.z));
   }

   public static void transformFramePoint2dIntoColumnVector(DenseMatrix64F matrix, FramePoint2d framePoint)
   {
      matrix.set(0, 0, framePoint.getX());
      matrix.set(1, 0, framePoint.getY());
   }

   public static void transformColumnVectorIntoFramePoint2d(DenseMatrix64F matrix, FramePoint2d framePoint)
   {
      framePoint.setX(matrix.get(0, 0));
      framePoint.setY(matrix.get(1, 0));
   }

   public static void transformFrameVector2dIntoColumnVector(DenseMatrix64F matrix, FrameVector2d frameVector)
   {
      matrix.set(0, 0, frameVector.getX());
      matrix.set(1, 0, frameVector.getY());
   }

   public static void transformColumnVectorIntoFrameVector2d(DenseMatrix64F matrix, FrameVector2d frameVector)
   {
      frameVector.setX(matrix.get(0, 0));
      frameVector.setY(matrix.get(1, 0));
   }

   public static void transformFramePoint2dIntoRowVector(DenseMatrix64F matrix, FramePoint2d framePoint)
   {
      matrix.set(0, 0, framePoint.getX());
      matrix.set(0, 1, framePoint.getY());
   }

   public static void transformRowVectorIntoFramePoint2d(DenseMatrix64F matrix, FramePoint2d framePoint)
   {
      framePoint.setX(matrix.get(0, 0));
      framePoint.setX(matrix.get(0, 1));
   }

   public static void transformFrameVector2dIntoRowVector(DenseMatrix64F matrix, FrameVector2d frameVector)
   {
      matrix.set(0, 0, frameVector.getX());
      matrix.set(0, 1, frameVector.getY());
   }

   public static void transformRowVectorIntoFrameVector2d(DenseMatrix64F matrix, FrameVector2d frameVector)
   {
      frameVector.setX(matrix.get(0, 0));
      frameVector.setY(matrix.get(0, 1));
   }

   public static void setDenseMatrixFromTuple3d(DenseMatrix64F matrixToPack, Tuple3d tuple, int startRow, int startColumn)
   {
      matrixToPack.set(startRow + 0, startColumn, tuple.getX());
      matrixToPack.set(startRow + 1, startColumn, tuple.getY());
      matrixToPack.set(startRow + 2, startColumn, tuple.getZ());
   }

   public static int denseMatrixToArrayColumnMajor(DenseMatrix64F src, int srcStartRow, int srcStartCol, int numRows, int numCols, double[] dest,
         int destStartIndex)
   {
      int currentIndex = destStartIndex;
      for (int j = srcStartCol; j < srcStartCol + numCols; j++)
      {
         for (int i = srcStartRow; i < srcStartRow + numRows; i++)
         {
            dest[currentIndex++] = src.get(i, j);
         }
      }

      return currentIndex - destStartIndex;
   }

   public static void extractDiagonal(DenseMatrix64F matrix, double[] diagonal)
   {
      for (int i = 0; i < Math.min(matrix.getNumRows(), matrix.getNumCols()); i++)
      {
         diagonal[i] = matrix.get(i, i);
      }
   }

   public static int denseMatrixToArrayColumnMajor(DenseMatrix64F src, double[] dest)
   {
      return denseMatrixToArrayColumnMajor(src, 0, 0, src.getNumRows(), src.getNumCols(), dest, 0);
   }

   public static void tuple3dToArray(Tuple3d tuple, double[] dest, int startIndex)
   {
      dest[startIndex + 0] = tuple.getX();
      dest[startIndex + 1] = tuple.getY();
      dest[startIndex + 2] = tuple.getZ();
   }

   public static void extractTuple3dFromEJMLVector(Tuple3d tuple3d, DenseMatrix64F ejmlVector, int[] indices)
   {
      tuple3d.setX(ejmlVector.get(indices[0], 0));
      tuple3d.setY(ejmlVector.get(indices[1], 0));
      tuple3d.setZ(ejmlVector.get(indices[2], 0));
   }

   public static void extractTuple3dFromEJMLVector(Tuple3d tuple3d, DenseMatrix64F ejmlVector, int startIndex)
   {
      tuple3d.setX(ejmlVector.get(startIndex + 0, 0));
      tuple3d.setY(ejmlVector.get(startIndex + 1, 0));
      tuple3d.setZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractFrameTupleFromEJMLVector(FrameTuple<?> frameTuple, DenseMatrix64F ejmlVector, ReferenceFrame desiredFrame, int startIndex)
   {
      frameTuple.setToZero(desiredFrame);
      frameTuple.setX(ejmlVector.get(startIndex + 0, 0));
      frameTuple.setY(ejmlVector.get(startIndex + 1, 0));
      frameTuple.setZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractQuat4dFromEJMLVector(Quat4d quaternion, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      double w = matrix.get(index++, 0);
      quaternion.set(x, y, z, w);
   }

   public static void insertTuple3dIntoEJMLVector(Tuple3d tuple3d, DenseMatrix64F ejmlVector, int[] indices)
   {
      ejmlVector.set(indices[0], 0, tuple3d.getX());
      ejmlVector.set(indices[1], 0, tuple3d.getY());
      ejmlVector.set(indices[2], 0, tuple3d.getZ());
   }

   public static void insertTuple3dIntoEJMLVector(Tuple3d tuple3d, DenseMatrix64F ejmlVector, int startIndex)
   {
      ejmlVector.set(startIndex + 0, 0, tuple3d.getX());
      ejmlVector.set(startIndex + 1, 0, tuple3d.getY());
      ejmlVector.set(startIndex + 2, 0, tuple3d.getZ());
   }

   public static void insertFrameTupleIntoEJMLVector(FrameTuple<?> frameTuple, DenseMatrix64F ejmlVector, int startIndex)
   {
      ejmlVector.set(startIndex + 0, 0, frameTuple.getX());
      ejmlVector.set(startIndex + 1, 0, frameTuple.getY());
      ejmlVector.set(startIndex + 2, 0, frameTuple.getZ());
   }

   public static void insertQuat4dIntoEJMLVector(DenseMatrix64F matrix, Quat4d quaternion, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, quaternion.getX());
      matrix.set(index++, 0, quaternion.getY());
      matrix.set(index++, 0, quaternion.getZ());
      matrix.set(index++, 0, quaternion.getW());
   }

   public static <T> int computeIndicesIntoVector(List<T> keys, Map<T, Integer> indices, Map<T, Integer> sizes)
   {
      int stateStartIndex = 0;

      for (T port : keys)
      {
         indices.put(port, stateStartIndex);
         stateStartIndex += sizes.get(port);
      }

      return stateStartIndex;
   }

   public static <RowKeyType, ColumnKeyType> void insertMatrixBlock(DenseMatrix64F bigMatrix, DenseMatrix64F matrixBlock, RowKeyType rowKey,
         Map<? super RowKeyType, Integer> rowStartIndices, ColumnKeyType columnKey, Map<? super ColumnKeyType, Integer> columnStartIndices)
   {
      if (matrixBlock != null)
      {
         int rowStartIndex = rowStartIndices.get(rowKey);
         int columnStartIndex = columnStartIndices.get(columnKey);
         CommonOps.insert(matrixBlock, bigMatrix, rowStartIndex, columnStartIndex);
      }
   }

   public static <RowKeyType> void insertVectorBlock(DenseMatrix64F bigVector, DenseMatrix64F vectorBlock, RowKeyType rowKey,
         Map<? super RowKeyType, Integer> startIndices)
   {
      if (vectorBlock != null)
      {
         int rowStartIndex = startIndices.get(rowKey);
         CommonOps.insert(vectorBlock, bigVector, rowStartIndex, 0);
      }
   }

   public static <RowKeyType> void extractVectorBlock(DenseMatrix64F vectorBlockToPack, DenseMatrix64F bigVector, RowKeyType rowKey,
         Map<? super RowKeyType, Integer> rowStartIndices, Map<? super RowKeyType, Integer> sizes)
   {
      int rowStartIndex = rowStartIndices.get(rowKey);
      int stateSize = sizes.get(rowKey);
      vectorBlockToPack.reshape(stateSize, 1);
      CommonOps.extract(bigVector, rowStartIndex, rowStartIndex + stateSize, 0, 1, vectorBlockToPack, 0, 0);
   }

   public static boolean isIdentity(Matrix3d matrix3d, double tolerance)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            double expectedValue = i == j ? 1.0 : 0.0;
            double element = matrix3d.getElement(i, j);
            if (Math.abs(expectedValue - element) > tolerance)
               return false;
         }
      }

      return true;
   }

   public static String denseMatrixToString(DenseMatrix64F mat)
   {
      ByteArrayOutputStream stream = new ByteArrayOutputStream();
      MatrixIO.print(new PrintStream(stream), mat, "%13.6g");

      return stream.toString();
   }

   public static void multOuter(Matrix3d result, Vector3d vector)
   {
      double x = vector.getX();
      double y = vector.getY();
      double z = vector.getZ();

      result.setElement(0, 0, x * x);
      result.setElement(1, 1, y * y);
      result.setElement(2, 2, z * z);

      double xy = x * y;
      result.setElement(0, 1, xy);
      result.setElement(1, 0, xy);

      double xz = x * z;
      result.setElement(0, 2, xz);
      result.setElement(2, 0, xz);

      double yz = y * z;
      result.setElement(1, 2, yz);
      result.setElement(2, 1, yz);
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be 3x3.
    * @param matrix
    * @param point
    */
   public static void mult(DenseMatrix64F matrix, Point3d point)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = point.x;
      double y = point.y;
      double z = point.z;

      point.x = matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z;
      point.y = matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z;
      point.z = matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z;
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be 3x3.
    * @param matrix
    * @param point
    */
   public static void mult(DenseMatrix64F matrix, Point3f point)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = point.x;
      double y = point.y;
      double z = point.z;

      point.x = (float) (matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z);
      point.y = (float) (matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z);
      point.z = (float) (matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z);
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be 3x3.
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector3d vector)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = vector.x;
      double y = vector.y;
      double z = vector.z;

      vector.x = matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z;
      vector.y = matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z;
      vector.z = matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z;
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be 3x3.
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector3f vector)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      float x = vector.x;
      float y = vector.y;
      float z = vector.z;

      vector.x = (float) (matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z);
      vector.y = (float) (matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z);
      vector.z = (float) (matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z);
   }

   /**
    * Multiply a 4x4 matrix by a 4x1 vector. Since result is stored in vector, the matrix must be 4x4.
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector4d vector)
   {
      if (matrix.numCols != 4 || matrix.numRows != 4)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = vector.x;
      double y = vector.y;
      double z = vector.z;
      double w = vector.w;

      vector.x = matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z + matrix.get(0, 3) * w;
      vector.y = matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z + matrix.get(1, 3) * w;
      vector.z = matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z + matrix.get(2, 3) * w;
      vector.w = matrix.get(3, 0) * x + matrix.get(3, 1) * y + matrix.get(3, 2) * z + matrix.get(3, 3) * w;
   }

   /**
    * Multiply a 4x4 matrix by a 4x1 vector. Since result is stored in vector, the matrix must be 4x4.
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector4f vector)
   {
      if (matrix.numCols != 4 || matrix.numRows != 4)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      float x = vector.x;
      float y = vector.y;
      float z = vector.z;
      float w = vector.w;

      vector.x = (float) (matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z + matrix.get(0, 3) * w);
      vector.y = (float) (matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z + matrix.get(1, 3) * w);
      vector.z = (float) (matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z + matrix.get(2, 3) * w);
      vector.w = (float) (matrix.get(3, 0) * x + matrix.get(3, 1) * y + matrix.get(3, 2) * z + matrix.get(3, 3) * w);
   }

   public static void removeRow(DenseMatrix64F matrixToRemoveRowTo, int indexOfRowToRemove)
   {
      for (int columnIndex = 0; columnIndex < matrixToRemoveRowTo.getNumCols(); columnIndex++)
      {
         for (int currentRowIndex = indexOfRowToRemove; currentRowIndex < matrixToRemoveRowTo.getNumRows() - 1; currentRowIndex++)
         {
            int nextRowIndex = currentRowIndex + 1;
            double valueOfNextRow = matrixToRemoveRowTo.get(nextRowIndex, columnIndex);
            double valueOfCurrentRow = matrixToRemoveRowTo.get(currentRowIndex, columnIndex);

            matrixToRemoveRowTo.set(nextRowIndex, columnIndex, valueOfCurrentRow);
            matrixToRemoveRowTo.set(currentRowIndex, columnIndex, valueOfNextRow);
         }
      }

      matrixToRemoveRowTo.reshape(matrixToRemoveRowTo.getNumRows() - 1, matrixToRemoveRowTo.getNumCols(), true);
   }

   public static void removeZeroRows(DenseMatrix64F matrixToModify, double epsilon)
   {
      for (int rowIndex = matrixToModify.getNumRows() - 1; rowIndex >= 0; rowIndex--)
      {
         double sumOfRowElements = 0.0;

         for (int columnIndex = 0; columnIndex < matrixToModify.getNumCols(); columnIndex++)
         {
            sumOfRowElements += Math.abs(matrixToModify.get(rowIndex, columnIndex));
         }

         boolean isZeroRow = MathTools.epsilonEquals(sumOfRowElements, 0.0, epsilon);
         if (isZeroRow)
            removeRow(matrixToModify, rowIndex);
      }
   }

   public static void printJavaForConstruction(String name, DenseMatrix64F matrix)
   {
      StringBuffer stringBuffer = new StringBuffer();
      printJavaForConstruction(stringBuffer, name, matrix);
      System.out.println(stringBuffer);
   }

   public static void printJavaForConstruction(StringBuffer stringBuffer, String name, DenseMatrix64F matrix)
   {
      int numRows = matrix.getNumRows();
      int numColumns = matrix.getNumCols();

      stringBuffer.append("      double[][] " + name + "Data = new double[][]{");

      for (int i = 0; i < numRows; i++)
      {
         stringBuffer.append("\n            {");

         for (int j = 0; j < numColumns; j++)
         {
            stringBuffer.append(matrix.get(i, j));
            if (j < numColumns - 1)
               stringBuffer.append(", ");

         }
         stringBuffer.append("}");
         if (i < numRows - 1)
            stringBuffer.append(", ");
      }
      stringBuffer.append("};\n");

      stringBuffer.append("      DenseMatrix64F " + name + " = new DenseMatrix64F(" + name + "Data);");
   }

   public static void checkMatrixDimensions(DenseMatrix64F matrixToCheck, int expectedRows, int expectedColumns)
   {
      if ((matrixToCheck.getNumRows() != expectedRows) || (matrixToCheck.getNumCols() != expectedColumns))
      {
         String message = "Matrix dimensions are (" + matrixToCheck.getNumRows() + ", " + matrixToCheck.getNumCols() + "), expected ("
                          + expectedRows + "," + expectedColumns + ")";
   
         throw new RuntimeException(message);
      }      
   }
}
