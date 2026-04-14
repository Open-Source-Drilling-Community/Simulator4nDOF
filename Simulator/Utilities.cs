using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using System.Globalization;
using System.Runtime.InteropServices;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public static class Utilities
    {
        public static Vector<double> Reverse(Vector<double> vector)
        {
            double[] array = vector.ToArray();
            Array.Reverse(array);
            Vector<double> reversedVector = Vector<double>.Build.DenseOfArray(array);
            return reversedVector;
        }

        public static Vector<double> Square(Vector<double> vector)
        {
            return vector.PointwiseMultiply(vector);
        }        
        public static Vector<double> ToVector(List<double> list)
        {
            return ToVector(list.ToArray());
        }

        public static Vector<double> ToVector(double value)
        {
            return ToVector(new double[] { value });
        }
        public static Vector<double> ToVector(double[] array)
        {
            return Vector<double>.Build.DenseOfArray(array);
        }

        public static Vector<double> ExtendVectorStart(double scalar, double[] array)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { scalar }.Concat(array).ToArray());
        }


        public static Vector<double> ExtendVectorStart(double scalar, Vector<double> array)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { scalar }.Concat(array).ToArray());
        }

        public static double[] MaxArray(double[] array, double minValue)
        {
            return array.Select(x => Math.Max(x, minValue)).ToArray();
        }

        public static Matrix<double> DiffRows(Matrix<double> X)
        {
            var diffMatrix = Matrix<double>.Build.Dense(X.RowCount - 1, X.ColumnCount);

            // Use the built-in operation for element-wise subtraction
            diffMatrix = X.SubMatrix(1, X.RowCount - 1, 0, X.ColumnCount)
                          - X.SubMatrix(0, X.RowCount - 1, 0, X.ColumnCount);

            return diffMatrix;
        }

        public static Vector<double> Diff(Vector<double> y)
        {
            return ToVector(Diff(y.ToArray()));
        }

        public static double[] Diff(double[] y)
        {
            double[] diff = new double[y.Length - 1];
            for (int i = 0; i < diff.Length; i++)
            {
                diff[i] = y[i + 1] - y[i];
            }
            return diff;
        }

        public static double[] Linspace(double start, double end, int numPoints)
        {
            double[] result = new double[numPoints];
            double step = (end - start) / (numPoints - 1);
            for (int i = 0; i < numPoints; i++)
            {
                result[i] = start + i * step;
            }
            return result;
        }

        public static double[,] ReadTextFile(string filename)
        {
            string[] lines = File.ReadAllLines(filename);
            int rowCount = lines.Length;
            int colCount = lines[0].Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries).Length;

            double[,] data = new double[rowCount, colCount];

            for (int i = 0; i < rowCount; i++)
            {
                double[] values = lines[i].Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries)
                          .Select(s => double.Parse(s, CultureInfo.InvariantCulture))
                          .ToArray();
                for (int j = 0; j < colCount; j++)
                {
                    data[i, j] = values[j];
                }
            }
            return data;
        }

        public static List<double> GetColumnList(double[,] matrix, int columnIndex)
        {
            return Enumerable.Range(0, matrix.GetLength(0))
                             .Select(i => matrix[i, columnIndex])
                             .ToList();
        }

        public static Vector<double> GetColumn(double[,] matrix, int columnIndex)
        {
            return ToVector(Enumerable.Range(0, matrix.GetLength(0))
                             .Select(i => matrix[i, columnIndex])
                             .ToArray());
        }

        public static Vector<double> Unwrap(Vector<double> phase, double tolerance)
        {
            return ToVector(Unwrap(phase.ToArray(), tolerance));
        }

        public static double[] Unwrap(double[] phase, double tolerance)
        {
            int n = phase.Length;
            double[] unwrapped = new double[n];
            unwrapped[0] = phase[0];

            for (int i = 1; i < n; i++)
            {
                double diff = phase[i] - phase[i - 1];
                if (diff > tolerance)
                {
                    // Subtract 2*pi to unwrap
                    unwrapped[i] = unwrapped[i - 1] + diff - 2 * Math.PI;
                }
                else if (diff < -tolerance)
                {
                    // Add 2*pi to unwrap
                    unwrapped[i] = unwrapped[i - 1] + diff + 2 * Math.PI;
                }
                else
                {
                    // No unwrapping needed
                    unwrapped[i] = unwrapped[i - 1] + diff;
                }
            }

            return unwrapped;
        }

        public static Vector<double> LinearInterpolate2(Vector<double> x, Vector<double> y, Vector<double> xTarget)
        {
            var X = x.ToArray();
            var Y = y.ToArray();
        
            // Ensure X is sorted
            Array.Sort(X, Y);
        
            var yInterpolated = xTarget.Map(xNew => LinearInterpolate2(X, Y,xNew));
            return yInterpolated;
        }

        public static Vector<double> LinearInterpolate(Vector<double> x, Vector<double> y, Vector<double> xTarget)
        {
            var X = x.ToArray();
            var Y = y.ToArray();
       
            // Ensure X is sorted
            Array.Sort(X, Y);
       
            var interpolator = LinearSpline.InterpolateSorted(X, Y);
            var yInterpolated = xTarget.Map(xNew => interpolator.Interpolate(xNew));
            return yInterpolated;
        }


        // Manual fast linear interpolation
        public static double LinearInterpolate2(double[] x, double[] y, double xTarget)
        {
            int n = x.Length;
       
            // Extrapolation: Before first point
            if (xTarget <= x[0])
                return y[0] + (y[1] - y[0]) / (x[1] - x[0]) * (xTarget - x[0]);
       
            // Extrapolation: After last point
            if (xTarget >= x[n - 1])
                return y[n - 1] + (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]) * (xTarget - x[n - 1]);
       
            // Binary search for interval
            int low = 0, high = n - 1;
            while (high - low > 1)
            {
                int mid = (low + high) / 2;
                if (x[mid] > xTarget)
                    high = mid;
                else
                    low = mid;
            }
       
            // Linear interpolation formula
            return y[low] + (y[high] - y[low]) / (x[high] - x[low]) * (xTarget - x[low]);
        }

        public static Vector<double> ComputeDerivative(Vector<double> values, double dx)
        {
            return ToVector(ComputeDerivative(values.ToArray(),dx));
        }
        public static Vector<double> ComputeDerivative(Vector<double> values, List<double> dx)
        {
            int n = values.Count;
            Vector<double> derivative = Vector<double>.Build.Dense(n);
            for (int i = 1; i < n - 1; i++)
            {
                derivative[i] = (values[i + 1] - values[i - 1]) / (2 * dx[i + 1]);
            }
            derivative[0] = (values[1] - values[0]) / dx[0];
            derivative[n - 1] = (values[n - 1] - values[n - 2]) / dx[n];
            return derivative;
        }

        public static double[] ComputeDerivative(double[] values, double dx)
        {
            int n = values.Length;
            double[] derivative = new double[n];
            for (int i = 1; i < n - 1; i++)
                derivative[i] = (values[i + 1] - values[i - 1]) / (2 * dx);
            derivative[0] = (values[1] - values[0]) / dx;
            derivative[n - 1] = (values[n - 1] - values[n - 2]) / dx;
            return derivative;
        }
       


        public static Vector<double> ComputeSecondDerivative(Vector<double> values, double dx)
        {
            return ToVector(ComputeSecondDerivative(values.ToArray(), dx));
        }
        public static Vector<double> ComputeSecondDerivative(Vector<double> values, List<double> dx)
        {
            int n = values.Count;
            Vector<double> secondDerivative = Vector<double>.Build.Dense(n);
            for (int i = 1; i < n - 1; i++)
            {
                secondDerivative[i] = (values[i + 1] - 2 * values[i] + values[i - 1]) / (dx[i + 1] * dx[i + 1]);                
            }
            secondDerivative[0] = secondDerivative[1];
            secondDerivative[n - 1] = (-values[n - 1] + values[n - 2]) / (dx[n] * dx[n]);
            return secondDerivative;
        
        }

        public static double[] ComputeSecondDerivative(double[] values, double dx)
        {
            int n = values.Length;
            double[] secondDerivative = new double[n];
            for (int i = 1; i < n - 1; i++)
                secondDerivative[i] = (values[i + 1] - 2 * values[i] + values[i - 1]) / (dx * dx);
            secondDerivative[0] = secondDerivative[1];
            secondDerivative[n - 1] = (-values[n - 1] + values[n - 2]) / (dx * dx);
            return secondDerivative;
        }

        public static double[] CummulativeTrapezoidal(double[] x, double[] y)
        {
            double[] result = new double[y.Length];
            result[0] = 0;
            for (int i = 1; i < y.Length; i++)
            {
                result[i] = result[i - 1] + 0.5 * (y[i] + y[i - 1]) * (x[i] - x[i - 1]);
            }
            return result;
        }
        public static double[] CummulativeIntegration(double[] x, double[] y)
        {
            double[] result = new double[y.Length];
            result[0] = 0;
            for (int i = 1; i < y.Length; i++)
            {
                result[i] = result[i - 1] + y[i] * (x[i] - x[i - 1]);
            }
            return result;
        }

            public static Vector<double> CumulativeTrapezoidal(Vector<double> x, Vector<double> y)
        {
            return ToVector(CummulativeTrapezoidal( x.ToArray(), y.ToArray()));
        }
        public static double DotProduct(double[] vec1, double[] vec2)
        {
            double dotProd = 0;
            for (int i = 0; i < vec1.Length; i++)
            {
                dotProd += vec1[i]*vec2[i];
            }
            return dotProd;            
        }
    }
}
