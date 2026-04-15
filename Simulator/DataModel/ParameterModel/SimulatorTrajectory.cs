using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulatorTrajectory
    {
        // Trajectory (from trajectory file processing)
        private Vector<double> MeasuredDepthProfile;      // [m] MD orig
        private Vector<double> InclinationProfile;     // [degrees] Inclination orig
        private Vector<double> AzimuthProfile;     // [degrees] Azimuth orig
        private Vector<double> VerticalDepthProfile;     // [m] TVD orig
        public Vector<double> InterpolatedVerticalDepth;        // [m] TVD interpolated
        public Vector<double> InterpolatedTheta;      // [deg] Theta interpolated
        public Vector<double> InterpolatedPhi;        // [deg] Phi  interpolated
        public Vector<double> DiffThetaInterpolated;
        public Vector<double> DiffPhiInterpolated;
        public Vector<double> DiffDiffThetaInterpolated;
        public Vector<double> DiffDiffPhiInterpolated;
        public Vector<double> Curvature;
        public Vector<double> Torsion;
        public Vector<double> CurvatureDerivative;
        public Vector<double> CurvatureSecondDerivative;
        public Vector<double> TorsionDerivative;
        public Vector<double> tx;
        public Vector<double> ty;
        public Vector<double> tz;
        public Vector<double> nx;
        public Vector<double> ny;
        public Vector<double> nz;
        public Vector<double> bz;
        public Vector<double> hx;
        public Vector<double> hy;
        public Vector<double> hz;
        private Vector<double> vx;
        private Vector<double> vy;
        private Vector<double> vz;

        public SimulatorTrajectory(in SimulatorDrillString drillString, in Trajectory trajectory)
        {
            

            List<SurveyPoint> surveyPoints = trajectory.InterpolatedTrajectory.ToList();
            int rows = surveyPoints.Count;
            MeasuredDepthProfile = Vector<double>.Build.Dense(rows);
            InclinationProfile = Vector<double>.Build.Dense(rows);
            AzimuthProfile = Vector<double>.Build.Dense(rows);
            VerticalDepthProfile = Vector<double>.Build.Dense(rows);            
            for (int i = 0; i < rows; i++)
            {
                MeasuredDepthProfile[i] = surveyPoints[i].MD ?? 0;
                InclinationProfile[i] = surveyPoints[i].Inclination ?? 0;
                AzimuthProfile[i] = surveyPoints[i].Azimuth ?? 0;
                VerticalDepthProfile[i] = surveyPoints[i].TVD ?? 0;
            }


            Vector<double> azimuthProfileInRadians = Math.PI / 180 * AzimuthProfile;
            Vector<double> unwrappedRadians = Unwrap(azimuthProfileInRadians, 1.9 * Math.PI);

            AzimuthProfile = 180 / Math.PI * unwrappedRadians;

            UpdateTrajectory(in drillString);
        }
        public void UpdateTrajectory(in SimulatorDrillString drillString)
        {
            Vector<double> nodeDepthVector = Vector<double>.Build.Dense(drillString.RelativeNodeDepth.Count); //CumulativeElementLength.SubVector(1, lumpedCells.CumulativeElementLength.Count() - 1);
            //vectorWithoutFirstElement[0] = drillString.RelativeNodeDepth[1];
            for (int i = 0; i < drillString.RelativeNodeDepth.Count; i++)
            {
                nodeDepthVector[i] = drillString.RelativeNodeDepth[i];
            }
            InterpolatedVerticalDepth = LinearInterpolate(MeasuredDepthProfile, VerticalDepthProfile, nodeDepthVector);
            InterpolatedTheta = Math.PI / 180 * LinearInterpolate(MeasuredDepthProfile, InclinationProfile, nodeDepthVector);
            InterpolatedPhi = Math.PI / 180 * LinearInterpolate(MeasuredDepthProfile, AzimuthProfile, nodeDepthVector);

            int n = InterpolatedTheta.Count();
            DiffThetaInterpolated = ComputeDerivative(InterpolatedTheta, drillString.ElementLength);
            DiffPhiInterpolated = ComputeDerivative(InterpolatedPhi, drillString.ElementLength);
            DiffDiffThetaInterpolated = ComputeSecondDerivative(InterpolatedTheta, drillString.ElementLength);
            DiffDiffPhiInterpolated = ComputeSecondDerivative(InterpolatedPhi, drillString.ElementLength);

            Curvature = Vector<double>.Build.Dense(DiffThetaInterpolated.Count);
            for (int i = 0; i < DiffThetaInterpolated.Count(); i++)
            {
                Curvature[i] = Math.Sqrt(
                    Math.Pow(DiffThetaInterpolated[i], 2) +
                    Math.Pow(DiffPhiInterpolated[i], 2) * Math.Pow(Math.Sin(InterpolatedTheta[i]), 2)
                );  
            }

            // Compute torsion using element-wise operations
            Torsion = Vector<double>.Build.Dense(DiffThetaInterpolated.Count);

            for (int i = 0; i < Torsion.Count; i++)
            {
                double num1 = DiffThetaInterpolated[i] * DiffDiffPhiInterpolated[i] - DiffDiffThetaInterpolated[i] * DiffPhiInterpolated[i];
                double denom = Math.Pow(Curvature[i], 2);
                double term1 = num1 / denom * Math.Sin(InterpolatedTheta[i]);
                double term2 = DiffPhiInterpolated[i] * (1 + Math.Pow(DiffThetaInterpolated[i], 2) / denom) * Math.Cos(InterpolatedTheta[i]);

                if (denom > 0)
                    Torsion[i] = term1 + term2;
                else
                    Torsion[i] = 0;
            }

            CurvatureDerivative = ComputeDerivative(Curvature, drillString.ElementLength);
            CurvatureSecondDerivative = ComputeSecondDerivative(Curvature, drillString.ElementLength);
            TorsionDerivative = ComputeDerivative(Torsion, drillString.ElementLength);

            int[] idxZeroCurvature = Curvature.Select((value, index) => value == 0 ? index : -1).Where(x => x != -1).ToArray();
            foreach (int index in idxZeroCurvature)
                Torsion[index] = 0;

            tx = InterpolatedTheta.PointwiseSin().PointwiseMultiply(InterpolatedPhi.PointwiseCos());
            ty = InterpolatedTheta.PointwiseSin().PointwiseMultiply(InterpolatedPhi.PointwiseSin());
            tz = InterpolatedTheta.PointwiseCos();

            nx = Vector<double>.Build.Dense(n);
            ny = Vector<double>.Build.Dense(n);
            nz = Vector<double>.Build.Dense(n);
            bz = Vector<double>.Build.Dense(n);

            for (int i = 0; i < n; i++)
            {
                if (Curvature[i] != 0)
                {
                    nx[i] = (Math.Cos(InterpolatedTheta[i]) * Math.Cos(InterpolatedPhi[i]) * DiffThetaInterpolated[i] -
                             Math.Sin(InterpolatedTheta[i]) * Math.Sin(InterpolatedPhi[i]) * DiffPhiInterpolated[i]) / Curvature[i];

                    ny[i] = (Math.Cos(InterpolatedTheta[i]) * Math.Sin(InterpolatedPhi[i]) * DiffThetaInterpolated[i] +
                             Math.Sin(InterpolatedTheta[i]) * Math.Cos(InterpolatedPhi[i]) * DiffPhiInterpolated[i]) / Curvature[i];

                    nz[i] = -DiffThetaInterpolated[i] * Math.Sin(InterpolatedTheta[i]) / Curvature[i];

                    bz[i] = DiffPhiInterpolated[i] * Math.Pow(Math.Sin(InterpolatedTheta[i]), 2) / Curvature[i];
                }
            }

            // Handling idx_zerocurvature (curvature = 0) cases
            for (int i = 0; i < n; i++)
            {
                if (Curvature[i] == 0)
                {
                    nx[i] = Math.Cos(InterpolatedTheta[i]) * Math.Cos(InterpolatedPhi[i]);
                    ny[i] = Math.Cos(InterpolatedTheta[i]) * Math.Sin(InterpolatedPhi[i]);
                    nz[i] = -Math.Sin(InterpolatedTheta[i]);
                    bz[i] = 0;
                }
            }

            hx = InterpolatedTheta.PointwiseCos().PointwiseMultiply(InterpolatedPhi.PointwiseCos());
            hy = InterpolatedTheta.PointwiseCos().PointwiseMultiply(InterpolatedPhi.PointwiseSin());
            hz = -InterpolatedTheta.PointwiseSin();

            vx = -InterpolatedPhi.PointwiseSin();
            vy = InterpolatedPhi.PointwiseCos();
            vz = Vector<double>.Build.Dense(n);
        }
    }
}
