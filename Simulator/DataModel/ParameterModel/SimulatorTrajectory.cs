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
        public Vector<double> InterpolatedVerticalDepthAtNode;        // [m] TVD interpolated
        public Vector<double> InterpolatedThetaAtNode;      // [deg] Theta interpolated
        public Vector<double> InterpolatedPhiAtNode;        // [deg] Phi  interpolated
        public Vector<double> DiffThetaInterpolatedAtNode;
        public Vector<double> DiffPhiInterpolatedAtNode;
        public Vector<double> DiffDiffThetaInterpolatedAtNode;
        public Vector<double> DiffDiffPhiInterpolatedAtNode;
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
            

            List<SurveyStation> surveyPoints = trajectory.SurveyStationList.ToList();
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
            InterpolatedVerticalDepthAtNode = LinearInterpolate(MeasuredDepthProfile, VerticalDepthProfile, nodeDepthVector);
            InterpolatedThetaAtNode = Math.PI / 180 * LinearInterpolate(MeasuredDepthProfile, InclinationProfile, nodeDepthVector);
            InterpolatedPhiAtNode = Math.PI / 180 * LinearInterpolate(MeasuredDepthProfile, AzimuthProfile, nodeDepthVector);

            int n = InterpolatedThetaAtNode.Count();
            DiffThetaInterpolatedAtNode = ComputeDerivative(InterpolatedThetaAtNode, drillString.ElementLength);
            DiffPhiInterpolatedAtNode = ComputeDerivative(InterpolatedPhiAtNode, drillString.ElementLength);
            DiffDiffThetaInterpolatedAtNode = ComputeSecondDerivative(InterpolatedThetaAtNode, drillString.ElementLength);
            DiffDiffPhiInterpolatedAtNode = ComputeSecondDerivative(InterpolatedPhiAtNode, drillString.ElementLength);

            Curvature = Vector<double>.Build.Dense(DiffThetaInterpolatedAtNode.Count);
            for (int i = 0; i < DiffThetaInterpolatedAtNode.Count(); i++)
            {
                Curvature[i] = Math.Sqrt(
                    Math.Pow(DiffThetaInterpolatedAtNode[i], 2) +
                    Math.Pow(DiffPhiInterpolatedAtNode[i], 2) * Math.Pow(Math.Sin(InterpolatedThetaAtNode[i]), 2)
                );  
            }

            // Compute torsion using element-wise operations
            Torsion = Vector<double>.Build.Dense(DiffThetaInterpolatedAtNode.Count);

            for (int i = 0; i < Torsion.Count; i++)
            {
                double num1 = DiffThetaInterpolatedAtNode[i] * DiffDiffPhiInterpolatedAtNode[i] - DiffDiffThetaInterpolatedAtNode[i] * DiffPhiInterpolatedAtNode[i];
                double denom = Math.Pow(Curvature[i], 2);
                double term1 = num1 / denom * Math.Sin(InterpolatedThetaAtNode[i]);
                double term2 = DiffPhiInterpolatedAtNode[i] * (1 + Math.Pow(DiffThetaInterpolatedAtNode[i], 2) / denom) * Math.Cos(InterpolatedThetaAtNode[i]);

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

            tx = InterpolatedThetaAtNode.PointwiseSin().PointwiseMultiply(InterpolatedPhiAtNode.PointwiseCos());
            ty = InterpolatedThetaAtNode.PointwiseSin().PointwiseMultiply(InterpolatedPhiAtNode.PointwiseSin());
            tz = InterpolatedThetaAtNode.PointwiseCos();

            nx = Vector<double>.Build.Dense(n);
            ny = Vector<double>.Build.Dense(n);
            nz = Vector<double>.Build.Dense(n);
            bz = Vector<double>.Build.Dense(n);

            for (int i = 0; i < n; i++)
            {
                if (Curvature[i] != 0)
                {
                    nx[i] = (Math.Cos(InterpolatedThetaAtNode[i]) * Math.Cos(InterpolatedPhiAtNode[i]) * DiffThetaInterpolatedAtNode[i] -
                             Math.Sin(InterpolatedThetaAtNode[i]) * Math.Sin(InterpolatedPhiAtNode[i]) * DiffPhiInterpolatedAtNode[i]) / Curvature[i];

                    ny[i] = (Math.Cos(InterpolatedThetaAtNode[i]) * Math.Sin(InterpolatedPhiAtNode[i]) * DiffThetaInterpolatedAtNode[i] +
                             Math.Sin(InterpolatedThetaAtNode[i]) * Math.Cos(InterpolatedPhiAtNode[i]) * DiffPhiInterpolatedAtNode[i]) / Curvature[i];

                    nz[i] = -DiffThetaInterpolatedAtNode[i] * Math.Sin(InterpolatedThetaAtNode[i]) / Curvature[i];

                    bz[i] = DiffPhiInterpolatedAtNode[i] * Math.Pow(Math.Sin(InterpolatedThetaAtNode[i]), 2) / Curvature[i];
                }
            }

            // Handling idx_zerocurvature (curvature = 0) cases
            for (int i = 0; i < n; i++)
            {
                if (Curvature[i] == 0)
                {
                    nx[i] = Math.Cos(InterpolatedThetaAtNode[i]) * Math.Cos(InterpolatedPhiAtNode[i]);
                    ny[i] = Math.Cos(InterpolatedThetaAtNode[i]) * Math.Sin(InterpolatedPhiAtNode[i]);
                    nz[i] = -Math.Sin(InterpolatedThetaAtNode[i]);
                    bz[i] = 0;
                }
            }

            hx = InterpolatedThetaAtNode.PointwiseCos().PointwiseMultiply(InterpolatedPhiAtNode.PointwiseCos());
            hy = InterpolatedThetaAtNode.PointwiseCos().PointwiseMultiply(InterpolatedPhiAtNode.PointwiseSin());
            hz = -InterpolatedThetaAtNode.PointwiseSin();

            vx = -InterpolatedPhiAtNode.PointwiseSin();
            vy = InterpolatedPhiAtNode.PointwiseCos();
            vz = Vector<double>.Build.Dense(n);
        }
    }
}
