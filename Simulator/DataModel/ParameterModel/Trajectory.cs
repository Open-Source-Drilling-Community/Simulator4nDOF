using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Trajectory
    {
        // Trajectory (from trajectory file processing)
        private Vector<double> MD_prof;      // [m] MD orig
        private Vector<double> inc_prof;     // [degrees] Inclination orig
        private Vector<double> azi_prof;     // [degrees] Azimuth orig
        private Vector<double> TVD_prof;     // [m] TVD orig
        public Vector<double> TVDVec;        // [m] TVD interpolated
        public Vector<double> thetaVec;      // [deg] Theta interpolated
        public Vector<double> phiVec;        // [deg] Phi  interpolated
        public Vector<double> thetaVec_dot;
        public Vector<double> phiVec_dot;
        public Vector<double> thetaVec_ddot;
        public Vector<double> phiVec_ddot;
        public Vector<double> curvature;
        public Vector<double> torsion;
        public Vector<double> curvature_dot;
        public Vector<double> curvature_ddot;
        public Vector<double> torsion_dot;
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

        public Trajectory(LumpedCells lc, string trajectoryFilename)
        {
            double[,] traj = ReadTextFile(trajectoryFilename);

            int rows = traj.GetLength(0);
            MD_prof = GetColumn(traj, 0);
            inc_prof = GetColumn(traj, 1);
            azi_prof = GetColumn(traj, 2);

            Vector<double> azi_prof_rad = Math.PI / 180 * azi_prof;
            Vector<double> unwrapped_rad = Unwrap(azi_prof_rad, 1.9 * Math.PI);

            azi_prof = 180 / Math.PI * unwrapped_rad;
            TVD_prof = GetColumn(traj, 3);

            UpdateTrajectory(lc);
        }

        public void UpdateTrajectory(in LumpedCells l)
        {
            Vector<double> vectorWithoutFirstElement = l.xL.SubVector(1, l.xL.Count() - 1);
            TVDVec = LinearInterpolate(MD_prof, TVD_prof, vectorWithoutFirstElement);
            thetaVec = Math.PI / 180 * LinearInterpolate(MD_prof, inc_prof, vectorWithoutFirstElement);
            phiVec = Math.PI / 180 * LinearInterpolate(MD_prof, azi_prof, vectorWithoutFirstElement);

            int n = thetaVec.Count();
            thetaVec_dot = ComputeDerivative(thetaVec, l.dxL);
            phiVec_dot = ComputeDerivative(phiVec, l.dxL);
            thetaVec_ddot = ComputeSecondDerivative(thetaVec, l.dxL);
            phiVec_ddot = ComputeSecondDerivative(phiVec, l.dxL);

            curvature = Vector<double>.Build.Dense(thetaVec_dot.Count);
            for (int i = 0; i < thetaVec_dot.Count(); i++)
            {
                curvature[i] = Math.Sqrt(
                    Math.Pow(thetaVec_dot[i], 2) +
                    Math.Pow(phiVec_dot[i], 2) * Math.Pow(Math.Sin(thetaVec[i]), 2)
                );  
            }

            // Compute torsion using element-wise operations
            torsion = Vector<double>.Build.Dense(thetaVec_dot.Count);

            for (int i = 0; i < torsion.Count; i++)
            {
                double num1 = thetaVec_dot[i] * phiVec_ddot[i] - thetaVec_ddot[i] * phiVec_dot[i];
                double denom = Math.Pow(curvature[i], 2);
                double term1 = num1 / denom * Math.Sin(thetaVec[i]);
                double term2 = phiVec_dot[i] * (1 + Math.Pow(thetaVec_dot[i], 2) / denom) * Math.Cos(thetaVec[i]);

                if (denom > 0)
                    torsion[i] = term1 + term2;
                else
                    torsion[i] = 0;
            }

            curvature_dot = ComputeDerivative(curvature, l.dxL);
            curvature_ddot = ComputeSecondDerivative(curvature, l.dxL);
            torsion_dot = ComputeDerivative(torsion, l.dxL);

            int[] idxZeroCurvature = curvature.Select((value, index) => value == 0 ? index : -1).Where(x => x != -1).ToArray();
            foreach (int index in idxZeroCurvature)
                torsion[index] = 0;

            tx = thetaVec.PointwiseSin().PointwiseMultiply(phiVec.PointwiseCos());
            ty = thetaVec.PointwiseSin().PointwiseMultiply(phiVec.PointwiseSin());
            tz = thetaVec.PointwiseCos();

            nx = Vector<double>.Build.Dense(n);
            ny = Vector<double>.Build.Dense(n);
            nz = Vector<double>.Build.Dense(n);
            bz = Vector<double>.Build.Dense(n);

            for (int i = 0; i < n; i++)
            {
                if (curvature[i] != 0)
                {
                    nx[i] = (Math.Cos(thetaVec[i]) * Math.Cos(phiVec[i]) * thetaVec_dot[i] -
                             Math.Sin(thetaVec[i]) * Math.Sin(phiVec[i]) * phiVec_dot[i]) / curvature[i];

                    ny[i] = (Math.Cos(thetaVec[i]) * Math.Sin(phiVec[i]) * thetaVec_dot[i] +
                             Math.Sin(thetaVec[i]) * Math.Cos(phiVec[i]) * phiVec_dot[i]) / curvature[i];

                    nz[i] = -thetaVec_dot[i] * Math.Sin(thetaVec[i]) / curvature[i];

                    bz[i] = phiVec_dot[i] * Math.Pow(Math.Sin(thetaVec[i]), 2) / curvature[i];
                }
            }

            // Handling idx_zerocurvature (curvature = 0) cases
            for (int i = 0; i < n; i++)
            {
                if (curvature[i] == 0)
                {
                    nx[i] = Math.Cos(thetaVec[i]) * Math.Cos(phiVec[i]);
                    ny[i] = Math.Cos(thetaVec[i]) * Math.Sin(phiVec[i]);
                    nz[i] = -Math.Sin(thetaVec[i]);
                    bz[i] = 0;
                }
            }

            hx = thetaVec.PointwiseCos().PointwiseMultiply(phiVec.PointwiseCos());
            hy = thetaVec.PointwiseCos().PointwiseMultiply(phiVec.PointwiseSin());
            hz = -thetaVec.PointwiseSin();

            vx = -phiVec.PointwiseSin();
            vy = phiVec.PointwiseCos();
            vz = Vector<double>.Build.Dense(n);
        }
    }
}
