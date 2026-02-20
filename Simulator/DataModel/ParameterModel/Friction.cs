using MathNet.Numerics.LinearAlgebra;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Friction
    {
        // Friction coeficcients
        public readonly double mu_s_factor = 0.3;       // [-] Static friction coefficient
        public readonly double mu_k_factor = 0.2;       // [-] Kinetic friction coefficient

        // Coulomb friction parameters
        public readonly double v_c = 0.05;              // [m/s] Velocity threshold for static to kinetic transition
        public double stribeck = 5;            // [s/rad] Parameter controlling smoothness of transition from static to kinetic friction (Stribeck model)

        public Vector<double> StaticFrictionCoefficient;                     // [-] Static friction coefficient
        public Vector<double> KinematicFrictionCoefficient;                     // [-] Kinetic friction coefficient

        public Friction(LumpedCells lc, double mu_s_factor, double mu_k_factor, double stribeck)
        {
            this.stribeck = stribeck;
            this.mu_s_factor = mu_s_factor;
            this.mu_k_factor = mu_k_factor;
            Vector<double> vectorOfOnes = Vector<double>.Build.Dense(lc.NumberOfLumpedElements, 1);
            StaticFrictionCoefficient = mu_s_factor * vectorOfOnes;
            KinematicFrictionCoefficient = mu_k_factor * vectorOfOnes;
        }
    }
}
