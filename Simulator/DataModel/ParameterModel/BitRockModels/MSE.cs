using MathNet.Numerics.LinearAlgebra;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class MSE : BitRock
    {
        /// <summary>
        /// [Pa] Confined compressive strength of rock (MSE model)
        /// </summary>
        public readonly double CCS = 10e6;
        /// <summary>
        /// [-] Exponent in rate - dependent torque on bit term (used in both Detournay and MSE model)
        /// </summary>
        public readonly double BitRockFrictionExponent = 0.1;
        /// <summary>
        /// [-] Efficiency factor (MSE model)
        /// </summary>
        public readonly double BitEfficiencyFactor = 0.35;      
        public override double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, Matrix<double> aa, SimulationParameters simulationParameters)
        {
            double tb = 0.0;
            double wb = 0.0;
            if (state.onBottom)
            {
                // Update the last element of l
                int lastIndex = state.DepthOfCut.Count - 1;
                state.DepthOfCut[lastIndex] = (1 - simulationParameters.BitRock.alpha_ROP) * state.DepthOfCut[lastIndex] + simulationParameters.BitRock.alpha_ROP * 2 * Math.PI * state.BitVelocity / mudoRotorAngularVelocity * (mudoRotorAngularVelocity > 0.5 ? 1 : 0);
                state.DepthOfCut[lastIndex] = Math.Max(state.DepthOfCut[lastIndex], 0);
                // Calculate mu_b
                double mu_b = simulationParameters.BitRock.mu * 0.5 * (1 + Math.Exp(- BitRockFrictionExponent * mudoRotorAngularVelocity / (2.0 * Math.PI)));
                // Calculate wb
                wb = Math.PI * Math.Pow(simulationParameters.Drillstring.Rb, 2) * CCS / BitEfficiencyFactor / (1 + 2 * mu_b * simulationParameters.Drillstring.Rb / (3 * state.DepthOfCut[lastIndex]));
                wb = Math.Max(wb, 0);
                // Calculate tb
                tb = 2.0 / 3.0 * mu_b * simulationParameters.Drillstring.Rb * wb;
            }
            else
            {
                tb = 0;
                wb = 0;
            }
            return new double[]{tb, wb};
        }           
    
    }
}