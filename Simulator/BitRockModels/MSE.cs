using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public class MSE : IBitRock
    {
        /// <summary>
        /// [-] Bit - rock friction coefficient(used in both Detournay and MSE model)
        /// </summary>
        public double Mu = 0.9;                        
        //todo flytte bitEfficiencyfactor til drillstring?
        public double AlphaROP;                                // ROP filter weight
        /// <summary>
        /// [N] Frictional component of weight on bit (Detournay model)
        /// </summary>
        public double WeightOnBitFrictionComponent;
        /// <summary>
        /// [N.m] Frictional component of torque on bit (Detournay model)
        /// </summary>
        public double TorqueFrictionComponent;
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
        public double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, SimulationParameters simulationParameters)
        {
            double tb = 0.0;
            double wb = 0.0;
            if (state.BitOnBotton)
            {
                // Update the last element of l
                int lastIndex = state.DepthOfCut.Count - 1;
                state.DepthOfCut[lastIndex] = (1 - AlphaROP) * state.DepthOfCut[lastIndex] + AlphaROP * 2 * Math.PI * state.BitVelocity / mudoRotorAngularVelocity * (mudoRotorAngularVelocity > 0.5 ? 1 : 0);
                state.DepthOfCut[lastIndex] = Math.Max(state.DepthOfCut[lastIndex], 0);
                // Calculate mu_b
                double mu_b = Mu * 0.5 * (1 + Math.Exp(- BitRockFrictionExponent * mudoRotorAngularVelocity / (2.0 * Math.PI)));
                // Calculate wb
                wb = Math.PI * Math.Pow(simulationParameters.Drillstring.BitRadius, 2) * CCS / BitEfficiencyFactor / (1 + 2 * mu_b * simulationParameters.Drillstring.BitRadius / (3 * state.DepthOfCut[lastIndex]));
                wb = Math.Max(wb, 0);
                // Calculate tb
                tb = 2.0 / 3.0 * mu_b * simulationParameters.Drillstring.BitRadius * wb;
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