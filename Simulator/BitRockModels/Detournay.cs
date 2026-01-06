using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public class Detournay : IBitRock
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
        /// [Pa] Rock strength parameter (used in both Detournay model)
        /// </summary>
        public readonly double epsilon = 100e6;
        /// <summary>
        /// [-] Ratio of vertical to horizontal cutting forces (Detournay model)
        /// </summary>
        public readonly double zeta = .8;
        /// <summary>
        /// [Pa] Bit - rock contact stress (Detournay model)
        /// </summary>
        public readonly double sigma = 60e6;
        /// <summary>
        /// [m] Wear flat length (Detournay model)
        /// </summary>
        public readonly double l = 5e-3;
        /// <summary>
        /// [-] Bit geometry parameter (Detournay model)
        /// </summary>
        public readonly double gamma = 1;
        /// <summary>
        /// [-] Number of blades of the PDC bit (Detournay model)
        /// </summary>
        public readonly double N = 5;        
        
        
        public Detournay(DistributedCells dc, Drillstring ds, double rockStrengthEpsilon, double bitWearLength, double bitRockFrictionCoeff, double pdcBladeNo)
        {
            epsilon = rockStrengthEpsilon;
            l = bitWearLength;
            Mu = bitRockFrictionCoeff;
            N = pdcBladeNo;

            // Bit - rock interaction parameters
            WeightOnBitFrictionComponent = ds.Rb * l * sigma;                             // [N] Frictional component of weight on bit(Detournay model)
            TorqueFrictionComponent = gamma * Mu * ds.Rb / 2 * WeightOnBitFrictionComponent;                   // [N.m] Frictional component of torque on bit(Detournay model)
            double dtTemp = dc.dxM / Math.Max(ds.TorsionalWaveSpeed, ds.AxialWaveSpeed) * 0.80; //As per the CFL condition for the axial / torsional wave equations
            double fc_ROP = 1.0;                                //[Hz] ROP filter cut-off frequency
            AlphaROP = 2 * Math.PI * dtTemp * fc_ROP / (2 * Math.PI * dtTemp * fc_ROP + 1); // ROP filter weight

        } 

        public double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, Matrix<double> aa, SimulationParameters simulationParameters)
        {
            double tb = 0;
            double wb = 0;
            int lpSize = (int)simulationParameters.LumpedCells.DistributedToLumpedRatio - 1;
            double lastElement = state.DepthOfCut[state.DepthOfCut.Count - 1]; // Get the last element of l
            double maxValue = Math.Max(lastElement, 0); // Compute max(l(end), 0)
            double d = this.N * maxValue; // Compute d
            double epsilon = 2 * Math.PI * 0.2;  // regularization term to avoid numerical issues at zero bit velocity
            double reg = mudoRotorAngularVelocity / Math.Sqrt(Math.Pow(mudoRotorAngularVelocity, 2) + Math.Pow(epsilon, 2));
            double wc = d * simulationParameters.Drillstring.Rb * zeta * epsilon;   // Cutting component of weight on bit
            double tc = d * Math.Pow(simulationParameters.Drillstring.Rb, 2) * epsilon / 2.0 * Math.Pow(reg, 2); // Cutting component of bit torque
            double ga = 1.0;
            double wf = WeightOnBitFrictionComponent * ga;
            double tf = 0.5 * (1 + Math.Exp(-Mu * mudoRotorAngularVelocity / (2.0 * Math.PI))) * TorqueFrictionComponent * ga;
            double g_tt = state.TorsionalDownwardTravelingWave[lpSize, state.TorsionalDownwardTravelingWave.ColumnCount - 1] * simulationParameters.Drillstring.PipePolarMoment.Last() * simulationParameters.Drillstring.ShearModuli.Last() / simulationParameters.Drillstring.TorsionalWaveSpeed - tc - tf;
            g_tt = (mudoRotorAngularVelocity < 0.1) ? Math.Min(g_tt, 0) : 0;
            double g_wt = aa[lpSize, aa.ColumnCount - 1] * simulationParameters.Drillstring.PipeArea.Last() * simulationParameters.Drillstring.YoungModuli.Last() / simulationParameters.Drillstring.AxialWaveSpeed - wc - wf;
            g_wt = (Math.Abs(g_tt) > 1e-3) ? g_wt : 0;
            if (state.onBottom)
            {
                // Calculate torque on bit and weight on bit
                tc = tc + tf + g_tt;
                wb = wc + wf + g_wt;
                Vector<double> l_pad = Vector<double>.Build.Dense(state.DepthOfCut.Count + 1);
                l_pad[0] = 0; // Left padding
                state.DepthOfCut.CopySubVectorTo(l_pad, 0, 1, state.DepthOfCut.Count);
                Vector<double> diff_l_pad = Vector<double>.Build.Dense(l_pad.Count - 1);
                for (int i = 0; i < diff_l_pad.Count; i++)
                {
                    diff_l_pad[i] = l_pad[i + 1] - l_pad[i];
                }
                state.DepthOfCut = state.DepthOfCut - (simulationParameters.InnerLoopTimeStep / simulationParameters.dxl) * Math.Max(mudoRotorAngularVelocity, 0) * N / (2 * Math.PI) * diff_l_pad + simulationParameters.InnerLoopTimeStep  * state.BitVelocity;
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