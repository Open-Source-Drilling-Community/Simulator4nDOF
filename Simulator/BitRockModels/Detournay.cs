using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
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
        
        
        public Detournay(
            in AxialModel axialModel,
            in TorsionalModel torsionalModel,                            
            in SimulatorDrillString drillString,
            in Configuration configuration
            )
        {
            epsilon = configuration.RockStrengthEpsilon;
            l = configuration.BitWearLength;
            Mu = configuration.BitRockFrictionCoeff;
            N = configuration.PdcBladeNo;

            // Bit - rock interaction parameters
            WeightOnBitFrictionComponent = drillString.BitRadius * l * sigma;                             // [N] Frictional component of weight on bit(Detournay model)
            TorqueFrictionComponent = gamma * Mu * drillString.BitRadius / 2 * WeightOnBitFrictionComponent;                   // [N.m] Frictional component of torque on bit(Detournay model)
            double dtTemp = axialModel.ElementLength / 
                Math.Max(torsionalModel.WaveSpeed, axialModel.WaveSpeed) * 0.80; //As per the CFL condition for the axial / torsional wave equations
            double fc_ROP = 1.0;                                //[Hz] ROP filter cut-off frequency
            AlphaROP = 2 * Math.PI * dtTemp * fc_ROP / (2 * Math.PI * dtTemp * fc_ROP + 1); // ROP filter weight

        } 

        public void CalculateInteractionForce(State state, 
                                              in AxialModel axialModel, 
                                              in TorsionalModel torsionalModel, 
                                              in SimulationParameters simulationParameters)
        {
            double tb;
            double wb;
            double angularVelocity = state.BitVelocity / simulationParameters.Drillstring.BitRadius; // Convert bit linear velocity to angular velocity using bit radius 
            if (state.BitOnBotton)
            {
                double previousDepthOfCut;
                double diffDepthOfCut;
                int lpSize = (int)simulationParameters.LumpedCells.DistributedToLumpedRatio - 1;
                double lastElement = state.DepthOfCut[state.DepthOfCut.Count - 1]; // Get the last element of l
                double maxValue = Math.Max(lastElement, 0); // Compute max(l(end), 0)
                double d = this.N * maxValue; // Compute d
                double epsilon = 2 * Math.PI * 0.2;  // regularization term to avoid numerical issues at zero bit velocity
                double reg = angularVelocity / Math.Sqrt(Math.Pow(angularVelocity, 2) + Math.Pow(epsilon, 2));
                double wc = d * simulationParameters.Drillstring.BitRadius * zeta * epsilon;   // Cutting component of weight on bit
                double tc = d * Math.Pow(simulationParameters.Drillstring.BitRadius, 2) * epsilon / 2.0 * Math.Pow(reg, 2); // Cutting component of bit torque
                double ga = 1.0;
                double wf = WeightOnBitFrictionComponent * ga;
                double tf = 0.5 * (1 + Math.Exp(-Mu * angularVelocity / (2.0 * Math.PI))) * TorqueFrictionComponent * ga;
                double g_tt = torsionalModel.DownwardWave[torsionalModel.DownwardWave.Count - 1] * 
                                simulationParameters.Drillstring.PipePolarMoment.Last() 
                                * simulationParameters.Drillstring.ShearModuli.Last() / simulationParameters.Drillstring.TorsionalWaveSpeed - tc - tf;
                g_tt = (angularVelocity < 0.1) ? Math.Min(g_tt, 0) : 0;                        
                double g_wt = axialModel.DownwardWave[axialModel.DownwardWave.Count - 1] 
                    * simulationParameters.Drillstring.PipeArea.Last() * 
                    simulationParameters.Drillstring.YoungModuli.Last() / simulationParameters.Drillstring.AxialWaveSpeed - wc - wf;
                g_wt = (Math.Abs(g_tt) > 1e-3) ? g_wt : 0;
            
                // Calculate torque on bit and weight on bit
                tb = tc + tf + g_tt;
                wb = wc + wf + g_wt;
                double constant = Math.Max(angularVelocity, 0) * N / (2 * Math.PI/simulationParameters.dxl);
                for (int i = 0; i < state.DepthOfCut.Count; i++)
                {
                    previousDepthOfCut = i == 0 ? 0:state.DepthOfCut[i-1];
                    diffDepthOfCut = state.DepthOfCut[i] - previousDepthOfCut;
                    state.DepthOfCut[i] -= simulationParameters.InnerLoopTimeStep * ( constant * diffDepthOfCut - state.BitVelocity);        
                }
            }
            else
            {
                tb = 0;
                wb = 0;
            }    
            state.TorqueOnBit = tb; 
            state.WeightOnBit = wb;    
            
            return;
        }           
                
    }
}