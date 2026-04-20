
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public class Detournay : IBitRock
    {        
        /// <summary>
        /// [-] Bit - rock friction coefficient(used in both Detournay and MSE model)
        /// </summary>
        private double Mu = 0.9;                        
        //todo flytte bitEfficiencyfactor til drillstring?
        private double AlphaROP;                                // ROP filter weight
        /// <summary>
        /// [N] Frictional component of weight on bit (Detournay model)
        /// </summary>
        private double WeightOnBitFrictionComponent;
        /// <summary>
        /// [N.m] Frictional component of torque on bit (Detournay model)
        /// </summary>
        private double TorqueFrictionComponent;
        /// <summary>
        /// [Pa] Rock strength parameter (used in both Detournay model)
        /// </summary>
        private readonly double RockStrength = 100e6;
        /// <summary>
        /// [-] Ratio of vertical to horizontal cutting forces (Detournay model)
        /// </summary>
        private readonly double zeta = .8;
        /// <summary>
        /// [Pa] Bit - rock contact stress (Detournay model)
        /// </summary>
        private readonly double sigma = 60e6;
        /// <summary>
        /// [m] Wear flat length (Detournay model)
        /// </summary>
        private readonly double l = 5e-3;
        /// <summary>
        /// [-] Bit geometry parameter (Detournay model)
        /// </summary>
        private readonly double gamma = 1;
        /// <summary>
        /// [-] Number of blades of the PDC bit (Detournay model)
        /// </summary>
        private readonly double N = 5;                

        private double torqueOnBit;
        private double weightOnBit;
        private double tangentialVelocity;
        private double bitStrain;

        private double previousDepthOfCut;
        private double diffDepthOfCut;
        private double lastElement;
        private double maxValue;
        private double epsilon;
       
        private double d;
        private double reg;
        private double cuttingWeightOnBit;
        private double cuttingTorque;
        private double ga = 1;
        private double weightOnBitFriction;
        private double torqueFriction;
        public Detournay(
            in SimulatorDrillString drillString,
            in Configuration configuration
            )
        {
            RockStrength = configuration.RockStrengthEpsilon;
            l = configuration.BitWearLength;
            Mu = configuration.BitRockFrictionCoeff;
            N = configuration.PdcBladeNo;
            
            
            /* Alpha ROP is not used! */
            //double dtTemp = axialModel.ElementLength / 
            //    Math.Max(torsionalModel.WaveSpeed, axialModel.WaveSpeed) * 0.80; //As per the CFL condition for the axial / torsional wave equations
            //double fc_ROP = 1.0;                                //[Hz] ROP filter cut-off frequency
            //AlphaROP = 2 * Math.PI * dtTemp * fc_ROP / (2 * Math.PI * dtTemp * fc_ROP + 1); // ROP filter weight
        } 

        public void CalculateInteractionForce(State state, in SimulationParameters parameters, in BitInternalForces bitInternalForces)
        {
            tangentialVelocity = state.AngularVelocity[state.AngularVelocity.Count - 1] / parameters.Drillstring.BitRadius; // Convert bit linear velocity to angular velocity using bit radius 
            bitStrain = (state.ZDisplacement[state.ZDisplacement.Count - 1] - state.ZDisplacement[state.ZDisplacement.Count - 2]) / parameters.Drillstring.ElementLength[parameters.Drillstring.ElementLength.Count - 1]; // Assuming the last element corresponds to the bit
            if (state.BitOnBotton)
            {
                lastElement = state.DepthOfCut[state.DepthOfCut.Count - 1]; // Get the last element of l
                maxValue = Math.Max(lastElement, 0); // Compute max(l(end), 0)
                d = N * maxValue; // Compute d
                epsilon = 2 * Math.PI * 0.2;  // regularization term to avoid numerical issues at zero bit velocity
                cuttingWeightOnBit = d * parameters.Drillstring.BitRadius * zeta * epsilon;   // Cutting component of weight on bit
                cuttingTorque = d * Math.Pow(parameters.Drillstring.BitRadius, 2) * epsilon / 2.0 * Math.Pow(reg, 2); // Cutting component of bit torque
                // Bit - rock interaction parameters
                WeightOnBitFrictionComponent = parameters.Drillstring.BitRadius * l * sigma;                             // [N] Frictional component of weight on bit(Detournay model)
                TorqueFrictionComponent = gamma * Mu * parameters.Drillstring.BitRadius / 2 * WeightOnBitFrictionComponent;                   // [N.m] Frictional component of torque on bit(Detournay model)
                weightOnBitFriction = WeightOnBitFrictionComponent * ga;
            
                reg = tangentialVelocity / Math.Sqrt(Math.Pow(tangentialVelocity, 2) + Math.Pow(epsilon, 2));
                torqueFriction = 0.5 * (1 + Math.Exp(-Mu * tangentialVelocity / (2.0 * Math.PI))) * TorqueFrictionComponent * ga;
                
                double totalBitTorque = bitInternalForces.ElasticTorque - cuttingTorque - torqueFriction;                
                totalBitTorque = (tangentialVelocity < 0.1) ? Math.Min(totalBitTorque, 0) : 0;              

                double totalWeightOnBit = bitInternalForces.ElasticAxialForce - cuttingWeightOnBit - weightOnBitFriction;
                totalWeightOnBit = (Math.Abs(totalBitTorque) > 1e-3) ? totalWeightOnBit : 0;
            
                // Calculate torque on bit and weight on bit
                torqueOnBit = cuttingTorque + torqueFriction + totalBitTorque;
                weightOnBit = cuttingWeightOnBit + weightOnBitFriction + totalWeightOnBit;
                double constant = Math.Max(tangentialVelocity, 0) * N / (2 * Math.PI/parameters.dxl);
                for (int i = 0; i < state.DepthOfCut.Count; i++)
                {
                    previousDepthOfCut = i == 0 ? 0:state.DepthOfCut[i-1];
                    diffDepthOfCut = state.DepthOfCut[i] - previousDepthOfCut;
                    state.DepthOfCut[i] -= parameters.InnerLoopTimeStep * ( constant * diffDepthOfCut - state.ZVelocity[state.ZVelocity.Count - 1]);        
                }
            }
            else
            {
                torqueOnBit = 0;
                weightOnBit = 0;
            }    
            state.TorqueOnBit = torqueOnBit; 
            state.WeightOnBit = weightOnBit;    
            
            return;
        }           
                
    }
}