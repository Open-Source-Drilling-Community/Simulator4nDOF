namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class BitRock
    {
        public readonly double BRIModel = 1;                    // bit - rock interaction model: 1 - Detournay, 2 - MSE model
        public readonly double epsilon = 100e6;                 // [Pa] Rock strength parameter(used in both Detournay model)
        public readonly double zeta = .8;                       // [-] Ratio of vertical to horizontal cutting forces(Detournay model)
        public readonly double sigma = 60e6;                    // [Pa] Bit - rock contact stress(Detournay model)
        public readonly double l = 5e-3;                        // [m] Wear flat length(Detournay model)
        public readonly double gamma = 1;                       // [-] Bit geometry parameter(Detournay model)
        public readonly double mu = 0.9;                        // [-] Bit - rock friction coefficient(used in both Detournay and MSE model)
        public readonly double N = 5;                           // [-] Number of blades of the PDC bit(Detournay model)
        public readonly double CCS = 10e6;                      // [Pa] Confined compressive strength of rock(MSE model)
        public readonly double bitRockFrictionExponent = 0.1;   // [-] Exponent in rate - dependent torque on bit term(used in both Detournay and MSE model)
        public readonly double bitEfficiencyFactor = 0.35;      // [-] Efficiency factor(MSE model)

        //todo flytte bitEfficiencyfactor til drillstring?

        public double alpha_ROP;                                // ROP filter weight
                                                             
        public double Wf;                                       // [N] Frictional component of weight on bit(Detournay model)
        public double Tf;                                       // [N.m] Frictional component of torque on bit(Detournay model)

        public BitRock(DistributedCells dc, Drillstring ds, double rockStrengthEpsilon, double bitWearLength, double bitRockFrictionCoeff, double pdcBladeNo)
        {
            epsilon = rockStrengthEpsilon;
            l = bitWearLength;
            mu = bitRockFrictionCoeff;
            N = pdcBladeNo;

            // Bit - rock interaction parameters
            Wf = ds.Rb * l * sigma;                             // [N] Frictional component of weight on bit(Detournay model)
            Tf = gamma * mu * ds.Rb / 2 * Wf;                   // [N.m] Frictional component of torque on bit(Detournay model)
            double dtTemp = dc.dxM / Math.Max(ds.c_t, ds.c_a) * 0.80; //As per the CFL condition for the axial / torsional wave equations
            double fc_ROP = 1.0;                                //[Hz] ROP filter cut-off frequency
            alpha_ROP = 2 * Math.PI * dtTemp * fc_ROP / (2 * Math.PI * dtTemp * fc_ROP + 1); // ROP filter weight

        }
    }
}
