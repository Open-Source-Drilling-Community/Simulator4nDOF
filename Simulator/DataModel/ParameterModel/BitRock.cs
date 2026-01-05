using MathNet.Numerics.LinearAlgebra;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class BitRock
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
        public virtual double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, Matrix<double> aa, SimulationParameters simulationParameters)
        {
            return new double[]{0,0};
        }
    }
}
