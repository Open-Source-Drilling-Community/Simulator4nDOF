using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public interface IBitRock
    {                    
        
        public double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, Matrix<double> aa, SimulationParameters simulationParameters)
        {
            return new double[]{0,0};
        }
    }
}
