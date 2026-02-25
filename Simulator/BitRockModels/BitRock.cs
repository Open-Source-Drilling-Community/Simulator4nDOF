using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public interface IBitRock
    {                    

        public double[] CalculateInteractionForce(State state, double mudoRotorAngularVelocity, SimulationParameters simulationParameters)
        {
            return new double[]{0,0};
        }
    }
}
