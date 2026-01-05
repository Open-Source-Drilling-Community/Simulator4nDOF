using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public interface ISolverODE
    {
        void IntegrationStep(State state, SimulationParameters simulationParameters);

    }     
}