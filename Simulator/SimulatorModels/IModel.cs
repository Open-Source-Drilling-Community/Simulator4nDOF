using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public interface IModel<TypeModel> where TypeModel : IModel<TypeModel>
    {
        void PrepareModel(TypeModel model, State state, SimulationParameters simulationParameters);
        void CalculateAccelerations(State state, in SimulationParameters parameters);
        
    }     
}