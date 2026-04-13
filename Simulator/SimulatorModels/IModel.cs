using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public interface IModel<TypeModel> where TypeModel : IModel<TypeModel>
    {
        void PrepareModel(in State state, in SimulationParameters simulationParameters);
        void CalculateAccelerations(State state, in SimulationParameters parameters);
        
    }     
}