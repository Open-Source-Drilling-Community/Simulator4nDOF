using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public interface ISolverODE<TypeModel> where TypeModel : IModel<TypeModel>
    {
        bool IntegrationStep(State state, TypeModel model, in SimulationParameters simulationParameters);
        bool IntegrationSurfacePosition(State state, TypeModel model, in SimulationParameters simulationParameters);
        bool IntegrationSleeve(State state, TypeModel model, in SimulationParameters simulationParameters);
        void AddNewLumpedElement();
        bool SimulationDivergedCheck(in State state, in int i);
        
    }     
}