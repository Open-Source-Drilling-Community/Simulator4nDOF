using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public interface ISolverODE<TypeModel> where TypeModel : IModel<TypeModel>
    {
        void IntegrationStep(State state, TypeModel model, SimulationParameters simulationParameters);
        void AddNewLumpedElement();
    }     
}