using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    /// <summary>
    ///     The interface is used to createa drill-string model.
    /// It is required to have a "PrepareModel" method which  
    /// updates the model before entering the smaller loop. This is 
    /// necessary as some of the functionalities are calculates asynchronously.
    /// 
    /// CalculateAccelerations method is required as it updates the State state
    /// instance of the accelerations which are used for the ODE solver.
    /// </summary>
    /// <typeparam name="TypeModel"></typeparam>
    public interface IModel<TypeModel> where TypeModel : IModel<TypeModel>
    {
        void PrepareModel(in State state, in SimulationParameters parameters);
        void CalculateAccelerations(State state, in SimulationParameters parameters);
        
    }     
}