using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public class UpwindScheme : ISolverODE<WaveModel>
    {     
        //Integration constant
        private double integrationConstant;
   

        public UpwindScheme(in  WaveModel waveModel, in SimulationParameters simulationParameters)
        {
            integrationConstant = waveModel.WaveSpeed * simulationParameters.InnerLoopTimeStep / waveModel.ElementLength;       
        }
        public void AddNewLumpedElement(){}
        public bool IntegrationStep(State state, WaveModel waveModel, in SimulationParameters simulationParameters)
        {   
            // Use the torsional model instance to estimate the accelerations
            waveModel.CalculateAccelerations(state, simulationParameters);
            //Update separately to avoid overwritting
            for (int i = 0; i < waveModel.NumberOfElements; i++)          
            {                                 
                waveModel.DownwardWave[i] -= integrationConstant * waveModel.DiffDownwardWave[i];
                waveModel.UpwardWave[i]   += integrationConstant * waveModel.DiffUpwardWave[i];                    
                if (
                        double.IsNaN(waveModel.DownwardWave[i]) ||
                        double.IsNaN(waveModel.UpwardWave[i]) 
                    )
                {
                    return false;
                }                               
            }
            waveModel.UpdateState(state);
            return true;
        }    
     
    }
}