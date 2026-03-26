using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class AxialModel : WaveModel 
    {     
      
        public AxialModel(in SimulationParameters simulationParameters) : base(simulationParameters)
        {
            WaveSpeed = simulationParameters.Drillstring.AxialWaveSpeed;
        }

        public override void CalculateAccelerations(State state, in SimulationParameters parameters)
        {   
            state.BitVelocity = 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
            // Axial boundary conditions
            UpdateDifferential(state.ZVelocity, state.TopDrive.CalculateSurfaceAxialVelocity);
            //Update the state with the interpolated values of velocity and strain for the next iteration            
       }   
        public override void UpdateState(State state)
        {                        
            base.UpdateState(state);
            state.PipeAxialStrain[0] = Strain[0];
            state.PipeAxialVelocity[0] = Velocity[0];     
            for (int i = 1; i < NumberOfLateralElements; i ++)
            {
                int j = i * LateralModelToWaveRatio - 1;     
                state.PipeAxialStrain[i] = Strain[j];
                state.PipeAxialVelocity[i] = Velocity[j];                            
            }   
        }
             
    }
}