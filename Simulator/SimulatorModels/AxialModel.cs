using MathNet.Numerics.LinearAlgebra;
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
        public override void UpdateState(State state, in SimulationParameters parameters)
        {                        
            base.UpdateState(state, in parameters);
            //state.ZVelocity[0] = Velocity[0];    
            state.AxialStrainDifference = Vector<double>.Build.Dense(NumberOfLateralElements);       
            state.AxialStrain = InterpolatedStrain;
            for (int i = 0; i < NumberOfElements; i ++)
            {
                int j = i / LateralModelToWaveRatio;     
                state.PipeAxialVelocity[i] = Velocity[i];      
                state.AxialStrainDifference[j] += StrainDifference[i] / LateralModelToWaveRatio;
            }   
        }
             
    }
}