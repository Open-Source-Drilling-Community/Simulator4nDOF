using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class TorsionalModel : WaveModel
    {     
        private double topDriveTorque;
        // Other variables
        private bool useMudMotor;
                            
        
        public TorsionalModel(in SimulationParameters simulationParameters) : base(simulationParameters)
        {
            useMudMotor = simulationParameters.UseMudMotor;
            WaveSpeed = simulationParameters.Drillstring.TorsionalWaveSpeed;
        }
   
        public void IntegrateTopDriveSpeed(State state, in SimulationParameters parameters)
        {
            topDriveTorque = parameters.Drillstring.PipePolarMoment[0] * parameters.Drillstring.ShearModuli[0] * Strain[0];            
            state.TopDrive.TopDriveAngularVelocity = state.TopDrive.TopDriveAngularVelocity + parameters.InnerLoopTimeStep * (state.TopDrive.TopDriveMotorTorque - topDriveTorque) / parameters.TopDriveDrawwork.TopDriveInertia;                
        }
        
        public override void CalculateAccelerations(State state, in SimulationParameters parameters)
        {                               
            double angularVelocity = (useMudMotor) ? 
            state.MudRotorAngularVelocity  : 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
            // ============== Calculate boundary conditions for the next iteration based on current velocities
            UpdateDifferential(state.AngularVelocity,  state.TopDrive.TopDriveAngularVelocity);
            
        }
        public override void UpdateState(State state, in SimulationParameters parameters)
        {                        
            base.UpdateState(state, in parameters);
            state.ShearStrain = InterpolatedStrain;
            state.ShearStrainDifference = Vector<double>.Build.Dense(NumberOfLateralElements);            
            state.ShearStrain = Vector<double>.Build.Dense(NumberOfLateralElements);
            for (int i = 0; i < NumberOfElements; i ++)
            {                
                int j = i / LateralModelToWaveRatio;
                state.PipeAngularVelocity[i] = Velocity[i];
                state.ShearStrainDifference[j] += StrainDifference[i] / LateralModelToWaveRatio;    
            }   
        }

    }
}