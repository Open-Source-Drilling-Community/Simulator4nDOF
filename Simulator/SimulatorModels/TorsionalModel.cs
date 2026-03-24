using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Diagnostics;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

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
            topDriveTorque = 0.5 * parameters.Drillstring.PipePolarMoment[0] * parameters.Drillstring.ShearModuli[0] / parameters.Drillstring.TorsionalWaveSpeed *
                (   
                    DownwardWave[0] 
                    - UpwardWave[0]
                );            
            state.TopDrive.TopDriveAngularVelocity = state.TopDrive.TopDriveAngularVelocity + parameters.InnerLoopTimeStep * (state.TopDrive.TopDriveMotorTorque - topDriveTorque) / parameters.TopDriveDrawwork.TopDriveInertia;                
        }
        
        public override void CalculateAccelerations(State state, in SimulationParameters parameters)
        {                               
            double angularVelocity = (useMudMotor) ? 
            state.MudRotorAngularVelocity  : 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
            // ============== Calculate boundary conditions for the next iteration based on current velocities
            UpdateDifferential(state.AngularVelocity,  parameters.TopDriveDrawwork.SurfaceRotation);
            InterpolateStateFromWave(state, this, parameters);
            state.PipeAngularVelocity = InterpolatedVelocity;                
            //state.AngularVelocity = InterpolatedVelocity;
            state.PipeShearStrain = InterpolatedStrain;
       }                
    }
}