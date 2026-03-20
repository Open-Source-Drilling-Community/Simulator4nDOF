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
        
        
        
        public TorsionalModel(State state, in SimulationParameters simulationParameters) : base(state, simulationParameters)
        {
            useMudMotor = simulationParameters.UseMudMotor;
            WaveSpeed = simulationParameters.Drillstring.TorsionalWaveSpeed;

            //UpdateBoundaryConditions(state, simulationParameters);           
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
            // Axial boundary conditions
            // Torsional boundary conditions
            double torsionalBoundaryTop = - UpwardWave[0] + 2 * state.TopDrive.TopDriveAngularVelocity;
            double torsionalBoundaryBottom = - DownwardWave[NumberOfElements - 1] + 2 * state.AngularVelocity[state.AngularVelocity.Count - 1];
            #region Update the differentials for the upwind scheme
            for (int i = 0; i < NumberOfElements; i ++)
            {        
                // Compute states from Riemann invariants
                Strain[i] = (DownwardWave[i] - UpwardWave[i]) / (2 * WaveSpeed);
                Velocity[i] = 0.5 * (DownwardWave[i] + UpwardWave[i]);
                //Update state with relevant variables
                state.PipeAngularVelocity[i] = Velocity[i];
                state.PipeShearStrain[i] = Strain[i];   
                //============= Create differential for waves ===============
                // Torsional waves
                DiffDownwardWave[i] = (i == 0) ? 
                    DownwardWave[i] - torsionalBoundaryTop : 
                    DownwardWave[i] - DownwardWave[i - 1];
                DiffUpwardWave[i] = (i == NumberOfElements - 1) ? 
                    torsionalBoundaryBottom - UpwardWave[i] : 
                    UpwardWave[i + 1] - UpwardWave[i];                
            }
            #endregion
            //Update the state with the interpolated values of velocity and strain for the next iteration
            InterpolateStateFromWave(state, this, parameters);
            state.AngularVelocity = InterpolatedVelocity;
            state.PipeShearStrain = InterpolatedStrain;
       }                
    }
}