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
    public class AxialModel : WaveModel 
    {     
      
        public AxialModel(State state, in SimulationParameters simulationParameters) : base(state, simulationParameters)
        {
            WaveSpeed = simulationParameters.Drillstring.AxialWaveSpeed;
        }

        public override void CalculateAccelerations(State state, in SimulationParameters parameters)
        {   
            state.BitVelocity = 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
           // Axial boundary conditions
            double axialBoundaryTop = - UpwardWave[0] + 2 * state.TopDrive.CalculateSurfaceAxialVelocity;
            double axialBoundaryBottom = - DownwardWave[NumberOfElements - 1] + 2 * state.ZVelocity[state.ZVelocity.Count - 1];
            // Torsional boundary conditions
            #region Update the differentials for the upwind scheme
            for (int i = 0; i < NumberOfElements; i ++)
            {
                // Compute states from Riemann invariants
                Strain[i] = (DownwardWave[i] - UpwardWave[i]) / (2 * WaveSpeed);
                Velocity[i] = 0.5 * (DownwardWave[i] + UpwardWave[i]);                
                //Update state with relevant variables
                state.PipeAxialVelocity[i] = Velocity[i];
                state.PipeAxialStrain[i] = Strain[i];                
                //============= Create differential for waves ===============
                // Axial waves
                DiffDownwardWave[i] = (i==0) ? 
                    DownwardWave[i] - axialBoundaryTop : 
                    DownwardWave[i] - DownwardWave[i-1];
                DiffUpwardWave[i] = (i == NumberOfElements - 1) ? 
                    axialBoundaryBottom - UpwardWave[i] : 
                    UpwardWave[i + 1] - UpwardWave[i];        
            }
            #endregion
            
       }   
  
             
    }
}