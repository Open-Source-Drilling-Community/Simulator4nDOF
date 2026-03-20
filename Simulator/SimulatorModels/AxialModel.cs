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
      
        public AxialModel(in SimulationParameters simulationParameters) : base(simulationParameters)
        {
             //Needs to be update after testing stage
            ElementLength = simulationParameters.DistributedCells.ElementLength;             
            // Calculate the number of elements based on the total length and element length, ensuring it's an integer
            NumberOfElements = simulationParameters.DistributedCells.NumberOfElements;
            DownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            UpwardWave = Vector<double>.Build.Dense(NumberOfElements);
            Strain = Vector<double>.Build.Dense(NumberOfElements);
            Velocity = Vector<double>.Build.Dense(NumberOfElements);
            DiffDownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            DiffUpwardWave = Vector<double>.Build.Dense(NumberOfElements);  
            InterpolatedStrain = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            InterpolatedVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
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
            //Update the state with the interpolated values of velocity and strain for the next iteration
            InterpolateStateFromWave(state, this, parameters);
            state.ZVelocity = InterpolatedVelocity;
            state.PipeAxialStrain = InterpolatedStrain;
        
       }   
  
             
    }
}