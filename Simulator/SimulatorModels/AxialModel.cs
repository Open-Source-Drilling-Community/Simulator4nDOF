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
            WaveSpeed = simulationParameters.Drillstring.AxialWaveSpeed;
        }

        public override void CalculateAccelerations(State state, in SimulationParameters parameters)
        {   
            state.BitVelocity = 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
            // Axial boundary conditions
            UpdateDifferential(state.ZVelocity, parameters.TopDriveDrawwork.SurfaceAxialVelocity);
            //Update the state with the interpolated values of velocity and strain for the next iteration
            InterpolateStateFromWave(state, this, parameters);
            state.PipeAxialVelocity = InterpolatedVelocity;                
            state.PipeAxialStrain = InterpolatedStrain;
        
       }   
  
             
    }
}