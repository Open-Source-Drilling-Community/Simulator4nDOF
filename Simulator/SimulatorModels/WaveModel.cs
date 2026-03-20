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
    public class WaveModel : IModel<WaveModel>
    {             
        // Wave propagation variables
        public Vector<double> DownwardWave;
        public Vector<double> UpwardWave;

        public Vector<double> DiffDownwardWave;
        public Vector<double> DiffUpwardWave;
                
        public double ElementLength;
        public int NumberOfElements;

        public double WaveSpeed;
        public Vector<double> Strain;
        public Vector<double> Velocity;
        //Interpolation variables
        private double currentPosition;
        private int lowerWaveIndex;
        private int upperWaveIndex;            
        private double lowerPosition;
        private double upperPosition;      
        private double deltaStrain;
        private double zeroStrain; 
        private double deltaVelocity;
        private double zeroVelocity;
        public Vector<double> InterpolatedStrain;
        public Vector<double> InterpolatedVelocity;
        public WaveModel(in SimulationParameters simulationParameters)
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
            //UpdateBoundaryConditions(state, simulationParameters);           
        }

        public void PrepareModel(WaveModel model, State state, SimulationParameters parameters)
        {
            for (int i = 0; i < NumberOfElements-1; i++)          
            {
                DownwardWave[i] = Strain[i] + WaveSpeed * Strain[i];
                UpwardWave[i]   = Strain[i] - WaveSpeed * Strain[i];
            }                     
        }
        
        public virtual void CalculateAccelerations(State state, in SimulationParameters parameters)
        {   
            state.BitVelocity = 0.5 * (DownwardWave[NumberOfElements - 1] + UpwardWave[NumberOfElements - 1]);
            // ============== Calculate boundary conditions for the next iteration based on current velocities
            // Axial boundary conditions
            double axialBoundaryTop = - UpwardWave[0] + 2 * state.TopDrive.CalculateSurfaceAxialVelocity;
            double axialBoundaryBottom = - DownwardWave[NumberOfElements - 1] + 2 * state.ZVelocity[state.ZVelocity.Count - 1];
            // Torsional boundary conditions
            #region Update the differentials for the upwind scheme
            for (int i = 0; i < NumberOfElements; i ++)
            {
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
        public void InterpolateStateFromWave(State state, in WaveModel model, in SimulationParameters simulationParameters)
        {                       
            for (int i = 0; i < state.ZVelocity.Count; i++)
            {
                currentPosition = i * simulationParameters.Drillstring.TotalLength / state.ZVelocity.Count;
                //Get the position index
                lowerWaveIndex = (int) Math.Floor(currentPosition / model.ElementLength);
                //Should always hold true. But for the sake of robustness, it is here
                upperWaveIndex = Math.Min(lowerWaveIndex + 1, model.NumberOfElements - 1); 
                lowerPosition = lowerWaveIndex * model.ElementLength;
                upperPosition = upperWaveIndex * model.ElementLength;
                
                //Interpolate strain
                deltaStrain = (model.Strain[upperWaveIndex] - model.Strain[lowerWaveIndex]) / (upperPosition - lowerPosition);
                zeroStrain = model.Strain[lowerWaveIndex] - deltaStrain * lowerPosition;
                InterpolatedStrain[i] = deltaStrain * currentPosition + zeroStrain;                
                //Interpolate velocity
                deltaVelocity = (model.Velocity[upperWaveIndex] - model.Velocity[lowerWaveIndex]) / (upperPosition - lowerPosition);
                zeroVelocity = model.Velocity[lowerWaveIndex] - deltaVelocity * lowerPosition;
                InterpolatedVelocity[i] = deltaVelocity * currentPosition + zeroVelocity;                
            }
        }                
    }
}