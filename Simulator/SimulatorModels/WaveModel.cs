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
        public double TopBoundary;
        public double BottomBoundary;
        


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
        private double deltaStrain;
        private double zeroStrain; 
        private double deltaVelocity;
        private double zeroVelocity;
        public double TopBoundaryVelocity;
        public double BottomBoundaryVelocity;
        public double TopBoundaryStrain;
        public double BottomBoundaryStrain;
        public int WaveToLumpedRatio;
        public Vector<double> InterpolatedStrain;
        public Vector<double> InterpolatedVelocity;
        public WaveModel(in SimulationParameters simulationParameters)
        {
            //Needs to be update after testing stage
            ElementLength = simulationParameters.DistributedCells.ElementLength;   
            WaveToLumpedRatio = simulationParameters.DistributedCells.WaveToLumpedRatio;          
            // Calculate the number of elements based on the total length and element length, ensuring it's an integer
            NumberOfElements = simulationParameters.DistributedCells.NumberOfElements + 2;
            DownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            UpwardWave = Vector<double>.Build.Dense(NumberOfElements);
            Strain = Vector<double>.Build.Dense(NumberOfElements + 1);
            Velocity = Vector<double>.Build.Dense(NumberOfElements + 1);
            DiffDownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            DiffUpwardWave = Vector<double>.Build.Dense(NumberOfElements);  
            InterpolatedStrain = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            InterpolatedVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            WaveSpeed = simulationParameters.Drillstring.AxialWaveSpeed;
            //UpdateBoundaryConditions(state, simulationParameters);           
        }

        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
            for (int i = 0; i < NumberOfElements-1; i++)          
            {
                DownwardWave[i] = Strain[i] + WaveSpeed * Strain[i];
                UpwardWave[i]   = Strain[i] - WaveSpeed * Strain[i];
            }                     
        }
        public void UpdateDifferential(Vector<double> veloctyVector, double initialVelocity)
        {

            for (int i = 1; i < NumberOfElements - 1; i++)
            {
                double auxDownward =  2 * veloctyVector[WaveToLumpedRatio * i - 1] - UpwardWave[i];
                double auxUpward =  2 * veloctyVector[WaveToLumpedRatio * i - 1] - DownwardWave[i];
                DownwardWave[i] = auxDownward;
                UpwardWave[i] = auxUpward;
            }
            DownwardWave[0] =  2 * initialVelocity - UpwardWave[0];            
            Velocity[0] = initialVelocity;
            Strain[0] = (UpwardWave[0] - DownwardWave[0]) / (2 * WaveSpeed);

            //Create differential wave 
            for (int i = 1; i < NumberOfElements - 1; i ++)
            {                                
                // Compute states from Riemann invariants
                Strain[i] = (DownwardWave[i] - UpwardWave[i]) / (2 * WaveSpeed);
                Velocity[i] = 0.5 * (DownwardWave[i] + UpwardWave[i]);
                //============= Create differential for waves ===============                   
                // Torsional waves
                DiffDownwardWave[i] = DownwardWave[i] - DownwardWave[i - 1];
                DiffUpwardWave[i] = UpwardWave[i + 1] - UpwardWave[i];            
            }    
        }
        public virtual void CalculateAccelerations(State state, in SimulationParameters parameters)
        {
            
        }
        public void InterpolateStateFromWave(State state, 
            in WaveModel model, 
            in SimulationParameters simulationParameters)
        {                       
            for (int i = 0; i < state.ZVelocity.Count; i++)
            {

                currentPosition = i * simulationParameters.Drillstring.TotalLength / state.ZVelocity.Count;
                //Get the position index
                lowerWaveIndex = (int) Math.Floor(currentPosition / model.ElementLength);
                upperWaveIndex = lowerWaveIndex + 1;
                lowerPosition = lowerWaveIndex * model.ElementLength;
                
                //Interpolate strain
                deltaStrain = (model.Strain[upperWaveIndex] - model.Strain[lowerWaveIndex]) / model.ElementLength;//(upperPosition - lowerPosition);
                zeroStrain = model.Strain[lowerWaveIndex] - deltaStrain * lowerPosition;
                InterpolatedStrain[i] = deltaStrain * currentPosition + zeroStrain;                
                //Interpolate velocity
                deltaVelocity = (model.Velocity[upperWaveIndex] - model.Velocity[lowerWaveIndex]) / model.ElementLength;// (upperPosition - lowerPosition);
                zeroVelocity = model.Velocity[lowerWaveIndex] - deltaVelocity * lowerPosition;
                InterpolatedVelocity[i] = deltaVelocity * currentPosition + zeroVelocity;                
            }
        }                
    }
}