using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class AxialTorsionalModel
    {     

        public Matrix<double> DownwardTorsionalWave; // Downward traveling wave, torsional
        public Matrix<double> UpwardTorsionalWave; // Upward traveling wave, torsional
        public Matrix<double> DownwardAxialWave; // Downward traveling wave, axial
        public Matrix<double> UpwardAxialWave; // Upward traveling wave, axial

        public Matrix<double> DownwardTorsionalWaveStackedWithLeftBoundary; // Downward traveling wave, torsional
        public Matrix<double> UpwardTorsionalWaveStackedWithLeftBoundary; // Upward traveling wave, torsional
        public Matrix<double> DownwardAxialWaveStackedWithRightBoundary; // Downward traveling wave, axial
        public Matrix<double> UpwardAxialWaveStackedWithRightBoundary; // Upward traveling wave, axial

        public Vector<double> DownwardTorsionalWaveLeftBoundary;
        public Vector<double> UpwardTorsionalWaveRightBoundary;
        public Vector<double> DownwardAxialWaveLeftBoundary;
        public Vector<double> UpwardAxialWaveRightBoundary;

        public Vector<double> AxialVelocity;
        public Vector<double> RotationalVelocity;

        public double WeightOnBit;
        public double TorqueOnBit;
        

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput)
        {
            //Dimension initial state
            DownwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            UpwardTorsionalWave = state.PipeAngularVelocity   - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            DownwardAxialWave = state.PipeAxialVelocity       + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            UpwardAxialWave = state.PipeAxialVelocity         - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
            // Allocate boundary condition vectors
            DownwardTorsionalWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardTorsionalWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            DownwardAxialWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardAxialWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpdateBoundaryConditions(state, simulationParameters, simulationInput);           
            // Model with boundary conditions
            DownwardTorsionalWaveStackedWithLeftBoundary = DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardTorsionalWave);
            UpwardTorsionalWaveStackedWithLeftBoundary = UpwardTorsionalWave.Stack(DownwardAxialWaveLeftBoundary.ToRowMatrix());               
            DownwardAxialWaveStackedWithRightBoundary = UpwardTorsionalWaveRightBoundary.ToRowMatrix().Stack(DownwardAxialWave);
            UpwardAxialWaveStackedWithRightBoundary = UpwardAxialWave.Stack(UpwardAxialWaveRightBoundary.ToRowMatrix());        
            WeightOnBit = 0.0;
            TorqueOnBit = 0.0;
        }

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput, AxialTorsionalModel oldModel)
        {
            //Re-Initialize if there is a new lumped element
            //Dimension initial state
            DownwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            UpwardTorsionalWave = state.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            DownwardAxialWave = state.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            UpwardAxialWave = state.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
             // Allocate boundary condition vectors
            DownwardTorsionalWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardTorsionalWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            DownwardAxialWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardAxialWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);    
            UpdateBoundaryConditions(state, simulationParameters, simulationInput);                              
            // Model with boundary conditions
            DownwardTorsionalWaveStackedWithLeftBoundary = DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardTorsionalWave);
            UpwardTorsionalWaveStackedWithLeftBoundary   = UpwardTorsionalWave.Stack(UpwardTorsionalWaveRightBoundary.ToRowMatrix());     

            DownwardAxialWaveStackedWithRightBoundary    = DownwardAxialWaveLeftBoundary.ToRowMatrix().Stack(DownwardAxialWave);
            UpwardAxialWaveStackedWithRightBoundary      = UpwardAxialWave.Stack(UpwardAxialWaveRightBoundary.ToRowMatrix());                 
            
            WeightOnBit = oldModel.WeightOnBit;
            TorqueOnBit = oldModel.TorqueOnBit; 
        }
        public void UpdateBoundaryConditions(State state, SimulationParameters parameters, Input simulationInput)
        {   
        
            //RotationalVelocity = ExtendVectorStart(state.TopDriveAngularVelocity, state.AngularVelocity);
            //AxialVelocity = ExtendVectorStart(simulationInput.CalculateSurfaceAxialVelocity, state.AxialVelocity);
            //
            ////Left boundaries
            //DownwardTorsionalWaveLeftBoundary = - UpwardTorsionalWave.Row(0) + 2 * RotationalVelocity.SubVector(0, RotationalVelocity.Count - 1);
            //DownwardAxialWaveLeftBoundary = - UpwardAxialWave.Row(0) + 2 * AxialVelocity.SubVector(0, AxialVelocity.Count - 1);            
            //// Right boundaries
            //UpwardTorsionalWaveRightBoundary = -DownwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * RotationalVelocity.SubVector(1, RotationalVelocity.Count - 1);            
            //UpwardAxialWaveRightBoundary = -DownwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * AxialVelocity.SubVector(1, AxialVelocity.Count - 1);            
                                    
            //// Left boundaries
            //DownwardTorsionalWaveLeftBoundary = -UpwardTorsionalWave.Row(0) + 2 * RotationalVelocity.SubVector(0, RotationalVelocity.Count - 1);
            //UpwardTorsionalWaveLeftBoundary = -UpwardAxialWave.Row(0) + 2 * AxialVelocity.SubVector(0, AxialVelocity.Count - 1);
            //// Right boundaries
            //DownwardAxialWaveRightBoundary = -DownwardTorsionalWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * RotationalVelocity.SubVector(1, RotationalVelocity.Count - 1);
            //UpwardAxialWaveRightBoundary = -DownwardAxialWave.Row(parameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * AxialVelocity.SubVector(1, AxialVelocity.Count - 1);            
            
            int N = state.AxialVelocity.Count;
            int idx = parameters.LumpedCells.DistributedToLumpedRatio - 1;
            for (int i = 0; i < N; i++)
            {
                double torsionalVelocityLeft = (i == 0) ? state.TopDriveAngularVelocity : state.AngularVelocity[i - 1];
                double axialVelocityLeft = (i == 0) ? simulationInput.CalculateSurfaceAxialVelocity : state.AxialVelocity[i - 1];
                //Left boundaries
                DownwardTorsionalWaveLeftBoundary[i] = - UpwardTorsionalWave[0, i] + 2 * torsionalVelocityLeft;
                DownwardAxialWaveLeftBoundary[i]    = - UpwardAxialWave[0, i] + 2 * axialVelocityLeft;
                //Right boundaries
                UpwardTorsionalWaveRightBoundary[i] = - DownwardTorsionalWave[idx, i] + 2 * state.AngularVelocity[i];
                UpwardAxialWaveRightBoundary[i]    = - DownwardAxialWave[idx, i] + 2 * state.AxialVelocity[i];
            }         
        
        }
    }
}