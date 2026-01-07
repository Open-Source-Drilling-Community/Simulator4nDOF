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
        public Vector<double> UpwardTorsionalWaveLeftBoundary;
        public Vector<double> DownwardAxialWaveRightBoundary;
        public Vector<double> UpwardAxialWaveRightBoundary;

        public double WeightOnBit;
        public double TorqueOnBit;
        

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput)
        {
            //Dimension initial state
            DownwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            UpwardTorsionalWave = state.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            DownwardAxialWave = state.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            UpwardAxialWave = state.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
            UpdateBoundaryConditions(state, simulationParameters, simulationInput);           
            // Model with boundary conditions
            DownwardTorsionalWaveStackedWithLeftBoundary = DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardTorsionalWave);
            UpwardTorsionalWaveStackedWithLeftBoundary = UpwardTorsionalWave.Stack(DownwardAxialWaveRightBoundary.ToRowMatrix());               
            DownwardAxialWaveStackedWithRightBoundary = UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardAxialWave);
            UpwardAxialWaveStackedWithRightBoundary = UpwardAxialWave.Stack(UpwardAxialWaveRightBoundary.ToRowMatrix());        
            WeightOnBit = 0.0;
            TorqueOnBit = 0.0;
        }

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, AxialTorsionalModel oldModel)
        {
            //Re-Initialize if there is a new lumped element
            //Dimension initial state
            DownwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            UpwardTorsionalWave = state.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            DownwardAxialWave = state.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            UpwardAxialWave = state.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
            
            // Model with boundary conditions
            DownwardTorsionalWaveStackedWithLeftBoundary = DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardTorsionalWave);
            UpwardTorsionalWaveStackedWithLeftBoundary = UpwardTorsionalWave.Stack(DownwardAxialWaveRightBoundary.ToRowMatrix());               
            DownwardAxialWaveStackedWithRightBoundary = UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardAxialWave);
            UpwardAxialWaveStackedWithRightBoundary = UpwardAxialWave.Stack(UpwardAxialWaveRightBoundary.ToRowMatrix());                 
            WeightOnBit = oldModel.WeightOnBit;
            TorqueOnBit = oldModel.TorqueOnBit; 
        }
        public void UpdateBoundaryConditions(State state, SimulationParameters parameters, Input simulationInput)
        {   
            double axialVelocityLeft;
            double axialVelocityRight;
            double torsionalVelocityLeft;
            double torsionalVelocityRight;        
            for (int i = 0; i < state.AxialVelocity.Count; i++)
            {
                axialVelocityLeft = (i == 0) ? simulationInput.CalculateSurfaceAxialVelocity : state.AxialVelocity[i - 1];
                torsionalVelocityLeft = (i == 0) ? state.TopDriveAngularVelocity : state.AngularVelocity[i - 1];
                axialVelocityRight = state.AxialVelocity[i];
                torsionalVelocityRight = state.AngularVelocity[i];
                // Left boundaries   
                DownwardTorsionalWaveLeftBoundary[i] = - UpwardTorsionalWave[0, i] + 2 * torsionalVelocityLeft;
                UpwardTorsionalWaveLeftBoundary[i]   = - UpwardAxialWave[0, i] + 2 * axialVelocityLeft;
                // Right boundaries
                DownwardAxialWaveRightBoundary[i] = - DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] + 2 * torsionalVelocityRight;
                UpwardAxialWaveRightBoundary[i] = - DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] + 2 * axialVelocityRight;                        
            }
            //RotationalVelocity = ExtendVectorStart(state.TopDriveAngularVelocity, state.AngularVelocity);
            //AxialVelocity = ExtendVectorStart(state.AxialVelocity[0], state.AxialVelocity);
            //// Left boundaries
            //DownwardTorsionalWaveLeftBoundary = -UpwardTorsionalWave.Row(0) + 2 * RotationalVelocity.SubVector(0, RotationalVelocity.Count - 1);
            //UpwardTorsionalWaveLeftBoundary = -UpwardAxialWave.Row(0) + 2 * AxialVelocity.SubVector(0, AxialVelocity.Count - 1);
            //// Right boundaries
            //DownwardAxialWaveRightBoundary = -DownwardTorsionalWave.Row(simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * RotationalVelocity.SubVector(1, RotationalVelocity.Count - 1);
            //UpwardAxialWaveRightBoundary = -DownwardAxialWave.Row(simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) + 2 * AxialVelocity.SubVector(1, AxialVelocity.Count - 1);            
        }
    }
}