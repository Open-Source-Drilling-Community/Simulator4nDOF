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
        public Vector<double> OL_vec;
        public Vector<double> VL_vec;

        public double WeightOnBit;
        public double TorqueOnBit;
        

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput)
        {
            //Dimension initial state
            DownwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            UpwardTorsionalWave = state.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            DownwardAxialWave = state.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            UpwardAxialWave = state.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
            OL_vec = ExtendVectorStart(state.TopDriveAngularVelocity, state.AngularVelocity);
            VL_vec = ExtendVectorStart(simulationInput.CalculateSurfaceAxialVelocity, state.AxialVelocity);
            // Left boundaries
            DownwardTorsionalWaveLeftBoundary = -UpwardTorsionalWave.Row(0) + 2 * OL_vec.SubVector(0, OL_vec.Count - 1);
            UpwardTorsionalWaveLeftBoundary = -UpwardAxialWave.Row(0) + 2 * VL_vec.SubVector(0, VL_vec.Count - 1);
            // Right boundaries
            DownwardAxialWaveRightBoundary = -DownwardTorsionalWave.Row(simulationParameters.LumpedCells.PL - 1) + 2 * OL_vec.SubVector(1, OL_vec.Count - 1);
            UpwardAxialWaveRightBoundary = -DownwardAxialWave.Row(simulationParameters.LumpedCells.PL - 1) + 2 * VL_vec.SubVector(1, VL_vec.Count - 1);
            // Model with boundary conditions
            DownwardTorsionalWaveStackedWithLeftBoundary = DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardTorsionalWave);
            UpwardTorsionalWaveStackedWithLeftBoundary = UpwardTorsionalWave.Stack(DownwardAxialWaveRightBoundary.ToRowMatrix());               
            DownwardAxialWaveStackedWithRightBoundary = UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(DownwardAxialWave);
            UpwardAxialWaveStackedWithRightBoundary = UpwardAxialWave.Stack(UpwardAxialWaveRightBoundary.ToRowMatrix());        
            WeightOnBit = 0.0;
            TorqueOnBit = 0.0;
        }
    }
}