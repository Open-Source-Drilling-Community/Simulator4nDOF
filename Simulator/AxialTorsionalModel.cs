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
        public Vector<double> OL_dot;//?
        public Vector<double> OS_dot;//?   
        //
        public Matrix<double> DownwardTorsionalWave; // Downward traveling wave, torsional
        public Matrix<double> UpwardTorsionalWave; // Upward traveling wave, torsional
        public Matrix<double> DownwardAxialWave; // Downward traveling wave, axial
        public Matrix<double> UpwardAxialWave; // Upward traveling wave, axial

        public Vector<double> DownwardTorsionalWaveLeftBoundary;
        public Vector<double> UpwardTorsionalWaveLeftBoundary;
        public Vector<double> DownwardAxialWaveRightBoundary;
        public Vector<double> UpwardAxialWaveRightBoundary;
        public Vector<double> OL_vec;
        public Vector<double> VL_vec;


    }
}