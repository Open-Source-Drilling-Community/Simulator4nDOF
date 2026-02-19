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

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class AxialTorsionalModel
    {     

        public Matrix<double> DownwardTorsionalWave; // Downward traveling wave, torsional
        public Matrix<double> UpwardTorsionalWave; // Upward traveling wave, torsional
        public Matrix<double> DownwardAxialWave; // Downward traveling wave, axial
        public Matrix<double> UpwardAxialWave; // Upward traveling wave, axial
        public Vector<double> DownwardTorsionalWaveLeftBoundary;
        public Vector<double> UpwardTorsionalWaveRightBoundary;
        public Vector<double> DownwardAxialWaveLeftBoundary;
        public Vector<double> UpwardAxialWaveRightBoundary;

        public Matrix<double> DiffDownwardTorsionalWave; // Downward traveling wave, torsional
        public Matrix<double> DiffUpwardTorsionalWave; // Upward traveling wave, torsional
        public Matrix<double> DiffDownwardAxialWave; // Downward traveling wave, axial
        public Matrix<double> DiffUpwardAxialWave; // Upward traveling wave, axial
        

        public double WeightOnBit;
        public double TorqueOnBit;
        

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput)
        {
            //Dimension initial state
            DownwardTorsionalWave = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, torsional
            UpwardTorsionalWave   = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, torsional
            DownwardAxialWave     = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, axial
            UpwardAxialWave       = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, axial            
            //Delta of the wave used for Upwind Scheme
            DiffDownwardTorsionalWave = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, torsional
            DiffUpwardTorsionalWave   = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, torsional
            DiffDownwardAxialWave     = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, axial
            DiffUpwardAxialWave       = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, axial                        

            // Allocate boundary condition vectors
            DownwardTorsionalWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardTorsionalWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            DownwardAxialWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardAxialWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpdateBoundaryConditions(state, simulationParameters, simulationInput);           
            WeightOnBit = 0.0;
            TorqueOnBit = 0.0;
        }

        public AxialTorsionalModel(State state, SimulationParameters simulationParameters, Input simulationInput, AxialTorsionalModel oldModel)
        {
            //Re-Initialize if there is a new lumped element
            //Dimension initial state
            DownwardTorsionalWave = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, torsional
            UpwardTorsionalWave   = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, torsional
            DownwardAxialWave     = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Downward traveling wave, axial
            UpwardAxialWave       = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, simulationParameters.LumpedCells.NumberOfLumpedElements); // Upward traveling wave, axial
            
             // Allocate boundary condition vectors
            DownwardTorsionalWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardTorsionalWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            DownwardAxialWaveLeftBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            UpwardAxialWaveRightBoundary = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);    
            UpdateBoundaryConditions(state, simulationParameters, simulationInput);                              
           
            WeightOnBit = oldModel.WeightOnBit;
            TorqueOnBit = oldModel.TorqueOnBit; 
        }
        public void UpdateBoundaryConditions(State state, SimulationParameters parameters, Input simulationInput)
        {                                               
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
        public void IntegrateTopDriveSpeed(State state, SimulationParameters parameters, Input simulationInput)
        {
           double topDriveTorque = 0.5 * parameters.Drillstring.PipePolarMoment[0] * parameters.Drillstring.ShearModuli[0] / parameters.Drillstring.TorsionalWaveSpeed *
                (   
                    DownwardTorsionalWave[0, 0] 
                    - UpwardTorsionalWave[0, 0]
                );

            state.TopDriveAngularVelocity = state.TopDriveAngularVelocity + parameters.InnerLoopTimeStep * (simulationInput.TopDriveMotorTorque - topDriveTorque) / parameters.Wellbore.TopDriveInertia;                
            //state.TopOfStringPosition = state.TopOfStringPosition + simulationInput.CalculateSurfaceAxialVelocity * parameters.InnerLoopTimeStep;                             
        }
    }
}