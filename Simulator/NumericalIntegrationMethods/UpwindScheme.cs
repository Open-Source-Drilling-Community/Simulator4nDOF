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
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public class UpwindScheme : ISolverODE<AxialTorsionalModel>
    {     
        private bool simulationDiverged = false;
        public void AddNewLumpedElement(){}
        public bool IntegrationStep(State state, AxialTorsionalModel axialTorsionalModel, in SimulationParameters simulationParameters)
        {   
            // Use the torsional model instance to estimate the accelerations
            axialTorsionalModel.CalculateAccelerations(state, simulationParameters);
            double c1 = simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength;            
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {                                
                for (int j = 0; j < simulationParameters.LumpedCells.NumberOfLumpedElements; j++)
                {
                    // The variation is needed for the upwind scheme. 
                    // The boundary conditions are masked depending on the position.
                    state.DiffDownwardTorsionalWave[i, j] = (
                        (i == 0) ? 
                        (state.DownwardTorsionalWave[i,j] - state.DownwardTorsionalWaveLeftBoundary[j]) 
                        : 
                        (state.DownwardTorsionalWave[i, j] - state.DownwardTorsionalWave[i - 1, j])
                    );
                    state.DiffUpwardTorsionalWave[i, j] = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (state.UpwardTorsionalWaveRightBoundary[j] - state.UpwardTorsionalWave[i, j]) 
                        : 
                        (state.UpwardTorsionalWave[i + 1, j] - state.UpwardTorsionalWave[i, j])
                    );
                    state.DiffDownwardAxialWave[i, j] = (
                        (i == 0) ? 
                        (state.DownwardAxialWave[i,j] - state.DownwardAxialWaveLeftBoundary[j]) 
                        : 
                        (state.DownwardAxialWave[i, j] - state.DownwardAxialWave[i - 1, j])
                    );
                    state.DiffUpwardAxialWave[i, j] = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (state.UpwardAxialWaveRightBoundary[j] - state.UpwardAxialWave[i, j]) 
                        : 
                        (state.UpwardAxialWave[i + 1, j] - state.UpwardAxialWave[i, j])
                    );                
                }
            }
            //Update separately to avoid overwritting
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {                                
                for (int j = 0; j < simulationParameters.LumpedCells.NumberOfLumpedElements; j++)
                {                              
                    state.DownwardTorsionalWave[i, j] -= c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * state.DiffDownwardTorsionalWave[i, j];
                    state.UpwardTorsionalWave[i, j]   += c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * state.DiffUpwardTorsionalWave[i, j];
                    state.DownwardAxialWave[i, j]     -= c1 * simulationParameters.Drillstring.AxialWaveSpeed * state.DiffDownwardAxialWave[i, j];
                    state.UpwardAxialWave[i, j]       += c1 * simulationParameters.Drillstring.AxialWaveSpeed * state.DiffUpwardAxialWave[i, j];   
                
                    if (double.IsNaN(state.DownwardTorsionalWave[i, j]) ||
                        double.IsNaN(state.UpwardTorsionalWave[i, j]) ||
                        double.IsNaN(state.DownwardAxialWave[i, j]) ||
                        double.IsNaN(state.UpwardAxialWave[i, j]))
                    {
                        return false;
                    }               
                }
            }
            return true;
        }
    }
}