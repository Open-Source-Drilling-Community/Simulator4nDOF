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

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public static class UpwindScheme
    {     
        public static void IntegrationStep(AxialTorsionalModel torsionalModel, SimulationParameters simulationParameters)
        {   
            double c1 = simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength;            
           
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {                                
                for (int j = 0; j < simulationParameters.LumpedCells.NumberOfLumpedElements; j++)
                {
                    // The variation is needed for the upwind scheme. 
                    // The boundary conditions are masked depending on the position.
                    torsionalModel.DiffDownwardTorsionalWave[i, j] = (
                        (i == 0) ? 
                        (torsionalModel.DownwardTorsionalWave[i,j] - torsionalModel.DownwardTorsionalWaveLeftBoundary[j]) 
                        : 
                        (torsionalModel.DownwardTorsionalWave[i, j] - torsionalModel.DownwardTorsionalWave[i - 1, j])
                    );
                    torsionalModel.DiffUpwardTorsionalWave[i, j] = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (torsionalModel.UpwardTorsionalWaveRightBoundary[j] - torsionalModel.UpwardTorsionalWave[i, j]) 
                        : 
                        (torsionalModel.UpwardTorsionalWave[i + 1, j] - torsionalModel.UpwardTorsionalWave[i, j])
                    );
                    torsionalModel.DiffDownwardAxialWave[i, j] = (
                        (i == 0) ? 
                        (torsionalModel.DownwardAxialWave[i,j] - torsionalModel.DownwardAxialWaveLeftBoundary[j]) 
                        : 
                        (torsionalModel.DownwardAxialWave[i, j] - torsionalModel.DownwardAxialWave[i - 1, j])
                    );
                    torsionalModel.DiffUpwardAxialWave[i, j] = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (torsionalModel.UpwardAxialWaveRightBoundary[j] - torsionalModel.UpwardAxialWave[i, j]) 
                        : 
                        (torsionalModel.UpwardAxialWave[i + 1, j] - torsionalModel.UpwardAxialWave[i, j])
                    );                
                }
            }
            //Update separately to avoid overwritting
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {                                
                for (int j = 0; j < simulationParameters.LumpedCells.NumberOfLumpedElements; j++)
                {                              
                    torsionalModel.DownwardTorsionalWave[i, j] -= c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * torsionalModel.DiffDownwardTorsionalWave[i, j];
                    torsionalModel.UpwardTorsionalWave[i, j]   += c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * torsionalModel.DiffUpwardTorsionalWave[i, j];
                    torsionalModel.DownwardAxialWave[i, j]     -= c1 * simulationParameters.Drillstring.AxialWaveSpeed * torsionalModel.DiffDownwardAxialWave[i, j];
                    torsionalModel.UpwardAxialWave[i, j]       += c1 * simulationParameters.Drillstring.AxialWaveSpeed * torsionalModel.DiffUpwardAxialWave[i, j];   
                }
            }
        
        }
    }
}