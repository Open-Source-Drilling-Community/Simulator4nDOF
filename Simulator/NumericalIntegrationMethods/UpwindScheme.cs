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
            //torsionalModel.DownwardTorsionalWave = torsionalModel.DownwardTorsionalWave - c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(torsionalModel.DownwardTorsionalWaveStackedWithLeftBoundary);
            //torsionalModel.UpwardTorsionalWave   = torsionalModel.UpwardTorsionalWave   + c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(torsionalModel.UpwardTorsionalWaveStackedWithLeftBoundary);
            //torsionalModel.DownwardAxialWave     = torsionalModel.DownwardAxialWave     - c1 * simulationParameters.Drillstring.AxialWaveSpeed *     DiffRows(torsionalModel.DownwardAxialWaveStackedWithRightBoundary);
            //torsionalModel.UpwardAxialWave       = torsionalModel.UpwardAxialWave       + c1 * simulationParameters.Drillstring.AxialWaveSpeed *     DiffRows(torsionalModel.UpwardAxialWaveStackedWithRightBoundary);         
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {
                for (int j = 0; j < simulationParameters.LumpedCells.NumberOfLumpedElements; j++)
                {
                    double diffDownwardTorsionalWave = (
                        (i == 0) ? 
                        (torsionalModel.DownwardTorsionalWave[i,j] - torsionalModel.DownwardTorsionalWaveLeftBoundary[j]) 
                        : 
                        (torsionalModel.DownwardTorsionalWave[i, j] - torsionalModel.DownwardTorsionalWave[i - 1, j])
                    );
                    double diffUpwardTorsionalWave = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (torsionalModel.UpwardTorsionalWaveRightBoundary[j] - torsionalModel.UpwardTorsionalWave[i, j]) 
                        : 
                        (torsionalModel.UpwardTorsionalWave[i + 1, j] - torsionalModel.UpwardTorsionalWave[i, j])
                    );
                    double diffDownwardAxialWave = (
                        (i == 0) ? 
                        (torsionalModel.DownwardAxialWave[i,j] - torsionalModel.DownwardAxialWaveLeftBoundary[j]) 
                        : 
                        (torsionalModel.DownwardAxialWave[i, j] - torsionalModel.DownwardAxialWave[i - 1, j])
                    );
                    double diffUpwardAxialWave = (
                        (i == simulationParameters.LumpedCells.DistributedToLumpedRatio - 1) ? 
                        (torsionalModel.UpwardAxialWaveRightBoundary[j] - torsionalModel.UpwardAxialWave[i, j]) 
                        : 
                        (torsionalModel.UpwardAxialWave[i + 1, j] - torsionalModel.UpwardAxialWave[i, j])
                    );
                
                    torsionalModel.DownwardTorsionalWave[i, j] -= c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * diffDownwardTorsionalWave;
                    torsionalModel.UpwardTorsionalWave[i, j]   += c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * diffUpwardTorsionalWave;
                    torsionalModel.DownwardAxialWave[i, j]     -= c1 * simulationParameters.Drillstring.AxialWaveSpeed * diffDownwardAxialWave;
                    torsionalModel.UpwardAxialWave[i, j]       += c1 * simulationParameters.Drillstring.AxialWaveSpeed * diffUpwardAxialWave;   
                }
            }
        
        }
    }
}