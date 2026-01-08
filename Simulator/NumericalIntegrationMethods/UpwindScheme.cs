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
            torsionalModel.DownwardTorsionalWave = torsionalModel.DownwardTorsionalWave - c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(torsionalModel.DownwardTorsionalWaveStackedWithLeftBoundary);
            torsionalModel.UpwardTorsionalWave   = torsionalModel.UpwardTorsionalWave   + c1 * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(torsionalModel.UpwardTorsionalWaveStackedWithLeftBoundary);
            torsionalModel.DownwardAxialWave     = torsionalModel.DownwardAxialWave     - c1 * simulationParameters.Drillstring.AxialWaveSpeed *     DiffRows(torsionalModel.DownwardAxialWaveStackedWithRightBoundary);
            torsionalModel.UpwardAxialWave       = torsionalModel.UpwardAxialWave       + c1 * simulationParameters.Drillstring.AxialWaveSpeed *     DiffRows(torsionalModel.UpwardAxialWaveStackedWithRightBoundary);         
        }
    }
}