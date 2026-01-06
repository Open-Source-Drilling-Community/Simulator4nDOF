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
        public static void IntegrationStep(AxialTorsionalModel model, SimulationParameters simulationParameters)
        {
            model.DownwardTorsionalWave = model.DownwardTorsionalWave - (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength) * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(model.DownwardTorsionalWaveStackedWithLeftBoundary);
            model.UpwardTorsionalWave = model.UpwardTorsionalWave + (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength) * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(model.UpwardTorsionalWaveStackedWithLeftBoundary);
            model.DownwardAxialWave = model.DownwardAxialWave - (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength) * simulationParameters.Drillstring.AxialWaveSpeed * DiffRows(model.DownwardAxialWaveStackedWithRightBoundary);
            model.UpwardAxialWave = model.UpwardAxialWave + (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.DistributedSectionLength) * simulationParameters.Drillstring.AxialWaveSpeed * DiffRows(model.UpwardAxialWaveStackedWithRightBoundary);         
        }
    }
}