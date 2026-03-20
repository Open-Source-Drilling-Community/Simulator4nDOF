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
    public class UpwindScheme : ISolverODE<WaveModel>
    {     
        //Integration constant
        private double integrationConstant;
   

        public UpwindScheme(in  WaveModel waveModel, in SimulationParameters simulationParameters)
        {
            integrationConstant = waveModel.WaveSpeed * simulationParameters.InnerLoopTimeStep / waveModel.ElementLength;       
        }
        public void AddNewLumpedElement(){}
        public bool IntegrationStep(State state, WaveModel waveModel, in SimulationParameters simulationParameters)
        {   
            // Use the torsional model instance to estimate the accelerations
            waveModel.CalculateAccelerations(state, simulationParameters);
            //Update separately to avoid overwritting
            for (int i = 0; i < simulationParameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {                                 
                waveModel.DownwardWave[i] -= integrationConstant * simulationParameters.Drillstring.TorsionalWaveSpeed * waveModel.DiffDownwardWave[i];
                waveModel.UpwardWave[i]   += integrationConstant * simulationParameters.Drillstring.TorsionalWaveSpeed * waveModel.DiffUpwardWave[i];                    
                if (
                        double.IsNaN(waveModel.DownwardWave[i]) ||
                        double.IsNaN(waveModel.UpwardWave[i]) 
                    )
                {
                    return false;
                }                               
            }
            return true;
        }    
     
    }
}