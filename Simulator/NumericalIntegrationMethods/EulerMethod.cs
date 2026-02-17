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
    public class EulerMethod : ISolverODE
    {     
        public void IntegrationStep(State state, SimulationParameters simulationParameters)
        {   
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                //Angular DoF
                state.AngularDisplacement[i] = state.AngularDisplacement[i] + state.AngularVelocity[i] * simulationParameters.InnerLoopTimeStep; 
                state.AngularVelocity[i]     = state.AngularVelocity[i]     + state.AngularAcceleration[i] * simulationParameters.InnerLoopTimeStep;           
                //Axial DoF
                state.AxialVelocity[i] = state.AxialVelocity[i] + state.AxialAcceleration[i] * simulationParameters.InnerLoopTimeStep;
                //X DoF
                state.XDisplacement[i] = state.XDisplacement[i] + state.XVelocity[i] * simulationParameters.InnerLoopTimeStep;
                state.XVelocity[i]     = state.XVelocity[i] + state.XAcceleration[i] * simulationParameters.InnerLoopTimeStep;
                //Y DoF
                state.YDisplacement[i] = state.YDisplacement[i] + state.YVelocity[i] * simulationParameters.InnerLoopTimeStep;
                state.YVelocity[i]     = state.YVelocity[i] + state.YAcceleration[i] * simulationParameters.InnerLoopTimeStep;    
            }

            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                //Sleeve DoF
                state.SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i] + state.SleeveAngularVelocity[i] * simulationParameters.InnerLoopTimeStep;
                state.SleeveAngularVelocity[i]     = state.SleeveAngularVelocity[i] + state.SleeveAngularAcceleration[i] * simulationParameters.InnerLoopTimeStep;
            }
        }
        public void AddNewLumpedElement()
        {

        }
    }
}