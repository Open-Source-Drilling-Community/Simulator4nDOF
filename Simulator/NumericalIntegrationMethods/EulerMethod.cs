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
            //Angular DoF
            state.AngularDisplacement = state.AngularDisplacement + state.AngularVelocity * simulationParameters.InnerLoopTimeStep;
            state.AngularVelocity     = state.AngularVelocity + state.AngularAcceleration * simulationParameters.InnerLoopTimeStep;
            //Sleeve DoF
            state.SleeveAngularDisplacement = state.SleeveAngularDisplacement + state.SleeveAngularVelocity * simulationParameters.InnerLoopTimeStep;
            state.SleeveAngularVelocity     = state.SleeveAngularVelocity + state.SleeveAngularAcceleration * simulationParameters.InnerLoopTimeStep;
            //Axial DoF
            state.AxialVelocity = state.AxialVelocity + state.AxialAcceleration * simulationParameters.InnerLoopTimeStep;
            //X DoF
            state.XDisplacement = state.XDisplacement + state.XVelocity * simulationParameters.InnerLoopTimeStep;
            state.XVelocity     = state.XVelocity + state.XAcceleration * simulationParameters.InnerLoopTimeStep;
            //Y DoF
            state.YDisplacement = state.YDisplacement + state.YVelocity * simulationParameters.InnerLoopTimeStep;
            state.YVelocity     = state.YVelocity + state.YAcceleration * simulationParameters.InnerLoopTimeStep;            
        }
        public void AddNewLumpedElement()
        {

        }
    }
}