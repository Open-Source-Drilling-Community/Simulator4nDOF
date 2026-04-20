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
    public class EulerMethod : ISolverODE<LumpedElementModel>
    {     
        private double timeStep;
        public EulerMethod(in SimulationParameters simulationParameters)        
        {
            timeStep = simulationParameters.InnerLoopTimeStep;
        }
         public bool SimulationDivergedCheck(in State state, in int i)
        {         
            return double.IsNaN(state.XVelocity[i]) || double.IsNaN(state.YVelocity[i]) || double.IsNaN(state.ZVelocity[i]) || double.IsNaN(state.AngularAcceleration[i]);
        }
        public bool IntegrationStep(State state, LumpedElementModel drillStringModel, in SimulationParameters simulationParameters)
        {   
            // Use the lateral model instance to estimate the accelerations
            drillStringModel.CalculateAccelerations(state, simulationParameters);
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                //Angular DoF
                state.AngularDisplacement[i] = state.AngularDisplacement[i] + state.AngularVelocity[i] * timeStep; 
                state.AngularVelocity[i]     = state.AngularVelocity[i]     + state.AngularAcceleration[i] * timeStep;                           
                //X DoF
                state.XDisplacement[i] = state.XDisplacement[i] + state.XVelocity[i] * timeStep;
                state.XVelocity[i]     = state.XVelocity[i] + state.XAcceleration[i] * timeStep;
                //Y DoF
                state.YDisplacement[i] = state.YDisplacement[i] + state.YVelocity[i] * timeStep;
                state.YVelocity[i]     = state.YVelocity[i] + state.YAcceleration[i] * timeStep;    
                //Axial DoF
                state.ZDisplacement[i] = state.ZDisplacement[i] + state.ZVelocity[i] * timeStep;
                state.ZVelocity[i] = state.ZVelocity[i] + state.ZAcceleration[i] * timeStep;
                if (SimulationDivergedCheck(in state, in i))
                {
                    return false;
                }      
            }
            return true;
        }
        public bool IntegrationSleeve(State state, LumpedElementModel model, in SimulationParameters simulationParameters)
        {      
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                //Sleeve DoF
                state.SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i] + state.SleeveAngularVelocity[i] * timeStep;
                state.SleeveAngularVelocity[i]     = state.SleeveAngularVelocity[i] + state.SleeveAngularAcceleration[i] * timeStep;
                if ( double.IsNaN(state.SleeveAngularVelocity[i]) || double.IsNaN(state.SleeveAngularDisplacement[i]) )
                {
                    return false;
                }
            }  
            return true;
        }
        public bool IntegrationSurfacePosition(State state, LumpedElementModel model, in SimulationParameters simulationParameters)
        {
            //Calculate the top of string position based on the calculated speed        
            state.TopDrive.RelativeAxialPosition = state.TopDrive.RelativeAxialPosition + state.TopDrive.AxialVelocity * timeStep;     
            state.TopDrive.AngularDisplacement = state.TopDrive.AngularDisplacement + state.TopDrive.AngularVelocity * timeStep;     
            return !double.IsNaN(state.TopDrive.RelativeAxialPosition);
        }

        public void AddNewLumpedElement()
        {

        }
    }
}