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
    public class VerletMethod : ISolverODE
    {     
        public bool FirstStep = true;
        public Vector<double> SleeveAngularDisplacementMinus1;        
        public Vector<double> AngularDisplacementMinus1;
        public Vector<double> XDisplacementMinus1;
        public Vector<double> YDisplacementMinus1;

        public Vector<double> AxialVelocityMinus1;
        public Vector<double> SleeveAngularVelocityMinus1;
        public Vector<double> AngularVelocityMinus1;
        public Vector<double> XVelocityMinus1;
        public Vector<double> YVelocityMinus1; 
        private double timeStepSquared;       

        public Vector<double> SleeveAngularDisplacement;                        
        public Vector<double> AngularDisplacement;                
        public Vector<double> XDisplacement;
        public Vector<double> YDisplacement;
        public Vector<double> AngularVelocity;        
 
        public Vector<double> XVelocity;                
        public Vector<double> YVelocity;
        public Vector<double> AxialVelocity;
        
        public VerletMethod(SimulationParameters simulationParameters)
        {
            SleeveAngularDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);            
            AngularDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            XDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            YDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AngularVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            XVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            YVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AxialVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);                
        
            SleeveAngularDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AngularDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            XDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            YDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AxialVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            SleeveAngularVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AngularVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            XVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            YVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
        }    

        public void IntegrationStep(State state, SimulationParameters simulationParameters)
        {   
            timeStepSquared = simulationParameters.InnerLoopTimeStep * simulationParameters.InnerLoopTimeStep;
            //Displacements            
            state.SleeveAngularDisplacement = 2 * SleeveAngularDisplacement - SleeveAngularDisplacementMinus1  + timeStepSquared * state.SleeveAngularAcceleration ;
            state.AngularDisplacement = 2 * state.AngularDisplacement - AngularDisplacementMinus1 + timeStepSquared * state.AngularAcceleration;                                            
            state.XDisplacement = 2 * state.XDisplacement - XDisplacementMinus1 + timeStepSquared * state.XAcceleration;            
            state.YDisplacement = 2 * state.YDisplacement - YDisplacementMinus1 + timeStepSquared * state.YAcceleration;
            
            // Velocities
            state.SleeveAngularVelocity     = state.SleeveAngularVelocity + state.SleeveAngularAcceleration * simulationParameters.InnerLoopTimeStep;            
            state.AngularVelocity     = state.AngularVelocity + state.AngularAcceleration * simulationParameters.InnerLoopTimeStep;
            state.XVelocity     = state.XVelocity + state.XAcceleration * simulationParameters.InnerLoopTimeStep;
            state.YVelocity     = state.YVelocity + state.YAcceleration * simulationParameters.InnerLoopTimeStep;            
            state.AxialVelocity = state.AxialVelocity + state.AxialAcceleration * simulationParameters.InnerLoopTimeStep;

        }
    }
}