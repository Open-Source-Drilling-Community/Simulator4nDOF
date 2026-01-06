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
        }    

        public void InitializeVerletMethod(State state, SimulationParameters simulationParameters)
        {

            timeStepSquared = simulationParameters.InnerLoopTimeStep * simulationParameters.InnerLoopTimeStep;
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                AngularDisplacementMinus1[i] = state.AngularDisplacement[i] - state.AngularVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.AngularAcceleration[i];
                XDisplacementMinus1[i] = state.XDisplacement[i] - state.XVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.XAcceleration[i];
                YDisplacementMinus1[i] = state.YDisplacement[i] - state.YVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.YAcceleration[i];
                AxialVelocityMinus1[i] = state.AxialVelocity[i] - 2 * state.AxialAcceleration[i] * simulationParameters.InnerLoopTimeStep;

                AngularDisplacement[i] = state.AngularDisplacement[i];
                XDisplacement[i] = state.XDisplacement[i];
                YDisplacement[i] = state.YDisplacement[i];
                AxialVelocity[i] = state.AxialVelocity[i];
            }
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                SleeveAngularDisplacementMinus1[i] = state.SleeveAngularDisplacement[i] - state.SleeveAngularVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.SleeveAngularAcceleration[i];
                SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];
            }
            FirstStep = false;
        }

        public void IntegrationStep(State state, SimulationParameters simulationParameters)
        {               
            timeStepSquared = simulationParameters.InnerLoopTimeStep * simulationParameters.InnerLoopTimeStep;            
            int n = state.SleeveAngularDisplacement.Count;
            // If it is the first step, initialize the minus one values                        
            if (FirstStep)
            {
                InitializeVerletMethod(state, simulationParameters);                             
            }
            //Temporary variable for axial velocity for data rollover
            double axialVelocity;
            //Integrate time steps using Verlet method
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                //Displacements  
                state.AngularDisplacement[i] = 2 * AngularDisplacement[i] - AngularDisplacementMinus1[i] + timeStepSquared * state.AngularAcceleration[i];
                state.XDisplacement[i] = 2 * XDisplacement[i] - XDisplacementMinus1[i] + timeStepSquared * state.XAcceleration[i];
                state.YDisplacement[i] = 2 * YDisplacement[i] - YDisplacementMinus1[i] + timeStepSquared * state.YAcceleration[i];
                // Velocities            
                state.AngularVelocity[i] = (state.AngularDisplacement[i] - AngularDisplacementMinus1[i]) / (2 * simulationParameters.InnerLoopTimeStep);
                state.XVelocity[i] = (state.XDisplacement[i] - XDisplacementMinus1[i]) / (2 * simulationParameters.InnerLoopTimeStep);
                state.YVelocity[i] = (state.YDisplacement[i] - YDisplacementMinus1[i]) / (2 * simulationParameters.InnerLoopTimeStep);
                //Temporary allocate axial velocity for proper data rollover
                axialVelocity = state.AxialVelocity[i]; 
                // Central difference for axial velocity
                state.AxialVelocity[i] = AxialVelocityMinus1[i] + 2 * simulationParameters.InnerLoopTimeStep * state.AxialAcceleration[i];
                //Rollover data for next iteration                
                AngularDisplacementMinus1[i] = AngularDisplacement[i];
                XDisplacementMinus1[i] = XDisplacement[i];
                YDisplacementMinus1[i] = YDisplacement[i];
                AxialVelocityMinus1[i] = axialVelocity;
                AngularDisplacement[i] = state.AngularDisplacement[i];
                XDisplacement[i] = state.XDisplacement[i];
                YDisplacement[i] = state.YDisplacement[i];            
            }
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                state.SleeveAngularDisplacement[i] = 2 * SleeveAngularDisplacement[i] - SleeveAngularDisplacementMinus1[i] + timeStepSquared * state.SleeveAngularAcceleration[i];
                state.SleeveAngularVelocity[i] = (state.SleeveAngularDisplacement[i] - SleeveAngularDisplacementMinus1[i]) / (2 * simulationParameters.InnerLoopTimeStep);       
                //Rollover data for next iteration 
                SleeveAngularDisplacementMinus1[i] = SleeveAngularDisplacement[i];
                SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];                         
            }                        
        }
    }
}