using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public class VerletMethod : ISolverODE<LateralModel>
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
        public Vector<double> AxialAcceleration;
        
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
            AxialAcceleration = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);

            SleeveAngularDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AngularDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            XDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            YDisplacementMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            AxialVelocityMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
        }    

        public void InitializeVerletMethod(State state, in SimulationParameters simulationParameters)
        {

            timeStepSquared = simulationParameters.InnerLoopTimeStep * simulationParameters.InnerLoopTimeStep;
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                AngularDisplacementMinus1[i] = state.AngularDisplacement[i] - state.AngularVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.AngularAcceleration[i];
                XDisplacementMinus1[i] = state.XDisplacement[i] - state.XVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.XAcceleration[i];
                YDisplacementMinus1[i] = state.YDisplacement[i] - state.YVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.YAcceleration[i];
                AxialVelocityMinus1[i] = state.AxialVelocity[i] - state.AxialAcceleration[i] * simulationParameters.InnerLoopTimeStep;

                AngularDisplacement[i] = state.AngularDisplacement[i];
                XDisplacement[i] = state.XDisplacement[i];
                YDisplacement[i] = state.YDisplacement[i];
                AxialVelocity[i] = state.AxialVelocity[i];

                AxialAcceleration[i] = state.AxialAcceleration[i];            
            }
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                SleeveAngularDisplacementMinus1[i] = state.SleeveAngularDisplacement[i] - state.SleeveAngularVelocity[i] * simulationParameters.InnerLoopTimeStep + 0.5 * timeStepSquared * state.SleeveAngularAcceleration[i];
                SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];
            }
            FirstStep = false;
        }

        public bool IntegrationStep(State state, LateralModel lateralModel, in SimulationParameters simulationParameters)
        {               
            // Use the lateral model instance to estimate the accelerations
            lateralModel.CalculateAccelerations(state, simulationParameters);
            timeStepSquared = simulationParameters.InnerLoopTimeStep * simulationParameters.InnerLoopTimeStep;            
            int n = state.SleeveAngularDisplacement.Count;
            // If it is the first step, initialize the minus one values                        
            if (FirstStep)
            {
                InitializeVerletMethod(state, in simulationParameters);                             
            }
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
                // Verlet Method for velocity update: v(t+dt) = v(t) + 0.5*dt*(a(t) + a(t+dt))
                state.AxialVelocity[i] = AxialVelocityMinus1[i] + 0.5 * simulationParameters.InnerLoopTimeStep * (AxialAcceleration[i] + state.AxialAcceleration[i]);
                //Rollover data for next iteration                
                AngularDisplacementMinus1[i] = AngularDisplacement[i];
                XDisplacementMinus1[i] = XDisplacement[i];
                YDisplacementMinus1[i] = YDisplacement[i];
                AxialVelocityMinus1[i] = AxialVelocity[i];
                
                AngularDisplacement[i] = state.AngularDisplacement[i];
                XDisplacement[i] = state.XDisplacement[i];
                YDisplacement[i] = state.YDisplacement[i];            
                AxialVelocity[i] = state.AxialVelocity[i]; // Rollover axial velocity for next iteration
                AxialAcceleration[i] = state.AxialAcceleration[i]; // Rollover axial acceleration for next iteration
                //Check if the simulation diverged
                if (double.IsNaN(state.XVelocity[i]) || double.IsNaN(state.YVelocity[i]) || double.IsNaN(state.AxialVelocity[i]) || double.IsNaN(state.AngularAcceleration[i]));
                {
                    return false;
                }        
            }
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                state.SleeveAngularDisplacement[i] = 2 * SleeveAngularDisplacement[i] - SleeveAngularDisplacementMinus1[i] + timeStepSquared * state.SleeveAngularAcceleration[i];
                state.SleeveAngularVelocity[i] = (state.SleeveAngularDisplacement[i] - SleeveAngularDisplacementMinus1[i]) / (2 * simulationParameters.InnerLoopTimeStep);       
                //Rollover data for next iteration 
                SleeveAngularDisplacementMinus1[i] = SleeveAngularDisplacement[i];
                SleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];                         
            }         
            return true;       
        }
        public void AddNewLumpedElement()
        {
            SleeveAngularDisplacementMinus1 = ExtendVectorStart(SleeveAngularDisplacementMinus1[0], SleeveAngularDisplacementMinus1);
            AngularDisplacementMinus1 = ExtendVectorStart(AngularDisplacementMinus1[0], AngularDisplacementMinus1);
            XDisplacementMinus1 = ExtendVectorStart(XDisplacementMinus1[0], XDisplacementMinus1);
            YDisplacementMinus1 = ExtendVectorStart(YDisplacementMinus1[0], YDisplacementMinus1);
            AxialVelocityMinus1 = ExtendVectorStart(AxialVelocityMinus1[0], AxialVelocityMinus1);
            AngularDisplacement = ExtendVectorStart(AngularDisplacement[0], AngularDisplacement);
            XDisplacement = ExtendVectorStart(XDisplacement[0], XDisplacement);
            YDisplacement = ExtendVectorStart(YDisplacement[0], YDisplacement);
            AngularVelocity = ExtendVectorStart(AngularVelocity[0], AngularVelocity);
            XVelocity = ExtendVectorStart(XVelocity[0], XVelocity);
            YVelocity = ExtendVectorStart(YVelocity[0], YVelocity);
            AxialVelocity = ExtendVectorStart(AxialVelocity[0], AxialVelocity);
        }
    }
}