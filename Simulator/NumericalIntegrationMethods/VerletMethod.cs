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
    public class VerletMethod : ISolverODE<LumpedElementModel>
    {     
        private bool firstStep = true;
        private bool firstSurfaceStep = true;
        private bool firstSleeveStep = true;
        private Vector<double> sleeveAngularDisplacementMinus1;        
        private Vector<double> angularDisplacementMinus1;
        private Vector<double> xDisplacementMinus1;
        private Vector<double> yDisplacementMinus1;
        private Vector<double> zDisplacementMinus1;
        private double topOfStringRelativeAxialPositionMinus1;
        private double timeStepSquared;
        private double timeStep;       

        private Vector<double> sleeveAngularDisplacement;                        
        private Vector<double> angularDisplacement;                
        private Vector<double> xDisplacement;
        private Vector<double> yDisplacement;
        private Vector<double> zDisplacement;
        private double topOfStringRelativeAxialPosition;
        
        public Vector<double> AngularVelocity;        
        public Vector<double> xVelocity;                
        public Vector<double> yVelocity;
        public Vector<double> zVelocity;
        
        

        public VerletMethod(SimulationParameters parameters)
        {
            sleeveAngularDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);            
            angularDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);
            xDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);
            yDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);
            zDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);
            
            AngularVelocity = Vector<double>.Build.Dense(parameters.NumberOfElements);
            xVelocity = Vector<double>.Build.Dense(parameters.NumberOfElements);
            yVelocity = Vector<double>.Build.Dense(parameters.NumberOfElements);
            zVelocity = Vector<double>.Build.Dense(parameters.NumberOfElements);                
            
            sleeveAngularDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfElements);
            angularDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfElements);
            xDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfElements);
            yDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfElements);
            zDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfElements);
            
            timeStep = parameters.InnerLoopTimeStep;
            timeStepSquared = parameters.InnerLoopTimeStep * parameters.InnerLoopTimeStep;


            topOfStringRelativeAxialPosition = 0.0;
            topOfStringRelativeAxialPositionMinus1 = 0.0;
            
        
        }    
        public bool SimulationDivergedCheck(in State state, in int i)
        {         
            return double.IsNaN(state.XVelocity[i]) || double.IsNaN(state.YVelocity[i]) || double.IsNaN(state.ZVelocity[i]) || double.IsNaN(state.AngularAcceleration[i]);
        }
        
        public void InitializeVerletMethod(State state, in SimulationParameters simulationParameters)
        {

            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                angularDisplacementMinus1[i] = state.AngularDisplacement[i] - state.AngularVelocity[i] * timeStep + 0.5 * timeStepSquared * state.AngularAcceleration[i];
                xDisplacementMinus1[i] = state.XDisplacement[i] - state.XVelocity[i] * timeStep + 0.5 * timeStepSquared * state.XAcceleration[i];
                yDisplacementMinus1[i] = state.YDisplacement[i] - state.YVelocity[i] * timeStep + 0.5 * timeStepSquared * state.YAcceleration[i];
                zDisplacementMinus1[i] = state.ZDisplacement[i] - state.ZVelocity[i] * timeStep + 0.5 * timeStepSquared * state.ZAcceleration[i];
                

                angularDisplacement[i] = state.AngularDisplacement[i];
                xDisplacement[i] = state.XDisplacement[i];
                yDisplacement[i] = state.YDisplacement[i];
                zDisplacement[i] = state.ZDisplacement[i];  
            }
           
            firstStep = false;
        }

        public bool IntegrationStep(State state, LumpedElementModel drillStringModel, in SimulationParameters simulationParameters)
        {               
            // Use the lateral model instance to estimate the accelerations
            drillStringModel.CalculateAccelerations(state, simulationParameters);
            timeStepSquared = timeStep * timeStep;            
            int n = state.SleeveAngularDisplacement.Count;
            // If it is the first step, initialize the minus one values                        
            if (firstStep)
            {
                InitializeVerletMethod(state, in simulationParameters);                             
            }
            //Integrate time steps using Verlet method
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                //Displacements  
                state.AngularDisplacement[i] = 2 * angularDisplacement[i] - angularDisplacementMinus1[i] + timeStepSquared * state.AngularAcceleration[i];
                state.XDisplacement[i] = 2 * xDisplacement[i] - xDisplacementMinus1[i] + timeStepSquared * state.XAcceleration[i];
                state.YDisplacement[i] = 2 * yDisplacement[i] - yDisplacementMinus1[i] + timeStepSquared * state.YAcceleration[i];
                state.ZDisplacement[i] = 2 * zDisplacement[i] - zDisplacementMinus1[i] + timeStepSquared * state.ZAcceleration[i];

                // Velocities            
                state.AngularVelocity[i] = (state.AngularDisplacement[i] - angularDisplacementMinus1[i]) / (2 * timeStep);
                state.XVelocity[i] = (state.XDisplacement[i] - xDisplacementMinus1[i]) / (2 * timeStep);
                state.YVelocity[i] = (state.YDisplacement[i] - yDisplacementMinus1[i]) / (2 * timeStep);
                state.ZVelocity[i] = (state.ZDisplacement[i] - zDisplacementMinus1[i]) / (2 * timeStep);
                
                // Verlet Method for velocity update: v(t+dt) = v(t) + 0.5*dt*(a(t) + a(t+dt))

                //state.ZVelocity[i] = ZVelocityMinus1[i] + 0.5 * timeStep * (ZAcceleration[i] + state.ZAcceleration[i]);
                //Rollover data for next iteration                
                angularDisplacementMinus1[i] = angularDisplacement[i];
                xDisplacementMinus1[i] = xDisplacement[i];
                yDisplacementMinus1[i] = yDisplacement[i];
                zDisplacementMinus1[i] = zDisplacement[i];
                
                angularDisplacement[i] = state.AngularDisplacement[i];
                xDisplacement[i] = state.XDisplacement[i];
                yDisplacement[i] = state.YDisplacement[i];            
                zDisplacement[i] = state.ZDisplacement[i];
                
                //Check if the simulation diverged
                if (SimulationDivergedCheck(in state, in i))
                {
                    return false;
                }        
            }                  
            return true;       
        }
        public bool IntegrationSleeve(State state, LumpedElementModel model, in SimulationParameters simulationParameters)
        {
            if (firstSleeveStep)
            {
                for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
                {
                    sleeveAngularDisplacementMinus1[i] = state.SleeveAngularDisplacement[i] - state.SleeveAngularVelocity[i] * timeStep + 0.5 * timeStepSquared * state.SleeveAngularAcceleration[i];
                    sleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];
                }
                firstSleeveStep = false;
            }
            for (int i = 0; i < state.SleeveAngularDisplacement.Count; i++)
            {
                state.SleeveAngularDisplacement[i] = 2 * sleeveAngularDisplacement[i] - sleeveAngularDisplacementMinus1[i] + timeStepSquared * state.SleeveAngularAcceleration[i];
                state.SleeveAngularVelocity[i] = (state.SleeveAngularDisplacement[i] - sleeveAngularDisplacementMinus1[i]) / (2 * timeStep);       
                //Rollover data for next iteration 
                sleeveAngularDisplacementMinus1[i] = sleeveAngularDisplacement[i];
                sleeveAngularDisplacement[i] = state.SleeveAngularDisplacement[i];                         
            }     
            return true;
        }

        public bool IntegrationSurfacePosition(State state, LumpedElementModel model, in SimulationParameters simulationParameters)
        {

            //Initialize properly
            if (firstSurfaceStep)
            {
                topOfStringRelativeAxialPosition = state.TopOfStringRelativeAxialPosition;
                topOfStringRelativeAxialPositionMinus1 = topOfStringRelativeAxialPosition - state.TopDrive.CalculateSurfaceAxialVelocity * timeStep;            
                firstSurfaceStep = false;
            }
            // Integrate the top of string position
            state.TopOfStringRelativeAxialPosition = topOfStringRelativeAxialPositionMinus1 + timeStep * state.TopDrive.CalculateSurfaceAxialVelocity;            
            // Rollover top of string relative axial position for next iteration
            topOfStringRelativeAxialPositionMinus1 = topOfStringRelativeAxialPosition;
            // Rollover top of string relative axial position for next iteration
            topOfStringRelativeAxialPosition = state.TopOfStringRelativeAxialPosition;
            // Return true is simulation is healthy
            return !double.IsNaN(state.TopOfStringRelativeAxialPosition);
        }

        public void AddNewLumpedElement()
        {
            sleeveAngularDisplacementMinus1 = ExtendVectorStart(sleeveAngularDisplacementMinus1[0], sleeveAngularDisplacementMinus1);
            angularDisplacementMinus1 = ExtendVectorStart(angularDisplacementMinus1[0], angularDisplacementMinus1);
            xDisplacementMinus1 = ExtendVectorStart(xDisplacementMinus1[0], xDisplacementMinus1);
            yDisplacementMinus1 = ExtendVectorStart(yDisplacementMinus1[0], yDisplacementMinus1);
            zDisplacementMinus1 = ExtendVectorStart(zDisplacementMinus1[0], zDisplacementMinus1);
       
            //ZVelocityMinus1 = ExtendVectorStart(ZVelocityMinus1[0], ZVelocityMinus1);
            angularDisplacement = ExtendVectorStart(angularDisplacement[0], angularDisplacement);
            xDisplacement = ExtendVectorStart(xDisplacement[0], xDisplacement);
            yDisplacement = ExtendVectorStart(yDisplacement[0], yDisplacement);
            zDisplacement = ExtendVectorStart(zDisplacement[0], zDisplacement);
            
            AngularVelocity = ExtendVectorStart(AngularVelocity[0], AngularVelocity);
            xVelocity = ExtendVectorStart(xVelocity[0], xVelocity);
            yVelocity = ExtendVectorStart(yVelocity[0], yVelocity);
            zVelocity = ExtendVectorStart(zVelocity[0], zVelocity);
        }
    }
}