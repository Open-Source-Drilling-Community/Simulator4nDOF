using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
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
        private double topDriveRotationAngleMinus1;
        
        private double timeStepSquared;
        private double timeStep;       

        private Vector<double> sleeveAngularDisplacement;                        
        private Vector<double> angularDisplacement;                
        private Vector<double> xDisplacement;
        private Vector<double> yDisplacement;
        private Vector<double> zDisplacement;
        private double topOfStringRelativeAxialPosition;
        private double topDriveRotationAngle;
        
        
        public Vector<double> AngularVelocity;        
        public Vector<double> xVelocity;                
        public Vector<double> yVelocity;
        public Vector<double> zVelocity;
        
        

        public VerletMethod(SimulationParameters parameters)
        {
            sleeveAngularDisplacement = Vector<double>.Build.Dense(parameters.NumberOfNodes);            
            angularDisplacement = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);
            xDisplacement = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            yDisplacement = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            zDisplacement = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);
            
            AngularVelocity = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);
            xVelocity = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            yVelocity = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            zVelocity = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);                
            
            sleeveAngularDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            angularDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);
            xDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            yDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfNodes);
            zDisplacementMinus1 = Vector<double>.Build.Dense(parameters.NumberOfAxialNodes);
            
            timeStep = parameters.InnerLoopTimeStep;
            timeStepSquared = parameters.InnerLoopTimeStep * parameters.InnerLoopTimeStep;


            topOfStringRelativeAxialPosition = 0.0;
            topOfStringRelativeAxialPositionMinus1 = 0.0;
            
        
        }    
        public bool SimulationDivergedCheck(in State state, in int i)
        {         
            return double.IsNaN(state.ZVelocity[i]) || double.IsNaN(state.AngularAcceleration[i]);
        }
        public bool LateralSimulationDivergedCheck(in State state, in int i)
        {         
            return double.IsNaN(state.XVelocity[i]) || double.IsNaN(state.YVelocity[i]);
        }
        
        public void InitializeVerletMethod(in State state)
        {
            for (int i = 0; i < state.ZDisplacement.Count; i++)
            {
                angularDisplacementMinus1[i] = state.AngularDisplacement[i] - state.AngularVelocity[i] * timeStep + 0.5 * timeStepSquared * state.AngularAcceleration[i];
                zDisplacementMinus1[i] = state.ZDisplacement[i] - state.ZVelocity[i] * timeStep + 0.5 * timeStepSquared * state.ZAcceleration[i];
                angularDisplacement[i] = state.AngularDisplacement[i];
                zDisplacement[i] = state.ZDisplacement[i];  
            }
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                xDisplacementMinus1[i] = state.XDisplacement[i] - state.XVelocity[i] * timeStep + 0.5 * timeStepSquared * state.XAcceleration[i];
                yDisplacementMinus1[i] = state.YDisplacement[i] - state.YVelocity[i] * timeStep + 0.5 * timeStepSquared * state.YAcceleration[i];
                xDisplacement[i] = state.XDisplacement[i];
                yDisplacement[i] = state.YDisplacement[i];
            }
            firstStep = false;
        }

        public bool IntegrationStep(State state, LumpedElementModel drillStringModel, in SimulationParameters simulationParameters)
        {               
            // Use the lateral model instance to estimate the accelerations
            drillStringModel.CalculateAccelerations(state, simulationParameters);
            timeStepSquared = timeStep * timeStep;            
            double inverseTwoTimeStep = 1.0 / (2 * timeStep);
            // If it is the first step, initialize the minus one values                        
            if (firstStep)
            {
                InitializeVerletMethod(in state);                             
            }
            //Integrate time steps using Verlet method - axial-torsional nodes
            for (int i = 0; i < state.ZDisplacement.Count; i++)
            {
                //Displacements  
                state.AngularDisplacement[i] = 2 * angularDisplacement[i] - angularDisplacementMinus1[i] + timeStepSquared * state.AngularAcceleration[i];
                state.ZDisplacement[i] = 2 * zDisplacement[i] - zDisplacementMinus1[i] + timeStepSquared * state.ZAcceleration[i];
                // Velocities            
                state.AngularVelocity[i] = (state.AngularDisplacement[i] - angularDisplacementMinus1[i]) * inverseTwoTimeStep;
                state.ZVelocity[i] = (state.ZDisplacement[i] - zDisplacementMinus1[i]) * inverseTwoTimeStep;
                //Rollover data for next iteration                
                angularDisplacementMinus1[i] = angularDisplacement[i];
                zDisplacementMinus1[i] = zDisplacement[i];
                angularDisplacement[i] = state.AngularDisplacement[i];
                zDisplacement[i] = state.ZDisplacement[i];
                //Check if the simulation diverged
                if (SimulationDivergedCheck(in state, in i))
                {
                    return false;
                }        
            }                  
            //Integrate time steps using Verlet method - lateral nodes
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                //Displacements  
                state.XDisplacement[i] = 2 * xDisplacement[i] - xDisplacementMinus1[i] + timeStepSquared * state.XAcceleration[i];
                state.YDisplacement[i] = 2 * yDisplacement[i] - yDisplacementMinus1[i] + timeStepSquared * state.YAcceleration[i];
                // Velocities            
                state.XVelocity[i] = (state.XDisplacement[i] - xDisplacementMinus1[i]) * inverseTwoTimeStep;
                state.YVelocity[i] = (state.YDisplacement[i] - yDisplacementMinus1[i]) * inverseTwoTimeStep;
                //Rollover data for next iteration                
                xDisplacementMinus1[i] = xDisplacement[i];
                yDisplacementMinus1[i] = yDisplacement[i];
                xDisplacement[i] = state.XDisplacement[i];
                yDisplacement[i] = state.YDisplacement[i];            
                //Check if the simulation diverged
                if (LateralSimulationDivergedCheck(in state, in i))
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
                topOfStringRelativeAxialPosition = state.TopDrive.RelativeAxialPosition;
                topOfStringRelativeAxialPositionMinus1 = topOfStringRelativeAxialPosition - state.TopDrive.AxialVelocity * timeStep;            

                topDriveRotationAngle = state.TopDrive.AngularDisplacement;
                topDriveRotationAngleMinus1 = topDriveRotationAngle - state.TopDrive.AngularVelocity * timeStep + 0.5 * timeStepSquared * state.TopDrive.AngularAcceleration;            
                
                firstSurfaceStep = false;
            
            }
            // Integrate the top of string position and rotation angle
            state.TopDrive.RelativeAxialPosition = topOfStringRelativeAxialPositionMinus1 + 2 * timeStep * state.TopDrive.AxialVelocity;  
            state.TopDrive.AngularDisplacement = 2 * topDriveRotationAngle - topDriveRotationAngleMinus1 + timeStepSquared * state.TopDrive.AngularAcceleration;
            state.TopDrive.AngularVelocity = 0.5 * (state.TopDrive.AngularDisplacement - topDriveRotationAngleMinus1) / timeStep;
            // Rollover top of string relative axial position for next iteration
            topOfStringRelativeAxialPositionMinus1 = topOfStringRelativeAxialPosition;
            topDriveRotationAngleMinus1 = topDriveRotationAngle;
            // Rollover top of string relative axial position for next iteration
            topOfStringRelativeAxialPosition = state.TopDrive.RelativeAxialPosition;
            topDriveRotationAngle = state.TopDrive.AngularDisplacement;
            // Return true is simulation is healthy
            return !double.IsNaN(state.TopDrive.RelativeAxialPosition);
        }

        public void AddNewLumpedElement(int lateralNodesAdded, double addedLength)
        {
            // The sleeve history is indexed by sleeve, and the number of sleeves does not change
            // Axial-torsional nodes
            angularDisplacementMinus1 = ExtendVectorStart(angularDisplacementMinus1[0], angularDisplacementMinus1);
            zDisplacementMinus1 = ExtendVectorStart(zDisplacementMinus1[0], zDisplacementMinus1);
            angularDisplacement = ExtendVectorStart(angularDisplacement[0], angularDisplacement);
            zDisplacement = ExtendVectorStart(zDisplacement[0], zDisplacement);
            AngularVelocity = ExtendVectorStart(AngularVelocity[0], AngularVelocity);
            zVelocity = ExtendVectorStart(zVelocity[0], zVelocity);
            // The axial reference of every node moves down by the new element length
            for (int i = 0; i < zDisplacement.Count; i++)
            {
                zDisplacement[i] -= addedLength;
                zDisplacementMinus1[i] -= addedLength;
            }
            topOfStringRelativeAxialPosition -= addedLength;
            topOfStringRelativeAxialPositionMinus1 -= addedLength;
            // Lateral nodes
            xDisplacementMinus1 = ExtendVectorStart(xDisplacementMinus1[0], xDisplacementMinus1, lateralNodesAdded);
            yDisplacementMinus1 = ExtendVectorStart(yDisplacementMinus1[0], yDisplacementMinus1, lateralNodesAdded);
            xDisplacement = ExtendVectorStart(xDisplacement[0], xDisplacement, lateralNodesAdded);
            yDisplacement = ExtendVectorStart(yDisplacement[0], yDisplacement, lateralNodesAdded);
            xVelocity = ExtendVectorStart(xVelocity[0], xVelocity, lateralNodesAdded);
            yVelocity = ExtendVectorStart(yVelocity[0], yVelocity, lateralNodesAdded);
        }
    }            
}

