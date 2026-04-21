using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using System.Text.RegularExpressions;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;
using System.Reflection.Metadata.Ecma335;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class Solver
    {
        private State state;
        private Output output;
        private SimulationParameters parameters;
        private readonly Configuration configuration;
        //private Input simulationParameters.Input;
        private TopdriveController topdriveController;
        
        private LumpedElementModel drillStringModel;

        private ISolverODE<LumpedElementModel> solverODE; 

        public Solver(SimulationParameters simulationParameters, in DataModel.Configuration configuration)
        {
            this.parameters = simulationParameters;
            this.configuration = configuration;
        
            state = new State(in simulationParameters);                        
            output = new Output(in simulationParameters, in configuration);
            topdriveController = new TopdriveController(in configuration, in simulationParameters);
            IBitRock bitRockModel = configuration.BitRockModelEnum switch
            {
                BitRockModelEnum.Detournay => new Detournay(
                                                    simulationParameters.Drillstring,
                                                    configuration),
                BitRockModelEnum.MSE => new MSE(),
                _ => throw new ArgumentException($"Unknown BitRockModelEnum: {configuration.BitRockModelEnum}")
            };
            drillStringModel = new LumpedElementModel(in simulationParameters, in bitRockModel);
          
            //Create an instance of the selected ODE solver
            solverODE = simulationParameters.SolverType switch
            {
                SolverType.EulerMethod => solverODE = new EulerMethod(simulationParameters),
                SolverType.VerletMethod => solverODE = new VerletMethod(simulationParameters),
                _ => throw new ArgumentException($"Unknown SolverType: {simulationParameters.SolverType}")
            };
            
        }

        public (State, Output, Input) OuterStep(SetPoints setPoints)
        {
            parameters.TopDriveDrawwork.SurfaceRotation = setPoints.SurfaceRPM;
            parameters.TopDriveDrawwork.SurfaceAxialVelocity = setPoints.TopOfStringVelocity;
            parameters.Input.BottomExtraNormalForce = setPoints.BottomExtraSideForce;
            parameters.Input.DifferenceStaticKineticFriction = setPoints.DifferenceStaticKineticFriction;
            parameters.Input.StickingBoolean = setPoints.Sticking;

            if (setPoints.StribeckCriticalVelocity > 0)
            {
                parameters.Input.StribeckCriticalVelocity = setPoints.StribeckCriticalVelocity;
                parameters.Friction.Stribeck = 1.0 / parameters.Input.StribeckCriticalVelocity;
            }
            if (parameters.Friction.StaticFrictionCoefficient.Count == parameters.Friction.KinematicFrictionCoefficient.Count)
            {
                for (int i = 0; i < parameters.Friction.StaticFrictionCoefficient.Count; i++)
                {
                    parameters.Friction.StaticFrictionCoefficient[i] = parameters.Friction.KinematicFrictionCoefficient[i] + parameters.Input.DifferenceStaticKineticFriction;
                }
            }

            // Calculate Drawworks speed and surface rotational speed then apply controller 
            topdriveController.StateStep(state, in parameters);
            // Controller for top drive - calculate u.tau_Motor
            topdriveController.ControllerStep(state, in parameters);

            //UpdateDepths(); // BitDepth, HoleDepth, TopOfStringPosition, OnBottom

            if (parameters.MovingDrillstring)
            {
                // update trajectory parameters and buoyancy force calculations
                if (Math.Abs(state.BitDepth - state.PreviousCalculatedBitDepth) > 1.0)
                {
                    parameters.Trajectory.UpdateTrajectory(parameters.Drillstring);
                    parameters.Flow.UpdateBuoyancy(parameters.Trajectory, parameters.Drillstring, parameters.UseBuoyancyFactor);
                    parameters.Wellbore.UpdateWellbore(parameters.Drillstring);
                    state.PreviousCalculatedBitDepth = state.BitDepth;
                }

                // if the top most element has traveled more than the distance between
                // elements, we create a new lumped element and corresponding distributed section;
                // we also need to reconstruct the parameter and state vectors to include the new elements
                if (state.ZDisplacement[0] > parameters.Drillstring.ElementLength[0])
                {
                    AddNewLumpedElement();
                    parameters.Trajectory.UpdateTrajectory(parameters.Drillstring);
                    parameters.Flow.UpdateBuoyancy(parameters.Trajectory, parameters.Drillstring, parameters.UseBuoyancyFactor);
                    parameters.Wellbore.UpdateWellbore(parameters.Drillstring);
                    parameters.Drillstring.IndexSensor = parameters.Drillstring.IndexSensor + 1;
                }
            }
            InnerStep();
            output.UpdateSSI(state.Step * parameters.OuterLoopTimeStep);
            state.Step += 1;
            return (state, output, parameters.Input);
        }

        public void UpdateDepths()
        {
            state.BitDepth = state.BitDepth + output.BitVelocity * parameters.OuterLoopTimeStep;
            state.TopDrive.AxialPosition = state.TopDrive.AxialPosition - state.TopDrive.AxialVelocity * parameters.OuterLoopTimeStep;

            if (state.HoleDepth - state.BitDepth < 1E-3 && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth - state.BitDepth < 1E-3 || state.OnBottomStart > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }
         public void UpdateDepthInnerLoop()
        {
            state.BitDepth = parameters.Input.InitialBitDepth + state.ZDisplacement[state.ZDisplacement.Count - 1];
            state.TopDrive.AxialPosition = parameters.Input.InitialTopOfStringPosition + state.TopDrive.RelativeAxialPosition;

            if (state.HoleDepth < state.BitDepth && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth < state.BitDepth;// || state.onBottom_startIdx > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }

        public void InnerStep()
        {
            // Set these output values in the beginning to match matlab plotting
            output.TopDriveRotationInRPM = state.TopDrive.AngularVelocity;
            if (!parameters.UseMudMotor)
                output.BitRotationInRPM = state.AngularVelocity[state.AngularVelocity.Count - 1];
            else
                output.BitRotationInRPM = state.MudRotorAngularVelocity;

            // Wave equations are transformed into their Riemann invariants
            drillStringModel.PrepareModel(state, parameters);
            
            // Solve lumped and distributed equations
            for (int innerIterationNo = 0; innerIterationNo < parameters.InnerLoopIterations; innerIterationNo++)
            {                
                // Update bit depth and top of string position based on current velocities for use in the bit-rock interaction model and top drive model within the inner loop
                UpdateDepthInnerLoop();                                                                               
                // Assuming the top drive is represented by the first element in the state vector for angular velocities
                //state.TopDrive.AngularVelocity = state.AngularVelocity[0]; 
                // Calculate lateral accelerations                    
                output.SimulationHealthy = solverODE.IntegrationStep(state, drillStringModel, in parameters);
                bool sleeveHealth = solverODE.IntegrationSleeve(state, drillStringModel, in parameters);
                bool topDriveHealth = solverODE.IntegrationSurfacePosition(state, drillStringModel, in parameters);
                
                //Abort simulation in case of divergence
                if (!output.SimulationHealthy || !sleeveHealth || !topDriveHealth)
                {                    
                    return;
                }
                // Mud motor
                if (parameters.UseMudMotor)
                {
                    state.MudStatorAngularVelocity = state.WhirlVelocity[state.WhirlVelocity.Count - 1];
                    state.MudRotorAngularVelocity = state.MudRotorAngularVelocity + parameters.InnerLoopTimeStep / (parameters.MudMotor.I_rotor + parameters.MudMotor.M_rotor * Math.Pow(parameters.MudMotor.delta_rotor, 2) * Math.Pow(parameters.MudMotor.N_rotor, 2)) *
                        (state.AngularAcceleration[state.WhirlVelocity.Count - 1] * parameters.MudMotor.M_rotor * Math.Pow(parameters.MudMotor.delta_rotor, 2) * parameters.MudMotor.N_stator * parameters.MudMotor.N_rotor + state.MudTorque - state.TorqueOnBit);
                }
            }            
            // Bending moments
            drillStringModel.UpdateBendingMoments(state, parameters);
            // Update output based on new state values
            output.UpdateOutput(state, parameters);
            
        }

        public void AddNewLumpedElement() // TODO sjekk
        {
            parameters.AddNewElement();

            state.AddNewLumpedElement();

            solverODE.AddNewLumpedElement();        

            //axialTorsionalModel = new AxialTorsionalModel(state, simulationParameters, axialTorsionalModel);            
            //lateralModel = new LateralModel(simulationParameters, state);
        }
    }
}
