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
            state.Step = state.Step + 1;
            return (state, output, parameters.Input);
        }

        public void UpdateDepths()
        {
            state.BitDepth = state.BitDepth + output.BitVelocity * parameters.OuterLoopTimeStep;
            state.TopOfStringPosition = state.TopOfStringPosition - state.TopDrive.CalculateSurfaceAxialVelocity * parameters.OuterLoopTimeStep;

            if (state.HoleDepth - state.BitDepth < 1E-3 && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth - state.BitDepth < 1E-3 || state.OnBottomStart > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }
         public void UpdateDepthInnerLoop()
        {
            state.BitDepth = state.BitDepth + state.BitVelocity * parameters.InnerLoopTimeStep;
            state.TopOfStringPosition = state.TopOfStringPosition - state.TopDrive.CalculateSurfaceAxialVelocity * parameters.InnerLoopTimeStep;

            if (state.HoleDepth < state.BitDepth && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth < state.BitDepth;// || state.onBottom_startIdx > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }


        public void CalculateInLocalFrame()
        {
            Matrix<double> SensorToPipeLocal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), parameters.Drillstring.SensorRadialDistance },
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { -Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle), 0, Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle), 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeLocalToPipeNonRotating = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(output.SensorAngularPosition), -Math.Sin(output.SensorAngularPosition), output.SensorBendingAngleY * Math.Cos(output.SensorAngularPosition) + output.SensorBendingAngleX * Math.Sin(output.SensorAngularPosition), 0 },
                        { Math.Sin(output.SensorAngularPosition), Math.Cos(output.SensorAngularPosition), output.SensorBendingAngleY * Math.Sin(output.SensorAngularPosition) - output.SensorBendingAngleX * Math.Cos(output.SensorAngularPosition), 0 },
                        { -output.SensorBendingAngleY, output.SensorBendingAngleX, 1, 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeNonRotatingToLateralRotationCenter = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { 1, 0, 0, output.SensorRadialPosition * Math.Cos(output.SensorWhirlAngle) },
                        { 0, 1, 0, output.SensorRadialPosition * Math.Sin(output.SensorWhirlAngle) },
                        { 0, 0, 1, output.SensorAxialDisplacement },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> LateralRotationCenterToGlobal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(output.SensorPipeAzimuthAt), Math.Cos(output.SensorPipeInclination) * Math.Sin(output.SensorPipeAzimuthAt), -Math.Sin(output.SensorPipeInclination) * Math.Sin(output.SensorPipeAzimuthAt), 0 },
                        { -Math.Sin(output.SensorPipeAzimuthAt), Math.Cos(output.SensorPipeInclination) * Math.Cos(output.SensorPipeAzimuthAt), -Math.Sin(output.SensorPipeInclination) * Math.Cos(output.SensorPipeAzimuthAt), 0 },
                        { 0, Math.Sin(output.SensorPipeInclination), Math.Cos(output.SensorPipeInclination), 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeLocalToSensor = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) },
                        { -Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) }
            });

            Matrix<double> PipeNonRotatingToPipeLocal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(output.SensorAngularPosition), Math.Sin(output.SensorAngularPosition), -output.SensorBendingAngleY },
                        { -Math.Sin(output.SensorAngularPosition), Math.Cos(output.SensorAngularPosition), output.SensorBendingAngleX },
                        { output.SensorBendingAngleY * Math.Cos(output.SensorAngularPosition) + output.SensorBendingAngleX * Math.Sin(output.SensorAngularPosition), output.SensorBendingAngleY * Math.Sin(output.SensorAngularPosition) - output.SensorBendingAngleX * Math.Cos(output.SensorAngularPosition), 1 }
            });

            Matrix<double> LateralRotationCenterToPipeNonRotating = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 }
            });

            Matrix<double> GlobalToLateralRotationCenter = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(output.SensorPipeAzimuthAt), -Math.Sin(output.SensorPipeAzimuthAt), 0 },
                        { Math.Cos(output.SensorPipeInclination) * Math.Sin(output.SensorPipeAzimuthAt), Math.Cos(output.SensorPipeInclination) * Math.Cos(output.SensorPipeAzimuthAt), Math.Sin(output.SensorPipeInclination) },
                        { -Math.Sin(output.SensorPipeInclination) * Math.Sin(output.SensorPipeAzimuthAt), -Math.Sin(output.SensorPipeInclination) * Math.Cos(output.SensorPipeAzimuthAt), Math.Cos(output.SensorPipeInclination) }
            });

            Matrix<double> PipeLocalToLateralRotationCenterSecondDerivative = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { -Math.Pow(output.SensorAngularVelocity, 2) * Math.Cos(output.SensorAngularPosition) - output.SensorAngularAcceleration * Math.Sin(output.SensorAngularPosition), Math.Pow(output.SensorAngularVelocity, 2) * Math.Sin(output.SensorAngularPosition) - output.SensorAngularAcceleration * Math.Cos(output.SensorAngularPosition), 0, (output.SensorRadialAcceleration - output.SensorRadialPosition * Math.Pow(output.SensorWhirlSpeed, 2)) * Math.Cos(output.SensorWhirlAngle) -
                        (2 * output.SensorRadialSpeed * output.SensorWhirlSpeed + output.SensorRadialPosition * output.SensorWhirlAcceleration) * Math.Sin(output.SensorWhirlAngle) },
                        { -Math.Pow(output.SensorAngularVelocity, 2) * Math.Sin(output.SensorAngularPosition) + output.SensorAngularAcceleration * Math.Cos(output.SensorAngularPosition), -Math.Pow(output.SensorAngularVelocity, 2) * Math.Cos(output.SensorAngularPosition) - output.SensorAngularAcceleration * Math.Sin(output.SensorAngularPosition), 0, (output.SensorRadialAcceleration - output.SensorRadialPosition * Math.Pow(output.SensorWhirlSpeed, 2)) * Math.Sin(output.SensorWhirlAngle) +
                        (2 * output.SensorRadialSpeed * output.SensorWhirlSpeed + output.SensorRadialPosition * output.SensorWhirlAcceleration) * Math.Cos(output.SensorWhirlAngle) },
                        { -output.SecondDerivativeSensorBendingAngleY, output.SecondDerivativeSensorBendingAngleX, 0, output.SensorAxialAcceleration },
                        { 0, 0, 0, 0 }
            });

            // Calculate acceleration in global frame
            var accelerationInGlobalFrameM = LateralRotationCenterToGlobal * PipeLocalToLateralRotationCenterSecondDerivative * SensorToPipeLocal * ToVector((parameters.Drillstring.SensorDisplacementsInLocalFrame.Append(1).ToArray()));
            var accelerationInGlobalFrame = accelerationInGlobalFrameM.SubVector(0, 3) - Vector<double>.Build.DenseOfArray(new double[] { 0, 0, Constants.GravitationalAcceleration });

            var accelerationInLocalFrame = PipeLocalToSensor * PipeNonRotatingToPipeLocal * LateralRotationCenterToPipeNonRotating * GlobalToLateralRotationCenter * accelerationInGlobalFrame;

            output.SensorRadialAccelerationLocalFrame = -accelerationInLocalFrame[0];
            output.SensorTangentialAccelerationLocalFrame = accelerationInLocalFrame[1];
            output.SensorAxialAccelerationLocalFrame = accelerationInLocalFrame[2];

            output.SensorBendingMomentX = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Sin((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * parameters.OuterLoopTimeStep);
            output.SensorBendingMomentY = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Cos((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * parameters.OuterLoopTimeStep);
        }

        // Suggestion for performance improvements after using the diagnostic tool in VS
        // Change to use multiply instead of pointwise power
        // Vector.Map is expensive - change to for loop and test
        // Matrix.Stack?
        // Use DiffRowsNew instead of DiffRows
        // Declare vectors and matrices side loop insteasd of every iteration in loop

        public void InnerStep()
        {
            // Set these output values in the beginning to match matlab plotting
            output.TopDriveRotationInRPM = state.TopDrive.TopDriveAngularVelocity;
            if (!parameters.UseMudMotor)
                output.BitRotationInRPM = state.AngularVelocity[state.AngularVelocity.Count - 1];
            else
                output.BitRotationInRPM = state.MudRotorAngularVelocity;

            double dtTemp = 1e-4;  // Needs to be calculated
            // Wave equations are transformed into their Riemann invariants
            drillStringModel.PrepareModel(state, parameters);
            // Solve lumped and distributed equations
            for (int innerIterationNo = 0; innerIterationNo < parameters.InnerLoopIterations; innerIterationNo++)
            {
                // Update bit depth and top of string position based on current velocities for use in the bit-rock interaction model and top drive model within the inner loop
                UpdateDepthInnerLoop();                                                                               
                // Assuming the top drive is represented by the first element in the state vector for angular velocities
                state.TopDrive.TopDriveAngularVelocity = state.AngularVelocity[0]; 
                // Calculate lateral accelerations                    
                output.SimulationHealthy = solverODE.IntegrationStep(state, drillStringModel, in parameters);
                output.SimulationHealthy = solverODE.IntegrationSleeve(state, drillStringModel, in parameters);
                output.SimulationHealthy = solverODE.IntegrationSurfacePosition(state, drillStringModel, in parameters);
                
                //Abort simulation in case of divergence
                if (!output.SimulationHealthy)
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
           
            double Theta_x = 0.0;
            double Theta_y = 0.0;
            double Theta_x_ddot = 0.0;
            double Theta_y_ddot = 0.0;
            double r0_sensor;
            double u_x;
            double u_y;
            double u_x_ddot;
            double u_y_ddot;
            double u_x_iMinus1;
            double u_y_iMinus1;
            double u_x_ddot_iMinus1;
            double u_y_ddot_iMinus1;
            double theta_ddot = 0.0;
            double r_ddot = 0.0;
            double phi_ddot = 0.0;

            if (parameters.UsePipeMovementReconstruction) // % compute additional variables used for pipe movement reconstruction
            {
                r_ddot = state.XAcceleration[parameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[parameters.Drillstring.IndexSensor]) + state.YAcceleration[parameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[parameters.Drillstring.IndexSensor]) -
                                state.XVelocity[parameters.Drillstring.IndexSensor] * state.WhirlVelocity[parameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[parameters.Drillstring.IndexSensor]) +
                                state.YVelocity[parameters.Drillstring.IndexSensor] * state.WhirlVelocity[parameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[parameters.Drillstring.IndexSensor]);

                phi_ddot = state.SlipCondition[parameters.Drillstring.IndexSensor] * (1.0 / (Math.Pow(state.RadialDisplacement[parameters.Drillstring.IndexSensor], 2) + Constants.RegularizationCoefficient) * (state.YAcceleration[parameters.Drillstring.IndexSensor] * state.XDisplacement[parameters.Drillstring.IndexSensor] -
                    state.XAcceleration[parameters.Drillstring.IndexSensor] * state.YDisplacement[parameters.Drillstring.IndexSensor]) - 2.0 * state.RadialVelocity[parameters.Drillstring.IndexSensor] * state.WhirlVelocity[parameters.Drillstring.IndexSensor] / (state.RadialDisplacement[parameters.Drillstring.IndexSensor] + Constants.RegularizationCoefficient)) +
                    (1 - state.SlipCondition[parameters.Drillstring.IndexSensor]) * state.PhiDdotNoSlipSensor;
                double phi0_sensor = 0;
                phi_ddot = 0.0;
               
                if (!parameters.Drillstring.SleeveIndexPosition.Contains(parameters.Drillstring.IndexSensor))
                {
                    r0_sensor = 0.5 * (parameters.Drillstring.ElementInnerRadius[parameters.Drillstring.IndexSensor] + parameters.Drillstring.ElementOuterRadius[parameters.Drillstring.IndexSensor]); // radial position of accelerometer relative to pipe centerline
                    theta_ddot = state.SlipCondition[parameters.Drillstring.IndexSensor] * state.AngularAcceleration[parameters.Drillstring.IndexSensor] + (1 - state.SlipCondition[parameters.Drillstring.IndexSensor]) * state.ThetaDotNoSlipSensor;
                    u_x = state.XDisplacement[parameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) - phi0_sensor * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor]);
                    u_y = state.YDisplacement[parameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) + r0_sensor * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor]);
                    u_x_ddot = state.XAcceleration[parameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) - state.AngularAcceleration[parameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor])) +
                        phi0_sensor * (Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) -
                        state.AngularAcceleration[parameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor]));
                    u_y_ddot = state.YAcceleration[parameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) - state.AngularAcceleration[parameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor])) -
                        r0_sensor * (Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor]) - state.AngularAcceleration[parameters.Drillstring.IndexSensor] * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor]));
                }
                else
                {
                    int idx_sleeve_sensor = parameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == parameters.Drillstring.IndexSensor);
                    r0_sensor = 0.5 * (parameters.Drillstring.SleeveInnerRadius + parameters.Drillstring.SleeveOuterRadius);

                    theta_ddot = state.SlipCondition[parameters.Drillstring.IndexSensor] * state.SleeveAngularAcceleration[idx_sleeve_sensor] + (1 - state.SlipCondition[parameters.Drillstring.IndexSensor]) * state.ThetaDotNoSlipSensor;
                    u_x = state.XDisplacement[parameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y = state.YDisplacement[parameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot = state.XAcceleration[parameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot = state.YAcceleration[parameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }
                if (!parameters.Drillstring.SleeveIndexPosition.Contains(parameters.Drillstring.IndexSensor - 1))
                {
                    r0_sensor = 0.5 * (parameters.Drillstring.ElementInnerRadius[parameters.Drillstring.IndexSensor - 1] + parameters.Drillstring.ElementOuterRadius[parameters.Drillstring.IndexSensor - 1]);
                    u_x_iMinus1 = state.XDisplacement[parameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) - phi0_sensor * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]);
                    u_y_iMinus1 = state.YDisplacement[parameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) + r0_sensor * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]);
                    u_x_ddot_iMinus1 = state.XAcceleration[parameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[parameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1])) + phi0_sensor * (Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[parameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]));
                    u_y_ddot_iMinus1 = state.YAcceleration[parameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[parameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1])) -
                       r0_sensor * (Math.Pow(state.WhirlVelocity[parameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[parameters.Drillstring.IndexSensor - 1] * Math.Cos(state.AngularDisplacement[parameters.Drillstring.IndexSensor - 1]));
                }
                else
                {
                    int idx_sleeve_sensor = parameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == parameters.Drillstring.IndexSensor - 1);
                    u_x_iMinus1 = state.XDisplacement[parameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y_iMinus1 = state.YDisplacement[parameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot_iMinus1 = state.XAcceleration[parameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot_iMinus1 = state.YAcceleration[parameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }

                // Bending angles
                Theta_x = -(u_y - u_y_iMinus1) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle x-component
                Theta_y = (u_x - u_x_iMinus1) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1];// Bending angle y-component
                Theta_x_ddot = -(u_y_ddot - u_y_ddot_iMinus1) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle second derivative x-component
                Theta_y_ddot = (u_x_ddot - u_x_ddot_iMinus1) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle second derivative y-component*/
            }

            output.Depth[0] = state.ZDisplacement[0] + state.TopOfStringPosition;
            for (int i = 1; i < parameters.NumberOfElements; i ++)
            {
                output.Depth[i] = output.Depth[i - 1] + state.ZDisplacement[i] + parameters.Drillstring.ElementLength[i-1];   
            }
            output.BitVelocity = state.BitVelocity;//Bit Velocity
            // Parse outputs
            output.NormalForceProfileStiffString = state.NormalCollisionForce; // Pipe shear strain 
            output.NormalForceProfileSoftString = state.SoftStringNormalForce;
            output.TensionProfile = state.Tension; // Tension profile
           
            output.Torque = state.Torque; // Torque profile vs. depth
            output.BendingMomentX = state.BendingMomentX;// Bending moment x-component profile
            output.BendingMomentY = state.BendingMomentY;// Bending moment y-component profile
            //output.TangentialForceProfile = TangentialCoulombFrictionForce;// Bending moment y-component profile Tangential force profile
            output.WeightOnBit = state.WeightOnBit;  // Weight on bit 
            output.TorqueOnBit = state.TorqueOnBit;  // Torque on bit
            

            output.SensorMb_x = state.BendingMomentX[parameters.Drillstring.IndexSensor];
            output.SensorMb_y = state.BendingMomentY[parameters.Drillstring.IndexSensor];
            output.RadialDisplacement = state.RadialDisplacement;
            output.WhirlAngle = state.WhirlAngle;
            output.WhirlSpeed = state.WhirlVelocity;
            output.BendingMoment = (Square(state.BendingMomentX) + Square(state.BendingMomentY)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

            if (parameters.UsePipeMovementReconstruction)
            {
                output.SensorAxialVelocity = state.ZVelocity[parameters.Drillstring.IndexSensor]; // sleeve angular displacement;
                output.SensorAxialDisplacement = output.SensorAxialDisplacement + output.SensorAxialVelocity * parameters.OuterLoopTimeStep;
                if (!parameters.Drillstring.SleeveIndexPosition.Contains(parameters.Drillstring.IndexSensor)) //sleeve angular velocity
                {
                    output.SensorAngularPosition = state.AngularDisplacement[parameters.Drillstring.IndexSensor]; //pipe angular displacement
                    output.SensorAngularVelocity = state.AngularVelocity[parameters.Drillstring.IndexSensor]; //pipe angular velocity
                }
                else
                {
                    int idx_sleeve_sensor = parameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == parameters.Drillstring.IndexSensor - 1);
                    output.SensorAngularPosition = state.SleeveAngularDisplacement[idx_sleeve_sensor];
                    output.SensorAngularVelocity = state.SleeveAngularVelocity[idx_sleeve_sensor];
                }
                output.SensorRadialPosition = state.RadialDisplacement[parameters.Drillstring.IndexSensor]; //radial position
                output.SensorWhirlAngle = state.WhirlAngle[parameters.Drillstring.IndexSensor]; //whirl angle
                output.SensorRadialSpeed =state.RadialVelocity[parameters.Drillstring.IndexSensor]; //radial velocity
                output.SensorWhirlSpeed = state.WhirlVelocity[parameters.Drillstring.IndexSensor]; //whirl velocity
                output.SensorAxialAcceleration = state.ZAcceleration[parameters.Drillstring.IndexSensor]; //axial acceleration
                output.SensorAngularAcceleration = theta_ddot; // angular acceleration
                output.SensorRadialAcceleration = r_ddot; // radial acceleration
                output.SensorWhirlAcceleration = phi_ddot; // whirl acceleration
                output.SensorBendingAngleX = Theta_x; // bending angle x-component
                output.SensorBendingAngleY = Theta_y; // bending angle y-component
                output.SecondDerivativeSensorBendingAngleX = Theta_x_ddot; // bending angle second derivative x-component
                output.SecondDerivativeSensorBendingAngleY = Theta_y_ddot; // bending angle second derivative y-component
                output.SensorPipeInclination = parameters.Trajectory.InterpolatedTheta[parameters.Drillstring.IndexSensor];
                output.SensorPipeAzimuthAt = parameters.Trajectory.InterpolatedPhi[parameters.Drillstring.IndexSensor];
                output.SensorTension = state.Tension[parameters.Drillstring.IndexSensor];
                output.SensorTorque = output.Torque[parameters.Drillstring.IndexSensor];

                CalculateInLocalFrame();
            }
        }

        public void AddNewLumpedElement() // TODO sjekk
        {
            parameters.AddNewLumpedElement();

            state.AddNewLumpedElement();

            solverODE.AddNewLumpedElement();        

            //axialTorsionalModel = new AxialTorsionalModel(state, simulationParameters, axialTorsionalModel);            
            //lateralModel = new LateralModel(simulationParameters, state);
        }
    }
}
