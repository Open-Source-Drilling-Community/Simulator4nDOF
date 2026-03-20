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

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class Solver
    {
        private State state;
        private Output output;
        private SimulationParameters simulationParameters;
        private readonly Configuration configuration;
        //private Input simulationParameters.Input;
        private TopdriveController topdriveController;
        private DrawworksAndTopdriveController drawworksAndTopdrive;
        private AxialModel axialModel;
        private TorsionalModel torsionalModel;
        
        private LateralModel lateralModel;
        private IBitRock bitRockModel;

        private ISolverODE<LateralModel> solverODELateral; 
        // Upwind scheme is used for the axial and torsional wave equations to better capture the wave propagation dynamics

        private ISolverODE<WaveModel> solverODEAxial; 
        private ISolverODE<WaveModel> solverODETorsional; 
        
        public Solver(SimulationParameters simulationParameters, in DataModel.Configuration configuration)
        {
            this.simulationParameters = simulationParameters;
            this.configuration = configuration;
        
     
            output = new Output(in simulationParameters, in configuration);
            topdriveController = new TopdriveController(in configuration, in simulationParameters);
            drawworksAndTopdrive = new DrawworksAndTopdriveController();
            torsionalModel = new TorsionalModel(state, simulationParameters);
            axialModel = new AxialModel(state, simulationParameters);            
            lateralModel = new LateralModel(simulationParameters, state);
            state = new State(in axialModel, in torsionalModel, in lateralModel, in simulationParameters);            
            bitRockModel = configuration.BitRockModelEnum switch
            {
                BitRockModelEnum.Detournay => new Detournay(
                                                    axialModel,
                                                    torsionalModel, 
                                                    simulationParameters.Drillstring, 
                                                    configuration),
                BitRockModelEnum.MSE => new MSE(),
                _ => throw new ArgumentException($"Unknown BitRockModelEnum: {configuration.BitRockModelEnum}")
            };

            //Create an instance of the selected ODE solver
            solverODELateral = simulationParameters.SolverType switch
            {
                SolverType.EulerMethod => solverODELateral = new EulerMethod(),
                SolverType.VerletMethod => solverODELateral = new VerletMethod(simulationParameters),
                _ => throw new ArgumentException($"Unknown SolverType: {simulationParameters.SolverType}")
            };
            
            solverODEAxial = new UpwindScheme(axialModel, in simulationParameters);
            solverODETorsional = new UpwindScheme(torsionalModel, in simulationParameters);
        }

        public (State, Output, Input) OuterStep(SetPoints setPoints)
        {
            simulationParameters.TopDriveDrawwork.SurfaceRotation = setPoints.SurfaceRPM;
            simulationParameters.TopDriveDrawwork.SurfaceAxialVelocity = setPoints.TopOfStringVelocity;
            simulationParameters.Input.BottomExtraNormalForce = setPoints.BottomExtraSideForce;
            simulationParameters.Input.DifferenceStaticKineticFriction = setPoints.DifferenceStaticKineticFriction;
            simulationParameters.Input.StickingBoolean = setPoints.Sticking;

            if (setPoints.StribeckCriticalVelocity > 0)
            {
                simulationParameters.Input.StribeckCriticalVelocity = setPoints.StribeckCriticalVelocity;
                simulationParameters.Friction.stribeck = 1.0 / simulationParameters.Input.StribeckCriticalVelocity;
            }
            if (simulationParameters.Friction.StaticFrictionCoefficient.Count == simulationParameters.Friction.KinematicFrictionCoefficient.Count)
            {
                for (int i = 0; i < simulationParameters.Friction.StaticFrictionCoefficient.Count; i++)
                {
                    simulationParameters.Friction.StaticFrictionCoefficient[i] = simulationParameters.Friction.KinematicFrictionCoefficient[i] + simulationParameters.Input.DifferenceStaticKineticFriction;
                }
            }

            // Calculate u.v0 and u.omega_sp 
            drawworksAndTopdrive.Step(state, in simulationParameters);

            // Controller for top drive - calculate u.tau_Motor
            topdriveController.Step(state, in simulationParameters);

            //UpdateDepths(); // BitDepth, HoleDepth, TopOfStringPosition, OnBottom

            if (simulationParameters.MovingDrillstring)
            {
                // update axial position of distributed and lumped elements to simulate moving drillstring
                simulationParameters.DistributedCells.x = simulationParameters.DistributedCells.x + ToVector(state.PipeAxialVelocity.ToColumnMajorArray()) * simulationParameters.OuterLoopTimeStep;
                simulationParameters.LumpedCells.ElementLength[0] = simulationParameters.LumpedCells.ElementLength[0] + state.TopDrive.CalculateSurfaceAxialVelocity * simulationParameters.OuterLoopTimeStep;
                simulationParameters.LumpedCells.ElementLength.SetSubVector(1, simulationParameters.LumpedCells.ElementLength.Count() - 1, simulationParameters.LumpedCells.ElementLength.SubVector(1, simulationParameters.LumpedCells.ElementLength.Count - 1) + state.ZVelocity * simulationParameters.OuterLoopTimeStep);

                // update trajectory parameters and buoyancy force calculations
                if (Math.Abs(state.BitDepth - state.PreviousCalculatedBitDepth) > 1.0)
                {
                    simulationParameters.Trajectory.UpdateTrajectory(simulationParameters.LumpedCells);
                    simulationParameters.Flow.UpdateBuoyancy(simulationParameters.LumpedCells, simulationParameters.Trajectory, simulationParameters.Drillstring, simulationParameters.UseBuoyancyFactor);
                    simulationParameters.Wellbore.UpdateWellbore(simulationParameters.Drillstring, simulationParameters.LumpedCells);
                    state.PreviousCalculatedBitDepth = state.BitDepth;
                }

                // if the top most element has traveled more than the distance between
                // elements, we create a new lumped element and corresponding distributed section;
                // we also need to reconstruct the parameter and state vectors to include the new elements
                if (simulationParameters.LumpedCells.ElementLength[0] > simulationParameters.LumpedCells.DistanceBetweenElements)
                {
                    AddNewLumpedElement();
                    simulationParameters.Trajectory.UpdateTrajectory(simulationParameters.LumpedCells);
                    simulationParameters.Flow.UpdateBuoyancy(simulationParameters.LumpedCells, simulationParameters.Trajectory, simulationParameters.Drillstring, simulationParameters.UseBuoyancyFactor);
                    simulationParameters.Wellbore.UpdateWellbore(simulationParameters.Drillstring, simulationParameters.LumpedCells);
                    simulationParameters.Drillstring.IndexSensor = simulationParameters.Drillstring.IndexSensor + 1;
                }
            }
            InnerStep();
            output.UpdateSSI(state.Step * simulationParameters.OuterLoopTimeStep);
            state.Step = state.Step + 1;
            return (state, output, simulationParameters.Input);
        }

        public void UpdateDepths()
        {
            state.BitDepth = state.BitDepth + output.BitVelocity * simulationParameters.OuterLoopTimeStep;
            state.TopOfStringPosition = state.TopOfStringPosition - state.TopDrive.CalculateSurfaceAxialVelocity * simulationParameters.OuterLoopTimeStep;

            if (state.HoleDepth - state.BitDepth < 1E-3 && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth - state.BitDepth < 1E-3 || state.OnBottomStart > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }
         public void UpdateDepthInnerLoop()
        {
            state.BitDepth = state.BitDepth + state.BitVelocity * simulationParameters.InnerLoopTimeStep;
            state.TopOfStringPosition = state.TopOfStringPosition - state.TopDrive.CalculateSurfaceAxialVelocity * simulationParameters.InnerLoopTimeStep;

            if (state.HoleDepth < state.BitDepth && !state.BitOnBotton)
                state.OnBottomStart = state.Step;

            state.BitOnBotton = state.HoleDepth < state.BitDepth;// || state.onBottom_startIdx > 0;            
            state.HoleDepth = Math.Max(state.BitDepth, state.HoleDepth);
        }


        public void CalculateInLocalFrame()
        {
            Matrix<double> SensorToPipeLocal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), simulationParameters.Drillstring.SensorRadialDistance },
                        { Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { -Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle), 0, Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle), 0 },
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
                        { Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) },
                        { -Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(simulationParameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(simulationParameters.Drillstring.SensorMisalignmentPolarAngle) }
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
            var accelerationInGlobalFrameM = LateralRotationCenterToGlobal * PipeLocalToLateralRotationCenterSecondDerivative * SensorToPipeLocal * ToVector((simulationParameters.Drillstring.SensorDisplacementsInLocalFrame.Append(1).ToArray()));
            var accelerationInGlobalFrame = accelerationInGlobalFrameM.SubVector(0, 3) - Vector<double>.Build.DenseOfArray(new double[] { 0, 0, Constants.GravitationalAcceleration });

            var accelerationInLocalFrame = PipeLocalToSensor * PipeNonRotatingToPipeLocal * LateralRotationCenterToPipeNonRotating * GlobalToLateralRotationCenter * accelerationInGlobalFrame;

            output.SensorRadialAccelerationLocalFrame = -accelerationInLocalFrame[0];
            output.SensorTangentialAccelerationLocalFrame = accelerationInLocalFrame[1];
            output.SensorAxialAccelerationLocalFrame = accelerationInLocalFrame[2];

            output.SensorBendingMomentX = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Sin((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * simulationParameters.OuterLoopTimeStep);
            output.SensorBendingMomentY = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Cos((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * simulationParameters.OuterLoopTimeStep);
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
            if (!simulationParameters.UseMudMotor)
                output.BitRotationInRPM = state.PipeAngularVelocity[state.PipeAngularVelocity.Count - 1];
            else
                output.BitRotationInRPM = state.MudRotorAngularVelocity;

            double dtTemp = simulationParameters.DistributedCells.DistributedSectionLength / Math.Max(simulationParameters.Drillstring.TorsionalWaveSpeed, simulationParameters.Drillstring.AxialWaveSpeed) * 0.8;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
            // Wave equations are transformed into their Riemann invariants
            axialModel.PrepareModel(axialModel, state, simulationParameters);   
            torsionalModel.PrepareModel(torsionalModel, state, simulationParameters);
            lateralModel.PrepareModel(lateralModel, state, simulationParameters);
            // Solve lumped and distributed equations
            for (int innerIterationNo = 0; innerIterationNo < simulationParameters.InnerLoopIterations; innerIterationNo++)
            {
                UpdateDepthInnerLoop();            
                // Update axial-torsional state using upwind scheme
                // The staggered method is used for a semi-implicit integration, increasing stability           
                output.SimulationHealthy = solverODEAxial.IntegrationStep(state, axialModel, in simulationParameters);                   
                output.SimulationHealthy = solverODETorsional.IntegrationStep(state, torsionalModel, in simulationParameters);                                   
                //  Calculate interaction forces on bit based on selected bit-rock model 
                // and update the state accordingly
                bitRockModel.CalculateInteractionForce(state, in simulationParameters);
                bitRockModel.ManageStickingOnBottom(state, in simulationParameters);
                // Calculate torque on bit and top drive torque for the next iteration                                                                   
                torsionalModel.IntegrateTopDriveSpeed(state, in simulationParameters);
                // Calculate lateral accelerations                    
                output.SimulationHealthy = solverODELateral.IntegrationStep(state, lateralModel, in simulationParameters);
                //Abort simulation in case of divergence
                if (!output.SimulationHealthy)
                {                    
                    return;
                }
                // Mud motor
                if (simulationParameters.UseMudMotor)
                {
                    state.MudStatorAngularVelocity = state.WhirlVelocity[state.WhirlVelocity.Count - 1];
                    state.MudRotorAngularVelocity = state.MudRotorAngularVelocity + simulationParameters.InnerLoopTimeStep / (simulationParameters.MudMotor.I_rotor + simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * Math.Pow(simulationParameters.MudMotor.N_rotor, 2)) *
                        (state.AngularAcceleration[state.WhirlVelocity.Count - 1] * simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * simulationParameters.MudMotor.N_stator * simulationParameters.MudMotor.N_rotor + state.MudTorque - state.TorqueOnBit);
                }
            }

            // Compute states from Riemann invariants
            //state.PipeAngularVelocity = 0.5 * (state.DownwardTorsionalWave + state.UpwardTorsionalWave);
            //state.PipeShearStrain = 1.0 / (2.0 * simulationParameters.Drillstring.TorsionalWaveSpeed) * (state.DownwardTorsionalWave - state.UpwardTorsionalWave);
            //state.PipeAxialVelocity = 0.5 * (state.DownwardAxialWave + state.UpwardAxialWave);
            //state.PipeAxialStrain = 1.0 / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed) * (state.DownwardAxialWave - state.UpwardAxialWave);
        
            // Bending moments
            lateralModel.UpdateBendingMoments(state, simulationParameters);
           
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

            if (simulationParameters.UsePipeMovementReconstruction) // % compute additional variables used for pipe movement reconstruction
            {
                r_ddot = state.XAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) + state.YAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) -
                                state.XVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) +
                                state.YVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]);

                phi_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * (1.0 / (Math.Pow(state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor], 2) + Constants.RegularizationCoefficient) * (state.YAcceleration[simulationParameters.Drillstring.IndexSensor] * state.XDisplacement[simulationParameters.Drillstring.IndexSensor] -
                    state.XAcceleration[simulationParameters.Drillstring.IndexSensor] * state.YDisplacement[simulationParameters.Drillstring.IndexSensor]) - 2.0 * state.RadialVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] / (state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor] + Constants.RegularizationCoefficient)) +
                    (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * state.PhiDdotNoSlipSensor;
                double phi0_sensor = 0;
                phi_ddot = 0.0;
               
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor]); // radial position of accelerometer relative to pipe centerline
                    theta_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] + (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * state.ThetaDotNoSlipSensor;
                    u_x = state.XDisplacement[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - phi0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]);
                    u_y = state.YDisplacement[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) + r0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]);
                    u_x_ddot = state.XAcceleration[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor])) +
                        phi0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]));
                    u_y_ddot = state.YAcceleration[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor])) -
                        r0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor);
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.SleeveInnerRadius + simulationParameters.Drillstring.SleeveOuterRadius);

                    theta_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * state.SleeveAngularAcceleration[idx_sleeve_sensor] + (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * state.ThetaDotNoSlipSensor;
                    u_x = state.XDisplacement[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y = state.YDisplacement[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot = state.XAcceleration[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot = state.YAcceleration[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor - 1))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor - 1] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_iMinus1 = state.XDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - phi0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_y_iMinus1 = state.YDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) + r0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_ddot_iMinus1 = state.XAcceleration[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1])) + phi0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]));
                    u_y_ddot_iMinus1 = state.YAcceleration[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1])) -
                       r0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor - 1);
                    u_x_iMinus1 = state.XDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y_iMinus1 = state.YDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot_iMinus1 = state.XAcceleration[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot_iMinus1 = state.YAcceleration[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }

                // Bending angles
                Theta_x = -(u_y - u_y_iMinus1) / simulationParameters.LumpedCells.DistanceBetweenElements; // Bending angle x-component
                Theta_y = (u_x - u_x_iMinus1) / simulationParameters.LumpedCells.DistanceBetweenElements;// Bending angle y-component
                Theta_x_ddot = -(u_y_ddot - u_y_ddot_iMinus1) / simulationParameters.LumpedCells.DistanceBetweenElements; // Bending angle second derivative x-component
                Theta_y_ddot = (u_x_ddot - u_x_ddot_iMinus1) / simulationParameters.LumpedCells.DistanceBetweenElements; // Bending angle second derivative y-component*/
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
            
            output.Depth = simulationParameters.LumpedCells.ElementLength;
            output.SensorMb_x = state.BendingMomentX[simulationParameters.Drillstring.IndexSensor];
            output.SensorMb_y = state.BendingMomentY[simulationParameters.Drillstring.IndexSensor];
            output.RadialDisplacement = state.RadialDisplacement;
            output.WhirlAngle = state.WhirlAngle;
            output.WhirlSpeed = state.WhirlVelocity;
            output.BendingMoment = (Square(state.BendingMomentX) + Square(state.BendingMomentY)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

            if (simulationParameters.UsePipeMovementReconstruction)
            {
                output.SensorAxialVelocity = state.ZVelocity[simulationParameters.Drillstring.IndexSensor]; // sleeve angular displacement;
                output.SensorAxialDisplacement = output.SensorAxialDisplacement + output.SensorAxialVelocity * simulationParameters.OuterLoopTimeStep;
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor)) //sleeve angular velocity
                {
                    output.SensorAngularPosition = state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]; //pipe angular displacement
                    output.SensorAngularVelocity = state.AngularVelocity[simulationParameters.Drillstring.IndexSensor]; //pipe angular velocity
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor - 1);
                    output.SensorAngularPosition = state.SleeveAngularDisplacement[idx_sleeve_sensor];
                    output.SensorAngularVelocity = state.SleeveAngularVelocity[idx_sleeve_sensor];
                }
                output.SensorRadialPosition = state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor]; //radial position
                output.SensorWhirlAngle = state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]; //whirl angle
                output.SensorRadialSpeed =state.RadialVelocity[simulationParameters.Drillstring.IndexSensor]; //radial velocity
                output.SensorWhirlSpeed = state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor]; //whirl velocity
                output.SensorAxialAcceleration = state.ZAcceleration[simulationParameters.Drillstring.IndexSensor]; //axial acceleration
                output.SensorAngularAcceleration = theta_ddot; // angular acceleration
                output.SensorRadialAcceleration = r_ddot; // radial acceleration
                output.SensorWhirlAcceleration = phi_ddot; // whirl acceleration
                output.SensorBendingAngleX = Theta_x; // bending angle x-component
                output.SensorBendingAngleY = Theta_y; // bending angle y-component
                output.SecondDerivativeSensorBendingAngleX = Theta_x_ddot; // bending angle second derivative x-component
                output.SecondDerivativeSensorBendingAngleY = Theta_y_ddot; // bending angle second derivative y-component
                output.SensorPipeInclination = simulationParameters.Trajectory.InterpolatedTheta[simulationParameters.Drillstring.IndexSensor];
                output.SensorPipeAzimuthAt = simulationParameters.Trajectory.InterpolatedPhi[simulationParameters.Drillstring.IndexSensor];
                output.SensorTension = state.Tension[simulationParameters.Drillstring.IndexSensor];
                output.SensorTorque = output.Torque[simulationParameters.Drillstring.IndexSensor];

                CalculateInLocalFrame();
            }
        }

        public void AddNewLumpedElement() // TODO sjekk
        {
            simulationParameters.AddNewLumpedElement();

            state.AddNewLumpedElement();

            solverODELateral.AddNewLumpedElement();        

            //axialTorsionalModel = new AxialTorsionalModel(state, simulationParameters, axialTorsionalModel);            
            //lateralModel = new LateralModel(simulationParameters, state);
        }
    }
}
