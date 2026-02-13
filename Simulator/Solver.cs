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
namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class Solver
    {
        private State state;
        private Output output;
        private SimulationParameters simulationParameters;
        private readonly Configuration configuration;
        private Input simulationInput;
        private TopdriveController topdriveController;
        private DrawworksAndTopdrive drawworksAndTopdrive;
        private AxialTorsionalModel axialTorsionalModel;
        private LateralModel lateralModel;

        private ISolverODE solverODE; 

        public Solver(SimulationParameters simulationParameters, in DataModel.Configuration configuration)
        {
            this.simulationParameters = simulationParameters;
            this.configuration = configuration;

            simulationInput = new Input();
            simulationInput.SurfaceRotation = configuration.SurfaceRPM;
            simulationInput.SurfaceAxialVelocity = configuration.TopOfStringVelocity;

            state = new State(in simulationParameters, configuration.BitDepth, configuration.HoleDepth, configuration.TopOfStringPosition);
            output = new Output(in simulationParameters, in configuration);
            topdriveController = new TopdriveController(in configuration, in simulationParameters);
            drawworksAndTopdrive = new DrawworksAndTopdrive();
            axialTorsionalModel = new AxialTorsionalModel(state, simulationParameters, simulationInput);
            lateralModel = new LateralModel(simulationParameters, state);
            //Create an instance of the selected ODE solver
            switch (simulationParameters.SolverODEEnum)
            {
                case SolverODEEnum.EulerMethod:
                    solverODE = new EulerMethod();
                    break;
                case SolverODEEnum.VerletMethod:
                    solverODE = new VerletMethod(simulationParameters);
                    break;                
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public (State, Output, Input) OuterStep(double SurfaceRPM, double TopOfStringVelocity, double bottomExtraVerticalForce, double differenceStaticKineticFriction, double stribeckCriticalVelocity, bool sticking)
        {
            simulationInput.SurfaceRotation = SurfaceRPM;
            simulationInput.SurfaceAxialVelocity = TopOfStringVelocity;
            simulationInput.BottomExtraNormalForce = bottomExtraVerticalForce;
            simulationInput.DifferenceStaticKineticFriction = differenceStaticKineticFriction;
            simulationInput.StickingBoolean = sticking;

            if (stribeckCriticalVelocity > 0)
            {
                simulationInput.StribeckCriticalVelocity = stribeckCriticalVelocity;
                simulationParameters.Friction.stribeck = 1.0 / simulationInput.StribeckCriticalVelocity;
            }
            if (simulationParameters.Friction.mu_s.Count == simulationParameters.Friction.mu_k.Count)
            {
                for (int i = 0; i < simulationParameters.Friction.mu_s.Count; i++)
                {
                    simulationParameters.Friction.mu_s[i] = simulationParameters.Friction.mu_k[i] + simulationInput.DifferenceStaticKineticFriction;
                }
            }

            // Calculate u.v0 and u.omega_sp 
            drawworksAndTopdrive.Step(in configuration, state, simulationInput);

            // Controller for top drive - calculate u.tau_Motor
            topdriveController.Step(in configuration, in state, in simulationParameters, ref simulationInput);

            UpdateDepths(); // BitDepth, HoleDepth, TopOfStringPosition, OnBottom

            if (configuration.MovingDrillstring)
            {
                // update axial position of distributed and lumped elements to simulate moving drillstring
                simulationParameters.DistributedCells.x = simulationParameters.DistributedCells.x + ToVector(state.PipeAxialVelocity.ToColumnMajorArray()) * configuration.TimeStep;
                simulationParameters.LumpedCells.ElementLength[0] = simulationParameters.LumpedCells.ElementLength[0] + simulationInput.CalculateSurfaceAxialVelocity * configuration.TimeStep;
                simulationParameters.LumpedCells.ElementLength.SetSubVector(1, simulationParameters.LumpedCells.ElementLength.Count() - 1, simulationParameters.LumpedCells.ElementLength.SubVector(1, simulationParameters.LumpedCells.ElementLength.Count - 1) + state.AxialVelocity * configuration.TimeStep);

                // update trajectory parameters and buoyancy force calculations
                if (Math.Abs(state.BitDepth - state.previousCalculatedBitDepth) > 1.0)
                {
                    simulationParameters.Trajectory.UpdateTrajectory(simulationParameters.LumpedCells);
                    simulationParameters.Buoyancy.UpdateBuoyancy(simulationParameters.LumpedCells, simulationParameters.Trajectory, simulationParameters.Drillstring, configuration.UseBuoyancyFactor);
                    simulationParameters.Wellbore.UpdateWellbore(simulationParameters.Drillstring, simulationParameters.LumpedCells);
                    state.previousCalculatedBitDepth = state.BitDepth;
                }

                // if the top most element has traveled more than the distance between
                // elements, we create a new lumped element and corresponding distributed section;
                // we also need to reconstruct the parameter and state vectors to include the new elements
                if (simulationParameters.LumpedCells.ElementLength[0] > simulationParameters.LumpedCells.DistanceBetweenElements)
                {
                    AddNewLumpedElement();
                    simulationParameters.Trajectory.UpdateTrajectory(simulationParameters.LumpedCells);
                    simulationParameters.Buoyancy.UpdateBuoyancy(simulationParameters.LumpedCells, simulationParameters.Trajectory, simulationParameters.Drillstring, configuration.UseBuoyancyFactor);
                    simulationParameters.Wellbore.UpdateWellbore(simulationParameters.Drillstring, simulationParameters.LumpedCells);
                    simulationParameters.Drillstring.IndexSensor = simulationParameters.Drillstring.IndexSensor + 1;
                }
            }
            InnerStep();

            output.UpdateSSI(state.Step * configuration.TimeStep);

            state.Step = state.Step + 1;

            return (state, output, simulationInput);
        }

        public void UpdateDepths()
        {
            state.BitDepth = state.BitDepth + (output.BitVelocity * configuration.TimeStep);
            state.TopOfStringPosition = state.TopOfStringPosition - simulationInput.CalculateSurfaceAxialVelocity * configuration.TimeStep;

            if (state.HoleDepth - state.BitDepth < 0.1 && !state.onBottom)
                state.onBottom_startIdx = state.Step;

            state.onBottom = state.HoleDepth - state.BitDepth < 0.1 || state.onBottom_startIdx > 0;
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
            var accelerationInGlobalFrame = accelerationInGlobalFrameM.SubVector(0, 3) - Vector<double>.Build.DenseOfArray(new double[] { 0, 0, Constants.g });

            var accelerationInLocalFrame = PipeLocalToSensor * PipeNonRotatingToPipeLocal * LateralRotationCenterToPipeNonRotating * GlobalToLateralRotationCenter * accelerationInGlobalFrame;

            output.SensorRadialAccelerationLocalFrame = -accelerationInLocalFrame[0];
            output.SensorTangentialAccelerationLocalFrame = accelerationInLocalFrame[1];
            output.SensorAxialAccelerationLocalFrame = accelerationInLocalFrame[2];

            output.SensorBendingMomentX = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Sin((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * configuration.TimeStep);
            output.SensorBendingMomentY = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Cos((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.Step * configuration.TimeStep);
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
            output.TopDriveRotationInRPM = state.TopDriveAngularVelocity;
            if (!configuration.UseMudMotor)
                output.BitRotationInRPM = state.PipeAngularVelocity[state.PipeAngularVelocity.RowCount - 1, state.PipeAngularVelocity.ColumnCount - 1];
            else
                output.BitRotationInRPM = state.MudRotorAngularVelocity;

            double dtTemp = simulationParameters.DistributedCells.DistributedSectionLength / Math.Max(simulationParameters.Drillstring.TorsionalWaveSpeed, simulationParameters.Drillstring.AxialWaveSpeed) * 0.8;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
            
            // Wave equations are transformed into their Riemann invariants
            AccelerationCalculation.PrepareAxialTorsional(axialTorsionalModel, state, simulationParameters);            
            AccelerationCalculation.PreprareLateral(lateralModel, state, simulationParameters);            
                     
            // Solve lumped and distributed equations
            for (int innerIterationNo = 0; innerIterationNo < simulationParameters.InnerLoopIterations; innerIterationNo++)
            {
                // Calculate axial-torsional pde properties
                AccelerationCalculation.AxialTorsionalSystem(axialTorsionalModel, simulationInput, configuration, state, simulationParameters);                                 
                // Update axial-torsional state using upwind scheme
                // The staggered method is used for a semi-implicit integration, increasing stability           
                UpwindScheme.IntegrationStep(axialTorsionalModel, simulationParameters);                                                     
                // Calculate lateral accelerations                    
                AccelerationCalculation.LateralSystem(lateralModel, axialTorsionalModel, simulationInput, configuration, state, simulationParameters);        
                solverODE.IntegrationStep(state, simulationParameters);                                                                                           
                // Mud motor
                if (configuration.UseMudMotor)
                {
                    state.MudStatorAngularVelocity = state.WhirlVelocity[state.WhirlVelocity.Count - 1];
                    state.MudRotorAngularVelocity = state.MudRotorAngularVelocity + simulationParameters.InnerLoopTimeStep / (simulationParameters.MudMotor.I_rotor + simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * Math.Pow(simulationParameters.MudMotor.N_rotor, 2)) *
                        (state.AngularAcceleration[state.WhirlVelocity.Count - 1] * simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * simulationParameters.MudMotor.N_stator * simulationParameters.MudMotor.N_rotor + lateralModel.MudTorque - axialTorsionalModel.TorqueOnBit);
                }
            }

            // Compute states from Riemann invariants
            state.PipeAngularVelocity = 0.5 * (axialTorsionalModel.DownwardTorsionalWave + axialTorsionalModel.UpwardTorsionalWave);
            state.PipeShearStrain = 1.0 / (2.0 * simulationParameters.Drillstring.TorsionalWaveSpeed) * (axialTorsionalModel.DownwardTorsionalWave - axialTorsionalModel.UpwardTorsionalWave);

            state.PipeAxialVelocity = 0.5 * (axialTorsionalModel.DownwardAxialWave + axialTorsionalModel.UpwardAxialWave);
            state.PipeAxialStrain = 1.0 / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed) * (axialTorsionalModel.DownwardAxialWave - axialTorsionalModel.UpwardAxialWave);
        
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

            if (configuration.UsePipeMovementReconstruction) // % compute additional variables used for pipe movement reconstruction
            {
                r_ddot = state.XAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) + state.YAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) -
                                state.XVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) +
                                state.YVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]);

                phi_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * (1.0 / (Math.Pow(state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor], 2) + Constants.eps) * (state.YAcceleration[simulationParameters.Drillstring.IndexSensor] * state.XDisplacement[simulationParameters.Drillstring.IndexSensor] -
                    state.XAcceleration[simulationParameters.Drillstring.IndexSensor] * state.YDisplacement[simulationParameters.Drillstring.IndexSensor]) - 2.0 * state.RadialVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] / (state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor] + Constants.eps)) +
                    (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * lateralModel.PhiDdotNoSlipSensor;
                double phi0_sensor = 0;
                phi_ddot = 0.0;
               
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor]); // radial position of accelerometer relative to pipe centerline
                    theta_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] + (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * lateralModel.ThetaDotNoSlipSensor;
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

                    theta_ddot = state.SlipCondition[simulationParameters.Drillstring.IndexSensor] * state.SleeveAngularAcceleration[idx_sleeve_sensor] + (1 - state.SlipCondition[simulationParameters.Drillstring.IndexSensor]) * lateralModel.ThetaDotNoSlipSensor;
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
          
            output.BitVelocity = state.PipeAxialVelocity[state.PipeAxialVelocity.RowCount - 1, state.PipeAxialVelocity.ColumnCount - 1]; // Bit velocity

            // Parse outputs
            output.NormalForceProfileStiffString = lateralModel.NormalCollisionForce; // Pipe shear strain 
            output.NormalForceProfileSoftString = lateralModel.SoftStringNormalForce;
            output.TensionProfile = lateralModel.Tension; // Tension profile

            var tempMatrix = state.PipeShearStrain.PointwiseMultiply(lateralModel.ScalingMatrix * lateralModel.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            output.Torque = ToVector(tempMatrix.ToColumnMajorArray()); // Torque profile vs. depth
            output.BendingMomentX = lateralModel.BendingMomentX;// Bending moment x-component profile
            output.BendingMomentY = lateralModel.BendingMomentY;// Bending moment y-component profile
            //output.TangentialForceProfile = TangentialCoulombFrictionForce;// Bending moment y-component profile Tangential force profile
            output.WeightOnBit = axialTorsionalModel.WeightOnBit;  // Weight on bit
            output.TorqueOnBit = axialTorsionalModel.TorqueOnBit;  // Torque on bit
            output.Depth = simulationParameters.LumpedCells.ElementLength;
            output.SensorMb_x = lateralModel.BendingMomentX[simulationParameters.Drillstring.IndexSensor];
            output.SensorMb_y = lateralModel.BendingMomentY[simulationParameters.Drillstring.IndexSensor];
            output.RadialDisplacement = state.RadialDisplacement;
            output.WhirlAngle = state.WhirlAngle;
            output.WhirlSpeed = state.WhirlVelocity;
            output.BendingMoment = (Square(lateralModel.BendingMomentX) + Square(lateralModel.BendingMomentY)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

            if (configuration.UsePipeMovementReconstruction)
            {
                output.SensorAxialVelocity = state.AxialVelocity[simulationParameters.Drillstring.IndexSensor]; // sleeve angular displacement;
                output.SensorAxialDisplacement = output.SensorAxialDisplacement + output.SensorAxialVelocity * configuration.TimeStep;
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor)) //sleeve angular velocity
                {
                    output.SensorAngularPosition = state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]; //pipe angular displacement
                    output.SensorAngularVelocity = state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor]; //pipe angular velocity
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
                output.SensorAxialAcceleration = state.AxialAcceleration[simulationParameters.Drillstring.IndexSensor]; //axial acceleration
                output.SensorAngularAcceleration = theta_ddot; // angular acceleration
                output.SensorRadialAcceleration = r_ddot; // radial acceleration
                output.SensorWhirlAcceleration = phi_ddot; // whirl acceleration
                output.SensorBendingAngleX = Theta_x; // bending angle x-component
                output.SensorBendingAngleY = Theta_y; // bending angle y-component
                output.SecondDerivativeSensorBendingAngleX = Theta_x_ddot; // bending angle second derivative x-component
                output.SecondDerivativeSensorBendingAngleY = Theta_y_ddot; // bending angle second derivative y-component
                output.SensorPipeInclination = simulationParameters.Trajectory.thetaVec[simulationParameters.Drillstring.IndexSensor];
                output.SensorPipeAzimuthAt = simulationParameters.Trajectory.phiVec[simulationParameters.Drillstring.IndexSensor];
                output.SensorTension = lateralModel.Tension[simulationParameters.Drillstring.IndexSensor];
                output.SensorTorque = output.Torque[simulationParameters.Drillstring.IndexSensor];

                CalculateInLocalFrame();
            }
        }

        public void AddNewLumpedElement() // TODO sjekk
        {
            simulationParameters.AddNewLumpedElement();

            state.AddNewLumpedElement();

            solverODE.AddNewLumpedElement();        

            //axialTorsionalModel = new AxialTorsionalModel(state, simulationParameters, axialTorsionalModel);            
            //lateralModel = new LateralModel(simulationParameters, state);
        }
    }
}
