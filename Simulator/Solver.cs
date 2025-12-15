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
                simulationParameters.LumpedCells.xL[0] = simulationParameters.LumpedCells.xL[0] + simulationInput.CalculateSurfaceAxialVelocity * configuration.TimeStep;
                simulationParameters.LumpedCells.xL.SetSubVector(1, simulationParameters.LumpedCells.xL.Count() - 1, simulationParameters.LumpedCells.xL.SubVector(1, simulationParameters.LumpedCells.xL.Count - 1) + state.LumpedElementAxialVelocity * configuration.TimeStep);

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
                if (simulationParameters.LumpedCells.xL[0] > simulationParameters.LumpedCells.dxL)
                {
                    AddNewLumpedElement();
                    simulationParameters.Trajectory.UpdateTrajectory(simulationParameters.LumpedCells);
                    simulationParameters.Buoyancy.UpdateBuoyancy(simulationParameters.LumpedCells, simulationParameters.Trajectory, simulationParameters.Drillstring, configuration.UseBuoyancyFactor);
                    simulationParameters.Wellbore.UpdateWellbore(simulationParameters.Drillstring, simulationParameters.LumpedCells);
                    simulationParameters.Drillstring.IndexSensor = simulationParameters.Drillstring.IndexSensor + 1;
                }
            }
            InnerStep();

            output.UpdateSSI(state.step * configuration.TimeStep);

            state.step = state.step + 1;

            return (state, output, simulationInput);
        }

        public void UpdateDepths()
        {
            state.BitDepth = state.BitDepth + (output.BitVelocity * configuration.TimeStep);
            state.TopOfStringPosition = state.TopOfStringPosition - simulationInput.CalculateSurfaceAxialVelocity * configuration.TimeStep;

            if (state.HoleDepth - state.BitDepth < 0.1 && !state.onBottom)
                state.onBottom_startIdx = state.step;

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

            output.SensorBendingMomentX = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Sin((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.step * configuration.TimeStep);
            output.SensorBendingMomentY = Math.Sqrt(Math.Pow(output.SensorMb_x, 2) + Math.Pow(output.SensorMb_y, 2)) * Math.Cos((output.SensorAngularVelocity - output.SensorWhirlSpeed) * state.step * configuration.TimeStep);
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

            double dtTemp = simulationParameters.DistributedCells.dxM / Math.Max(simulationParameters.Drillstring.TorsionalWaveSpeed, simulationParameters.Drillstring.AxialWaveSpeed) * .4;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
            
            // Wave equations are transformed into their Riemann invariants
            Matrix<double> downwardTorsionalWave = state.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Downward traveling wave, torsional
            Matrix<double> upwardTorsionalWave = state.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            Matrix<double> downwardAxialWave = state.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            Matrix<double> upwardAxialWave = state.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial

            Vector<double> elementWiseProduct = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeArea);
            Matrix<double> scalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.PL, 1).ToColumnMatrix();
            Matrix<double> dragMatrix = scalingMatrix * elementWiseProduct.ToRowMatrix();
            dragMatrix = state.PipeAxialStrain.PointwiseMultiply(dragMatrix);
            Vector<double> drag_flattened = ToVector(dragMatrix.ToColumnMajorArray());
            Vector<double> drag = LinearInterpolate(simulationParameters.DistributedCells.x, drag_flattened, simulationParameters.LumpedCells.xL);

            Vector<double> phiVec_dote = ExtendVectorStart(0, simulationParameters.Trajectory.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, simulationParameters.Trajectory.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, simulationParameters.Trajectory.thetaVec);
            var cumTrapz = CumTrapz(simulationParameters.LumpedCells.xL, Reverse(simulationParameters.Buoyancy.dsigma_dx));
            var tension = Reverse(cumTrapz) + simulationParameters.Buoyancy.axialBuoyancyForceChangeOfDiameters - drag;

            Vector<double> fN_softstring = (Square((tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - simulationParameters.Buoyancy.Wb.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();

            Vector<double> I_fN_softstring = Utilities.CumTrapz(simulationParameters.LumpedCells.xL, fN_softstring);
            Vector<double> F_N_softstring = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)

            Vector<double> AiExtended = ExtendVectorStart(simulationParameters.Drillstring.InnerArea[0], simulationParameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(simulationParameters.Drillstring.OuterArea[0], simulationParameters.Drillstring.OuterArea);
            Vector<double> F_comp = AiExtended.PointwiseMultiply(simulationParameters.Buoyancy.stringPressure - simulationParameters.Buoyancy.hydrostaticStringPressure) * (1 - 2 * simulationParameters.Drillstring.PoissonRatio)
                                  - AoExtended.PointwiseMultiply(simulationParameters.Buoyancy.annularPressure - simulationParameters.Buoyancy.hydrostaticAnnularPressure) * (1 - 2 * simulationParameters.Drillstring.PoissonRatio)
                                  - tension;
            tension = -F_comp;

            // Update bending stiffness
            Vector<double> kbExtended = ExtendVectorStart(simulationParameters.Drillstring.BendingStiffness[0], simulationParameters.Drillstring.BendingStiffness); ;// Vector<double>.Build.DenseOfArray(new double[] { simulationParameters.kb[0] }.Concat(simulationParameters.kb).ToArray());
            Vector<double> kb = Vector<double>.Build.Dense(simulationParameters.Drillstring.BendingStiffness.Count + 1);

            kb = (kbExtended - Math.Pow(Math.PI, 2) * F_comp / (2 * simulationParameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);

            Vector<double> elementWiseProductJG = simulationParameters.Drillstring.PipePolarMoment.PointwiseMultiply(simulationParameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> TorqueMatrix = state.PipeShearStrain.PointwiseMultiply(scalingMatrix * elementWiseProductJG.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(TorqueMatrix.ToColumnMajorArray());
            Vector<double> torque = LinearInterpolate(simulationParameters.DistributedCells.x, torqueFlattened, simulationParameters.LumpedCells.xL);

            // Normal force components in Frenet-Serret coordinate system
            Vector<double> bzExtended = ExtendVectorStart(simulationParameters.Trajectory.bz[0], simulationParameters.Trajectory.bz);
            Vector<double> nzExtended = ExtendVectorStart(simulationParameters.Trajectory.nz[0], simulationParameters.Trajectory.nz);
            Vector<double> curvatureExtended = ExtendVectorStart(simulationParameters.Trajectory.curvature[0], simulationParameters.Trajectory.curvature);
            Vector<double> curvature_dotExtended = ExtendVectorStart(simulationParameters.Trajectory.curvature_dot[0], simulationParameters.Trajectory.curvature_dot);
            Vector<double> curvature_ddotExtended = ExtendVectorStart(simulationParameters.Trajectory.curvature_ddot[0], simulationParameters.Trajectory.curvature_ddot);
            Vector<double> EExtended = ExtendVectorStart(simulationParameters.Drillstring.YoungModuli[0], simulationParameters.Drillstring.YoungModuli);
            Vector<double> IExtended = ExtendVectorStart(simulationParameters.Drillstring.PipeInertia[0], simulationParameters.Drillstring.PipeInertia);
            Vector<double> torsionExtended = ExtendVectorStart(simulationParameters.Trajectory.torsion[0], simulationParameters.Trajectory.torsion);
            Vector<double> torsion_dotExtended = ExtendVectorStart(simulationParameters.Trajectory.torsion_dot[0], simulationParameters.Trajectory.torsion_dot);

            Vector<double> diffTorqueExtended = ExtendVectorStart(0, Diff(torque) / simulationParameters.LumpedCells.dxL);

            Vector<double> fB =
                simulationParameters.Buoyancy.Wb.PointwiseMultiply(bzExtended) +
                curvatureExtended.PointwiseMultiply(diffTorqueExtended) +
                curvature_dotExtended.PointwiseMultiply(torque) -
                2 * EExtended.PointwiseMultiply(IExtended).PointwiseMultiply(curvature_dotExtended).PointwiseMultiply(torsionExtended) -
                EExtended.PointwiseMultiply(IExtended).PointwiseMultiply(curvatureExtended).PointwiseMultiply(torsion_dotExtended);

            // Compute pre-stressed forces
            Vector<double> I_fB = CumTrapz(simulationParameters.LumpedCells.xL, fB);

            Vector<double> fN =
                curvatureExtended.PointwiseMultiply(tension + simulationParameters.Buoyancy.normalBuoyancyForceChangeOfDiameters) +
                simulationParameters.Buoyancy.Wb.PointwiseMultiply(nzExtended) -
                EExtended.PointwiseMultiply(IExtended).PointwiseMultiply(curvature_ddotExtended) +
                EExtended.PointwiseMultiply(IExtended).PointwiseMultiply(curvatureExtended).PointwiseMultiply(Square(torsionExtended)) -
                curvatureExtended.PointwiseMultiply(torsionExtended).PointwiseMultiply(torque);

            Vector<double> I_fN = CumTrapz(simulationParameters.LumpedCells.xL, fN);
            Vector<double> F_B_prestress = Diff(I_fB);
            Vector<double> F_N_prestress = Diff(I_fN);

            Vector<double> expression =
                (simulationParameters.Trajectory.hy.PointwiseMultiply(simulationParameters.Trajectory.nz) - (simulationParameters.Trajectory.hz.PointwiseMultiply(simulationParameters.Trajectory.ny))).PointwiseMultiply(simulationParameters.Trajectory.tx) +
                (simulationParameters.Trajectory.hz.PointwiseMultiply(simulationParameters.Trajectory.nx) - (simulationParameters.Trajectory.hx.PointwiseMultiply(simulationParameters.Trajectory.nz))).PointwiseMultiply(simulationParameters.Trajectory.ty) +
                (simulationParameters.Trajectory.hx.PointwiseMultiply(simulationParameters.Trajectory.ny) - (simulationParameters.Trajectory.hy.PointwiseMultiply(simulationParameters.Trajectory.nx))).PointwiseMultiply(simulationParameters.Trajectory.tz);

            //Vector<double> sign_tf = expression.Map(x => (double)Math.Sign(x));

            Vector<double> sign_tf = Vector<double>.Build.Dense(expression.Count);
            Vector<double> dotProduct = Vector<double>.Build.Dense(expression.Count);
            for (int i = 0; i < expression.Count; i++)
            {
                sign_tf[i] = Math.Sign(expression[i]);
                dotProduct[i] = simulationParameters.Trajectory.hx[i] * simulationParameters.Trajectory.nx[i] +
                                simulationParameters.Trajectory.hy[i] * simulationParameters.Trajectory.ny[i] +
                                simulationParameters.Trajectory.hz[i] * simulationParameters.Trajectory.nz[i];
                dotProduct[i] = Math.Max(-1, dotProduct[i]);
                dotProduct[i] = Math.Min(1, dotProduct[i]);
            }

            // Compute the toolface angle
            Vector<double> tf_angle = dotProduct.PointwiseAcos().PointwiseMultiply(sign_tf);

            Vector<double> Xc_iMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> Xc_iPlus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> Yc_iMinus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> Yc_iPlus1 = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);

            Vector<double> Xc_ddot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> Yc_ddot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);

            Vector<double> phi = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> phi_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> rc = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> r_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> phi_ddot_noslip = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> theta_ddot_noslip = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> OL_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> OS_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            // Force vectors
            Vector<double> Fc_t = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);//Tangential coulomb force
            Vector<double> Fc_a = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);//Axial coulomb force
            Vector<double> F_N = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);//Normal lateral collision force
            Vector<double> VL_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);//Lumped element axial acceleration
            Vector<double> inverted_slip_condition = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);

            Vector<double> Fstatic_t = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);
            Vector<double> Fstatic_a = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);

            double wb = 0.0;
            double tb = 0.0;

            Vector<double> Mb_x;//Bending Moments
            Vector<double> Mb_y;//Bending Moments

            // Solve lumped and distributed equations

            for (int innerIterationNo = 0; innerIterationNo < simulationParameters.InnerLoopIterations; innerIterationNo++)
            {
                double Vtd = simulationInput.CalculateSurfaceAxialVelocity; // Top drive axial velocity

                // Add the scalar to the front of the vector
                Vector<double> OL_vec = ExtendVectorStart(state.TopDriveAngularVelocity, state.LumpedElementAngularVelocity);
                Vector<double> VL_vec = ExtendVectorStart(Vtd, state.LumpedElementAxialVelocity);

                // Left boundaries
                Vector<double> downwardTorsionalWabeLeftBoundary = -upwardTorsionalWave.Row(0) + 2 * OL_vec.SubVector(0, OL_vec.Count - 1);
                Vector<double> upwardTorsionalWabeLeftBoundary = -upwardAxialWave.Row(0) + 2 * VL_vec.SubVector(0, VL_vec.Count - 1);

                // Right boundaries
                Vector<double> downwardAxialWabeRightBoundary = -downwardTorsionalWave.Row(simulationParameters.LumpedCells.PL - 1) + 2 * OL_vec.SubVector(1, OL_vec.Count - 1);
                Vector<double> upwardAxialWabeRightBoundary = -downwardAxialWave.Row(simulationParameters.LumpedCells.PL - 1) + 2 * VL_vec.SubVector(1, VL_vec.Count - 1);

                // Bit rock interaction
                // Bit velocity
                state.BitVelocity = 0.5 * (downwardAxialWave[simulationParameters.LumpedCells.PL - 1, downwardAxialWave.ColumnCount - 1] + upwardAxialWave[simulationParameters.LumpedCells.PL - 1, upwardAxialWave.ColumnCount - 1]);

                double angularVelocityBottom;
                if (!configuration.UseMudMotor)
                    angularVelocityBottom = 0.5 * (downwardTorsionalWave[simulationParameters.LumpedCells.PL - 1, downwardTorsionalWave.ColumnCount - 1] + upwardTorsionalWave[simulationParameters.LumpedCells.PL - 1, upwardTorsionalWave.ColumnCount - 1]);
                else
                    angularVelocityBottom = state.MudRotorAngularVelocity;


                double[] bitForces = simulationParameters.BitRock.CalculateInteractionForce(state, angularVelocityBottom, downwardTorsionalWave, simulationParameters);
                tb = bitForces[0];
                wb = bitForces[1];
              
                // manage the bit sticking off bottom condition
                if (!state.onBottom)
                {
                    double omega_ = state.LumpedElementAngularVelocity[state.LumpedElementAngularVelocity.Count - 1];
                    if (simulationInput.StickingBoolean)
                    {
                        int lastIndex = simulationParameters.Drillstring.ShearModuli.Count - 1;
                        Vector<double> a_t = upwardTorsionalWave.Row(simulationParameters.LumpedCells.PL - 1);
                        Vector<double> a_a = upwardAxialWave.Row(simulationParameters.LumpedCells.PL - 1);
                        tb = simulationParameters.Drillstring.PipePolarMoment[lastIndex] * simulationParameters.Drillstring.ShearModuli[lastIndex] / simulationParameters.Drillstring.TorsionalWaveSpeed * a_t[a_t.Count - 1];
                        wb = simulationParameters.Drillstring.PipeArea[lastIndex] * simulationParameters.Drillstring.YoungModuli[lastIndex] / simulationParameters.Drillstring.AxialWaveSpeed * a_a[a_t.Count - 1];
                    }
                    else
                    {
                        double normalForce_ = simulationInput.BottomExtraNormalForce;
                        if (normalForce_ > 0)
                        {
                            double ro_ = simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.OuterRadius.Count - 1];
                            double Fs_ = normalForce_ * 1.0;
                            double Fc_ = normalForce_ * 0.5;
                            double va_ = state.LumpedElementAxialVelocity[state.LumpedElementAxialVelocity.Count - 1];
                            double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_);
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / simulationParameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_ + 0.001 * 0.001);
                            if (Math.Abs(v_) < 1e-6)
                            {
                                tb = 0;
                                wb = 0;
                            }
                            else
                            {
                                tb = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                                wb = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            }
                        }
                        else
                        {
                            tb = 0;
                            wb = 0;
                        }
                    }
                }
                // Augment Riemann invariants with interface values
                var a_pad = downwardTorsionalWabeLeftBoundary.ToRowMatrix().Stack(downwardTorsionalWave);

                //  var rows = aa.RowCount + 1; // Extra row for aa_left
                //  var cols = aa.ColumnCount;

                // Create the final stacked matrix
                //  var a_pad = Matrix<double>.Build.Dense(rows, cols);
                //
                //  // Set the first row from aa_left
                //  a_pad.SetRow(0, aa_left.ToArray());
                //
                //  // Set the remaining rows from aa
                //  for (int i = 0; i < aa.RowCount; i++)
                //  {
                //      a_pad.SetRow(i + 1, aa.Row(i));
                //  }
                //
                //  // Create the final stacked matrix
                //  var u_pad = Matrix<double>.Build.Dense(rows, cols);
                //
                //  // Set the first row from aa_left
                //  u_pad.SetRow(0, uu_left.ToArray());
                //
                //  // Set the remaining rows from aa
                //  for (int i = 0; i < uu.RowCount; i++)
                //  {
                //      u_pad.SetRow(i + 1, uu.Row(i));
                //  }

                var b_pad = upwardTorsionalWave.Stack(downwardAxialWabeRightBoundary.ToRowMatrix());
                var u_pad = upwardTorsionalWabeLeftBoundary.ToRowMatrix().Stack(downwardAxialWave);
                var v_pad = upwardAxialWave.Stack(upwardAxialWabeRightBoundary.ToRowMatrix());
                // Stack a row vector bb_right on top of matrix bb
                // var b_pad = Matrix<double>.Build.Dense(rows, cols);
                // b_pad.SetRow(0, bb_right.ToArray());  // Set the first row from bb_right
                // for (int i = 0; i < bb.RowCount; i++)  // Set remaining rows from bb
                // {
                //     b_pad.SetRow(i + 1, bb.Row(i));
                // }
                // var v_pad = Matrix<double>.Build.Dense(rows, cols);
                //
                // v_pad.SetRow(0, vv_right.ToArray());  // Set the first row from bb_right
                // for (int i = 0; i < vv.RowCount; i++)  // Set remaining rows from bb
                // {
                //     v_pad.SetRow(i + 1, vv.Row(i));
                // }

                // Update states using Upwind scheme
                downwardTorsionalWave = downwardTorsionalWave - (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.dxM) * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(a_pad);
                upwardTorsionalWave = upwardTorsionalWave + (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.dxM) * simulationParameters.Drillstring.TorsionalWaveSpeed * DiffRows(b_pad);
                downwardAxialWave = downwardAxialWave - (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.dxM) * simulationParameters.Drillstring.AxialWaveSpeed * DiffRows(u_pad);
                upwardAxialWave = upwardAxialWave + (simulationParameters.InnerLoopTimeStep / simulationParameters.DistributedCells.dxM) * simulationParameters.Drillstring.AxialWaveSpeed * DiffRows(v_pad);

                // Compute torque from the top drive on the first distributed element
                double tauTD = (simulationParameters.Drillstring.PipePolarMoment[0] * simulationParameters.Drillstring.ShearModuli[0]) / (2 * simulationParameters.Drillstring.TorsionalWaveSpeed) * (downwardTorsionalWave[0, 0] - upwardTorsionalWave[0, 0]);

                // Compute torque for every other lumped element apart from top drive
                Vector<double> factor = simulationParameters.Drillstring.PipePolarMoment.PointwiseMultiply(simulationParameters.Drillstring.ShearModuli) / (2.0 * simulationParameters.Drillstring.TorsionalWaveSpeed);

                // Extract row pPL from aa and bb, then subtract
                Vector<double> rowDiff = downwardTorsionalWave.Row(simulationParameters.LumpedCells.PL - 1) - upwardTorsionalWave.Row(simulationParameters.LumpedCells.PL - 1);

                // Element-wise multiplication of factor with row difference
                Vector<double> tau_m = factor.PointwiseMultiply(rowDiff);

                // Compute temp array
                // Extract elements from index 1 to end (equivalent to MATLAB's 2:end)
                Vector<double> J_sub = simulationParameters.Drillstring.PipePolarMoment.SubVector(1, simulationParameters.Drillstring.PipePolarMoment.Count - 1);
                Vector<double> G_sub = simulationParameters.Drillstring.ShearModuli.SubVector(1, simulationParameters.Drillstring.ShearModuli.Count - 1);

                // Extract row 0 (first row) from columns 1 to end
                Vector<double> aa_sub = downwardTorsionalWave.Row(0).SubVector(1, downwardTorsionalWave.ColumnCount - 1);
                Vector<double> bb_sub = upwardTorsionalWave.Row(0).SubVector(1, upwardTorsionalWave.ColumnCount - 1);

                // Compute temp
                Vector<double> temp = J_sub.PointwiseMultiply(G_sub)
                                      .PointwiseMultiply(aa_sub - bb_sub)
                                      / (2 * simulationParameters.Drillstring.TorsionalWaveSpeed);

                double tm;
                // Create a new vector tau_p with a length equal to the sum of the lengths of temp and tb
                Vector<double> tau_p;
                if (!configuration.UseMudMotor)
                {
                    tm = 0;
                    tau_p = Vector<double>.Build.DenseOfArray(temp.Append(tb).ToArray());
                }
                else
                {
                    double speedRatio = (state.MudRotorAngularVelocity - state.MudStatorAngularVelocity) / simulationParameters.MudMotor.omega0_motor;

                    if (speedRatio <= 1)
                        tm = simulationParameters.MudMotor.T_max_motor * Math.Pow(1 - speedRatio, 1.0 / simulationParameters.MudMotor.alpha_motor);
                    else
                        tm = -simulationParameters.MudMotor.P0_motor * simulationParameters.MudMotor.V_motor * (speedRatio - 1);

                    tau_p = Vector<double>.Build.DenseOfArray(temp.Append(tm).ToArray());
                }


                Vector<double> factor2 = simulationParameters.Drillstring.PipeArea.PointwiseMultiply(simulationParameters.Drillstring.YoungModuli) / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed);
                Vector<double> rowDiff2 = downwardAxialWave.Row(simulationParameters.LumpedCells.PL - 1) - upwardAxialWave.Row(simulationParameters.LumpedCells.PL - 1);
                Vector<double> for_m = factor2.PointwiseMultiply(rowDiff2);

                Vector<double> A_sub = simulationParameters.Drillstring.PipeArea.SubVector(1, simulationParameters.Drillstring.PipeArea.Count - 1);
                Vector<double> E_sub = simulationParameters.Drillstring.YoungModuli.SubVector(1, simulationParameters.Drillstring.YoungModuli.Count - 1);
                Vector<double> uu_sub = downwardAxialWave.Row(0).SubVector(1, downwardAxialWave.ColumnCount - 1);
                Vector<double> vv_sub = upwardAxialWave.Row(0).SubVector(1, upwardAxialWave.ColumnCount - 1);
                Vector<double> tempVector = 1.0 / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed) * A_sub.PointwiseMultiply(E_sub).PointwiseMultiply(uu_sub - vv_sub);
                Vector<double> for_p = Vector<double>.Build.DenseOfArray(tempVector.Append(wb).ToArray());

                rc = (Square(state.Xc) + Square(state.Yc)).PointwiseSqrt();

                //for (int i = 0; i < state.Yc.Count; i++)
                //{
                //    rc[i] = Math.Sqrt(state.Xc[i] * state.Xc[i] + state.Yc[i] * state.Yc[i]);
                //}

                //phi = state.Yc.PointwiseAtan2(state.Xc);
                for (int i = 0; i < state.Yc.Count; i++)
                {
                    phi[i] = Math.Atan2(state.Yc[i], state.Xc[i]);
                }
                r_dot = state.Xc_dot.PointwiseMultiply(phi.PointwiseCos()) + state.Yc_dot.PointwiseMultiply(phi.PointwiseSin());


                // Compute iC = (rc >= simulationParameters.rc) (element-wise condition)
                Vector<double> iC = Vector<double>.Build.Dense(simulationParameters.Wellbore.rc.Count);
                for (int i = 0; i < rc.Count; i++)
                {
                    iC[i] = rc[i] >= simulationParameters.Wellbore.rc[i] ? 1.0 : 0.0;
                }

                // The wall is modeled as a spring-dashpot system with a spring constant simulationParameters.Wellbore.kw and a linear damping coefficient
                // simulationParameters.Wellbore.dw. The magnitude of the normal force then depends on the lateral displacement
                // of the drill string inside the borehole wall, which undergoes elastic deformation.
                F_N = simulationParameters.Wellbore.kw * (rc - simulationParameters.Wellbore.rc).PointwiseMultiply(iC) + simulationParameters.Wellbore.dw * r_dot.PointwiseMultiply(iC);
                if (F_N.Count > 1)
                    F_N[F_N.Count - 2] += simulationInput.ForceToInduceBitWhirl;


                phi_dot = (state.Yc_dot.PointwiseMultiply(state.Xc) - state.Xc_dot.PointwiseMultiply(state.Yc)).PointwiseDivide(Square(rc) + Constants.eps);

                Xc_iMinus1 = ExtendVectorStart(0, state.Xc.SubVector(0, state.Xc.Count - 1));
                Xc_iPlus1 = Vector<double>.Build.DenseOfArray(state.Xc.SubVector(1, state.Xc.Count - 1).Append(0).ToArray());
                Yc_iMinus1 = ExtendVectorStart(0, state.Yc.SubVector(0, state.Yc.Count - 1));
                Yc_iPlus1 = Vector<double>.Build.DenseOfArray(state.Yc.SubVector(1, state.Yc.Count - 1).Append(0).ToArray());

                // lateral forces due to bending            
                var kb1 = kb.SubVector(0, kb.Count - 1);
                var kb2 = kb.SubVector(1, kb.Count - 1);
                var Fk_x = -(kb1 + kb2).PointwiseMultiply(state.Xc) +
                            kb1.PointwiseMultiply(Xc_iMinus1) +
                            kb2.PointwiseMultiply(Xc_iPlus1);

                var Fk_y = -(kb1 + kb2).PointwiseMultiply(state.Yc) +
                            kb1.PointwiseMultiply(Yc_iMinus1) +
                            kb2.PointwiseMultiply(Yc_iPlus1);


                Vector<double> F_prestress_x = F_N_prestress.PointwiseMultiply(tf_angle.PointwiseSin()) +
                                               F_B_prestress.PointwiseMultiply(tf_angle.PointwiseCos());
                Vector<double> F_prestress_y = F_N_prestress.PointwiseMultiply(tf_angle.PointwiseCos()) -
                                               F_B_prestress.PointwiseMultiply(tf_angle.PointwiseSin());


                // lateral forces due to fluid-structure interaction
                Vector<double> Ff_x = -simulationParameters.Wellbore.Df * state.Xc_dot - simulationParameters.Drillstring.FluidAddedMass.PointwiseMultiply(state.LumpedElementAngularVelocity).PointwiseMultiply(state.Yc_dot) -
                                      simulationParameters.Wellbore.Df * state.LumpedElementAngularVelocity.PointwiseMultiply(0.5 * state.Yc) +
                                      simulationParameters.Drillstring.FluidAddedMass.PointwiseMultiply(Square(state.LumpedElementAngularVelocity)).PointwiseMultiply(1.0 / 4.0 * state.Xc);
                Vector<double> Ff_y = -simulationParameters.Wellbore.Df * state.Yc_dot + simulationParameters.Drillstring.FluidAddedMass.PointwiseMultiply(state.LumpedElementAngularVelocity).PointwiseMultiply(state.Xc_dot) +
                                      simulationParameters.Wellbore.Df * state.LumpedElementAngularVelocity.PointwiseMultiply(0.5 * state.Xc) +
                                      simulationParameters.Drillstring.FluidAddedMass.PointwiseMultiply(Square(state.LumpedElementAngularVelocity)).PointwiseMultiply(1.0 / 4.0 * state.Yc);

                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    Ff_x[index] = -simulationParameters.Wellbore.Df * state.Xc_dot[index] - simulationParameters.Drillstring.FluidAddedMass[index] * state.SleeveAngularVelocity[i] * state.Yc_dot[index]
                                  - simulationParameters.Wellbore.Df * state.SleeveAngularVelocity[i] * state.Yc[index] / 2.0 + simulationParameters.Drillstring.FluidAddedMass[index] * state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * state.Xc[index] / 4.0;

                    Ff_y[index] = -simulationParameters.Wellbore.Df * state.Yc_dot[index] + simulationParameters.Drillstring.FluidAddedMass[index] * state.SleeveAngularVelocity[i] * state.Xc_dot[index]
                                  + simulationParameters.Wellbore.Df * state.SleeveAngularVelocity[i] * state.Xc[index] / 2.0 + simulationParameters.Drillstring.FluidAddedMass[index] * state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * state.Yc[index] / 4.0;
                }

                // sleeve braking force
                Vector<double> F_S = Vector<double>.Build.Dense(simulationParameters.Drillstring.SleeveIndexPosition.Count());
                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    F_S[i] = simulationParameters.Drillstring.SleeveTorsionalDamping[i] * (state.LumpedElementAngularVelocity[index] - state.SleeveAngularVelocity[i]); // torsional damping
                }

                // Coulomb friction
                // V_t is the tangential slip velocity/ 
                Vector<double> V_a = Vector<double>.Build.Dense(state.LumpedElementAxialVelocity.Count());
                state.LumpedElementAxialVelocity.CopyTo(V_a);
                Vector<double> V_t = Vector<double>.Build.Dense(state.LumpedElementAxialVelocity.Count());
                V_t = phi_dot.PointwiseMultiply(rc) + state.LumpedElementAngularVelocity.PointwiseMultiply(simulationParameters.Drillstring.OuterRadius);
                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    V_t[index] = phi_dot[index] * rc[index] + state.SleeveAngularVelocity[i] * simulationParameters.Drillstring.SleeveOuterRadius; // replace with the sleeve angular velocity if there is a sleeve
                }

                // Compute forces required to enter static friction condition(i.e., no axial movement and rolling without slip)

                Fstatic_a = 1.0 / simulationParameters.InnerLoopTimeStep * state.LumpedElementAxialVelocity.PointwiseMultiply(simulationParameters.Drillstring.LumpedElementMass) + for_m - for_p - simulationParameters.Drillstring.CalculatedAxialDamping * state.LumpedElementAxialVelocity;

                theta_ddot_noslip = Vector<double>.Build.Dense(simulationParameters.Drillstring.LumpedElementMomentOfInertia.Count());
                var denominator = simulationParameters.Drillstring.LumpedElementMomentOfInertia + (simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass).PointwiseMultiply(Square(simulationParameters.Drillstring.OuterRadius)) -
                                  simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseMultiply(simulationParameters.Drillstring.OuterRadius).PointwiseMultiply((state.Theta - phi).PointwiseCos());
                var term1 = (simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass).PointwiseMultiply(simulationParameters.Drillstring.OuterRadius).PointwiseMultiply(r_dot).PointwiseMultiply(phi_dot);
                var term2 = simulationParameters.Drillstring.OuterRadius.PointwiseMultiply((Fk_x + F_prestress_x + Ff_x)).PointwiseMultiply(phi.PointwiseSin());
                var term3 = -simulationParameters.Drillstring.OuterRadius.PointwiseMultiply((Fk_y + F_prestress_y + Ff_y)).PointwiseMultiply(phi.PointwiseCos());
                var term4 = tau_m - tau_p - simulationParameters.Drillstring.CalculatedTorsionalDamping * state.LumpedElementAngularVelocity;
                var term5 = -simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseMultiply(simulationParameters.Drillstring.OuterRadius).PointwiseMultiply(Square(state.LumpedElementAngularVelocity)).PointwiseMultiply((state.Theta - phi).PointwiseSin());
                var numerator = term1 + term2 + term3 + term4 + term5;
                theta_ddot_noslip = numerator.PointwiseDivide(denominator);

                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];

                    double denominator__ = simulationParameters.Drillstring.SleeveMassMomentOfInertia + (simulationParameters.Drillstring.LumpedElementMass[index] + simulationParameters.Drillstring.FluidAddedMass[index]) * simulationParameters.Drillstring.SleeveOuterRadius * simulationParameters.Drillstring.SleeveOuterRadius
                     - simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] * simulationParameters.Drillstring.SleeveOuterRadius * Math.Cos(state.Theta_S[i] - phi[index]);
                    double term1__ = (simulationParameters.Drillstring.LumpedElementMass[index] + simulationParameters.Drillstring.FluidAddedMass[index]) * simulationParameters.Drillstring.SleeveOuterRadius * r_dot[index] * phi_dot[index];
                    double term2__ = simulationParameters.Drillstring.SleeveOuterRadius * (Fk_x[index] + F_prestress_x[index] + Ff_x[index]) * Math.Sin(phi[index]);
                    double term3__ = -simulationParameters.Drillstring.SleeveOuterRadius * (Fk_y[index] + F_prestress_y[index] + Ff_y[index]) * Math.Cos(phi[index]);
                    double term5__ = simulationParameters.Drillstring.SleeveInnerRadius * F_S[i] - simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] * simulationParameters.Drillstring.SleeveOuterRadius * state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * Math.Sin(state.Theta_S[i] - phi[index]);

                    var numerator__ = term1__ + term2__ + term3__ + term5__;
                    theta_ddot_noslip[index] = numerator__ / denominator__;
                }
                phi_ddot_noslip = Vector<double>.Build.Dense(simulationParameters.Drillstring.LumpedElementMomentOfInertia.Count());

                var denominator_ = simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass + simulationParameters.Drillstring.LumpedElementMomentOfInertia.PointwiseDivide(Square(simulationParameters.Drillstring.OuterRadius)) - (simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseDivide(simulationParameters.Drillstring.OuterRadius).PointwiseMultiply((state.Theta - phi).PointwiseCos()));
                var term1_ = (-2 * (simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass) - simulationParameters.Drillstring.LumpedElementMomentOfInertia.PointwiseDivide(Square(simulationParameters.Drillstring.OuterRadius)) + (simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseDivide(simulationParameters.Drillstring.OuterRadius).PointwiseMultiply((state.Theta - phi).PointwiseCos()))).PointwiseMultiply(r_dot).PointwiseMultiply(phi_dot);
                var term2_ = -(Fk_x + F_prestress_x + Ff_x).PointwiseMultiply(phi.PointwiseSin());
                var term3_ = (Fk_y + F_prestress_y + Ff_y).PointwiseMultiply(phi.PointwiseCos());
                var term4_ = -(tau_m - tau_p - simulationParameters.Drillstring.CalculatedTorsionalDamping * state.LumpedElementAngularVelocity).PointwiseDivide(simulationParameters.Drillstring.OuterRadius);
                var term5_ = simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseMultiply(Square(state.LumpedElementAngularVelocity)).PointwiseMultiply((state.Theta - phi).PointwiseSin());
                phi_ddot_noslip = (term1_ + term2_ + term3_ + term4_ + term5_).PointwiseDivide(denominator_);

                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    phi_ddot_noslip[index] = 1.0 / (simulationParameters.Drillstring.LumpedElementMass[index] + simulationParameters.Drillstring.FluidAddedMass[index] + simulationParameters.Drillstring.SleeveMassMomentOfInertia / Math.Pow(simulationParameters.Drillstring.SleeveOuterRadius, 2) - simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] / simulationParameters.Drillstring.SleeveOuterRadius * Math.Cos(state.Theta_S[i] - phi[index]))
                                     * ((-2 * (simulationParameters.Drillstring.LumpedElementMass[index] + simulationParameters.Drillstring.FluidAddedMass[index]) - simulationParameters.Drillstring.SleeveMassMomentOfInertia / Math.Pow(simulationParameters.Drillstring.SleeveOuterRadius, 2) +
                                     simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] / simulationParameters.Drillstring.SleeveOuterRadius * Math.Cos(state.Theta_S[i] - phi[index])) * r_dot[index] * phi_dot[index] -
                                     (Fk_x[index] + F_prestress_x[index] + Ff_x[index]) * Math.Sin(phi[index]) + (Fk_y[index] + F_prestress_y[index] + Ff_y[index]) * Math.Cos(phi[index]) -
                                     simulationParameters.Drillstring.SleeveInnerRadius / simulationParameters.Drillstring.SleeveOuterRadius * F_S[i] + simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] * state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * Math.Sin(state.Theta_S[i] - phi[index]));
                }

                Fstatic_t = Vector<double>.Build.Dense(simulationParameters.Drillstring.LumpedElementMomentOfInertia.Count());
                Fstatic_t = simulationParameters.Drillstring.LumpedElementMomentOfInertia.PointwiseDivide(Square(simulationParameters.Drillstring.OuterRadius)).PointwiseMultiply(r_dot.PointwiseMultiply(phi_dot) + rc.PointwiseMultiply(phi_ddot_noslip)) + (tau_m - tau_p - simulationParameters.Drillstring.CalculatedTorsionalDamping * state.LumpedElementAngularVelocity).PointwiseDivide(simulationParameters.Drillstring.OuterRadius);
                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    Fstatic_t[index] = (simulationParameters.Drillstring.SleeveMassMomentOfInertia / (simulationParameters.Drillstring.SleeveOuterRadius * simulationParameters.Drillstring.SleeveOuterRadius)) * (r_dot[index] * phi_dot[index] + rc[index] * phi_ddot_noslip[index]) + simulationParameters.Drillstring.SleeveInnerRadius / simulationParameters.Drillstring.SleeveOuterRadius * F_S[i];
                }

                Vector<double> VBar = (Square(V_a) + Square(V_t)).PointwiseSqrt() + Constants.eps;
                Vector<double> FResBar = (Square(Fstatic_a) + Square(Fstatic_t)).PointwiseSqrt() + Constants.eps;

                Vector<double> FcTh = F_N.PointwiseMultiply(simulationParameters.Friction.mu_s); // static force

                for (int j = 0; j < simulationParameters.LumpedCells.NL; j++)
                {
                    if (state.slip_condition[j] == 0)
                    {
                        if (FResBar[j] > FcTh[j])
                            state.slip_condition[j] = 1;
                        else
                            state.slip_condition[j] = 0;
                    }
                    else
                    {
                        if (VBar[j] < simulationParameters.Friction.v_c)
                        {
                            state.slip_condition[j] = 0;
                        }
                    }
                }

                //inverted_slip_condition = state.slip_condition.Map(x => x == 1.0 ? 0.0 : 1); // Equivalent to ~slip_condition(j) (There is no Vector<int> og Vector<bool> in MathNet

                for (int i = 0; i < state.slip_condition.Count; i++)
                {
                    inverted_slip_condition[i] = state.slip_condition[i] == 1.0 ? 0.0 : 1.0;
                }


                // Fc_a and Fc_t are axial and tangential components of the coloumb friction force between the borehole and the 
                // lumped element 
                Vector<double> Fc = inverted_slip_condition.PointwiseMultiply((FResBar.PointwiseMinimum(FcTh)).PointwiseMaximum(-1.0 * FcTh)) +
                    state.slip_condition.PointwiseMultiply(F_N).PointwiseMultiply(simulationParameters.Friction.mu_k + (simulationParameters.Friction.mu_s - simulationParameters.Friction.mu_k).PointwiseMultiply((-simulationParameters.Friction.stribeck * VBar).PointwiseExp()));
                Fc_a = inverted_slip_condition.PointwiseMultiply(Fc).PointwiseMultiply(Fstatic_a.PointwiseDivide(FResBar)) + state.slip_condition.PointwiseMultiply(Fc).PointwiseMultiply(V_a).PointwiseDivide(VBar);
                Fc_t = inverted_slip_condition.PointwiseMultiply(Fc).PointwiseMultiply(Fstatic_t.PointwiseDivide(FResBar)) + state.slip_condition.PointwiseMultiply(Fc).PointwiseMultiply(V_t).PointwiseDivide(VBar);

                //for the elements with sleeves, account for reduction in axial friction due to the spur wheels
                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    Fc_a[index] = Fc_a[index] * (1 - simulationParameters.Drillstring.AxialFrictionReduction);
                    Fc_t[index] = Math.Sqrt(Fc[index] * Fc[index] - Fc_a[index] * Fc_a[index]) * Math.Sign(Fc_t[index]);
                }

                // Solve Lumped ODEs -check for numerical instability (fix by decreasing simulationParameters.InnerLoopTimeStep, increasing I_L, M_L, or increasing simulationParameters.Drillstring.kt, simulationParameters.ka, simulationParameters.kl)
                state.TopDriveAngularVelocity = state.TopDriveAngularVelocity + 1.0 / simulationParameters.Wellbore.I_TD * simulationParameters.InnerLoopTimeStep * (simulationInput.TopDriveTorque - tauTD);

                OL_dot = (tau_m - tau_p - simulationParameters.Drillstring.CalculatedTorsionalDamping * state.LumpedElementAngularVelocity - Fc_t.PointwiseMultiply(simulationParameters.Drillstring.OuterRadius)).PointwiseDivide(simulationParameters.Drillstring.LumpedElementMomentOfInertia);
                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    OL_dot[index] = 1.0 / simulationParameters.Drillstring.LumpedElementMomentOfInertia[index] * (tau_m[index] - tau_p[index] - simulationParameters.Drillstring.CalculatedTorsionalDamping * state.LumpedElementAngularVelocity[index] - F_S[i] * simulationParameters.Drillstring.OuterRadius[index]);
                }

                // Extract specific elements: Fc_t(simulationParameters.Drillstring.iS)
                var result = Vector<double>.Build.Dense(simulationParameters.Drillstring.SleeveIndexPosition.Select(index => Fc_t[(int)index]).ToArray());

                OS_dot = 1.0 / simulationParameters.Drillstring.SleeveMassMomentOfInertia * simulationParameters.InnerLoopTimeStep * (F_S * simulationParameters.Drillstring.SleeveInnerRadius - simulationParameters.Drillstring.SleeveOuterRadius * result);
                VL_dot = (for_m - for_p - simulationParameters.Drillstring.CalculatedAxialDamping * state.LumpedElementAxialVelocity - Fc_a).PointwiseDivide(simulationParameters.Drillstring.LumpedElementMass);

                state.Theta = state.Theta + state.LumpedElementAngularVelocity * simulationParameters.InnerLoopTimeStep;
                state.Theta_S = state.Theta_S + state.SleeveAngularVelocity * simulationParameters.InnerLoopTimeStep;
                state.LumpedElementAngularVelocity = state.LumpedElementAngularVelocity + OL_dot * simulationParameters.InnerLoopTimeStep;
                state.SleeveAngularVelocity = state.SleeveAngularVelocity + OS_dot * simulationParameters.InnerLoopTimeStep;
                state.LumpedElementAxialVelocity = state.LumpedElementAxialVelocity + VL_dot * simulationParameters.InnerLoopTimeStep;

                // lateral forces due to mass imbalance
                // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center, 
                // which causes a lateral force and a torque as the pipe is displaced
                Vector<double> Fe_x = simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseMultiply(Square(state.LumpedElementAngularVelocity).PointwiseMultiply(state.Theta.PointwiseCos()) + OL_dot.PointwiseMultiply(state.Theta.PointwiseSin()));
                Vector<double> Fe_y = simulationParameters.Drillstring.EccentricMass.PointwiseMultiply(simulationParameters.Drillstring.Eccentricity).PointwiseMultiply(Square(state.LumpedElementAngularVelocity).PointwiseMultiply(state.Theta.PointwiseSin()) - OL_dot.PointwiseMultiply(state.Theta.PointwiseCos()));

                for (int i = 0; i < simulationParameters.Drillstring.SleeveIndexPosition.Count; i++)
                {
                    int index = (int)simulationParameters.Drillstring.SleeveIndexPosition[i];
                    Fe_x[index] = simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] * (state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * Math.Cos(state.Theta_S[i]) + OS_dot[i] * Math.Cos(state.Theta_S[i]));
                    Fe_y[index] = simulationParameters.Drillstring.EccentricMass[index] * simulationParameters.Drillstring.Eccentricity[index] * (state.SleeveAngularVelocity[i] * state.SleeveAngularVelocity[i] * Math.Sin(state.Theta_S[i]) - OS_dot[i] * Math.Sin(state.Theta_S[i]));
                }

                // lateral accelerations in cartesian coordinates
                // simulationParameters.Drillstring.kl is a viscous structural damping coeficcient for
                // F_N is the normal force
                // Fc_t is the tangential friction force
                // Ff_x and Ff_y are forces caused by the fluid-structure interaction
                // Fk_x and Fk_y are lateral forces due to bending
                // Fe_x and Fe_y are lateral forces due to mass imbalance
                Xc_ddot = (Fk_x + F_prestress_x + Ff_x + Fe_x - simulationParameters.Drillstring.CalculateLateralDamping * state.Xc_dot - F_N.PointwiseMultiply(phi.PointwiseCos()) + Fc_t.PointwiseMultiply(phi.PointwiseSin())).PointwiseDivide(simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass);
                Yc_ddot = (Fk_y + F_prestress_y + Ff_y + Fe_y - simulationParameters.Drillstring.CalculateLateralDamping * state.Yc_dot - F_N.PointwiseMultiply(phi.PointwiseSin()) - Fc_t.PointwiseMultiply(phi.PointwiseCos())).PointwiseDivide(simulationParameters.Drillstring.LumpedElementMass + simulationParameters.Drillstring.FluidAddedMass);

                state.Xc = state.Xc + state.Xc_dot * simulationParameters.InnerLoopTimeStep;
                state.Xc_dot = state.Xc_dot + Xc_ddot * simulationParameters.InnerLoopTimeStep;
                state.Yc = state.Yc + state.Yc_dot * simulationParameters.InnerLoopTimeStep;
                state.Yc_dot = state.Yc_dot + Yc_ddot * simulationParameters.InnerLoopTimeStep;

                // Mud motor
                if (configuration.UseMudMotor)
                {
                    state.MudStatorAngularVelocity = state.LumpedElementAngularVelocity[state.LumpedElementAngularVelocity.Count - 1];
                    state.MudRotorAngularVelocity = state.MudRotorAngularVelocity + simulationParameters.InnerLoopTimeStep / (simulationParameters.MudMotor.I_rotor + simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * Math.Pow(simulationParameters.MudMotor.N_rotor, 2)) *
                        (OL_dot[state.LumpedElementAngularVelocity.Count - 1] * simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * simulationParameters.MudMotor.N_stator * simulationParameters.MudMotor.N_rotor + tm - tb);
                }
            }

            // Compute states from Riemann invariants
            state.PipeAngularVelocity = 1.0 / 2.0 * (downwardTorsionalWave + upwardTorsionalWave);
            state.PipeShearStrain = 1.0 / (2.0 * simulationParameters.Drillstring.TorsionalWaveSpeed) * (downwardTorsionalWave - upwardTorsionalWave);

            state.PipeAxialVelocity = 1.0 / 2.0 * (downwardAxialWave + upwardAxialWave);
            state.PipeAxialStrain = 1.0 / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed) * (downwardAxialWave - upwardAxialWave);

            // Bending moments
            Mb_x = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeInertia).PointwiseMultiply(Xc_iPlus1 - 2 * state.Xc + Xc_iMinus1) / (simulationParameters.LumpedCells.dxL * simulationParameters.LumpedCells.dxL); // Bending moment x-component
            Mb_y = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeInertia).PointwiseMultiply(Yc_iPlus1 - 2 * state.Yc + Yc_iMinus1) / (simulationParameters.LumpedCells.dxL * simulationParameters.LumpedCells.dxL); // Bending moment y-component

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
                r_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] * Math.Cos(phi[simulationParameters.Drillstring.IndexSensor]) + Yc_ddot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(phi[simulationParameters.Drillstring.IndexSensor]) -
                                state.Xc_dot[simulationParameters.Drillstring.IndexSensor] * phi_dot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(phi[simulationParameters.Drillstring.IndexSensor]) +
                                state.Yc_dot[simulationParameters.Drillstring.IndexSensor] * phi_dot[simulationParameters.Drillstring.IndexSensor] * Math.Cos(phi[simulationParameters.Drillstring.IndexSensor]);

                phi_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * (1.0 / (Math.Pow(rc[simulationParameters.Drillstring.IndexSensor], 2) + Constants.eps) * (Yc_ddot[simulationParameters.Drillstring.IndexSensor] * state.Xc[simulationParameters.Drillstring.IndexSensor] -
                    Xc_ddot[simulationParameters.Drillstring.IndexSensor] * state.Yc[simulationParameters.Drillstring.IndexSensor]) - 2.0 * r_dot[simulationParameters.Drillstring.IndexSensor] * phi_dot[simulationParameters.Drillstring.IndexSensor] / (rc[simulationParameters.Drillstring.IndexSensor] + Constants.eps)) +
                    inverted_slip_condition[simulationParameters.Drillstring.IndexSensor] * phi_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                double phi0_sensor = 0;
                phi_ddot = 0.0;

                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor]); // radial position of accelerometer relative to pipe centerline
                    theta_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * OL_dot[simulationParameters.Drillstring.IndexSensor] + inverted_slip_condition[simulationParameters.Drillstring.IndexSensor] * theta_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                    u_x = state.Xc[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor]) - phi0_sensor * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor]);
                    u_y = state.Yc[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor]) + r0_sensor * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor]);
                    u_x_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor]) - OL_dot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor])) +
                        phi0_sensor * (Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor]) -
                        OL_dot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor]));
                    u_y_ddot = Yc_ddot[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor]) - OL_dot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor])) -
                        r0_sensor * (Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor]) - OL_dot[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor);
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.SleeveInnerRadius + simulationParameters.Drillstring.SleeveOuterRadius);

                    theta_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * OS_dot[idx_sleeve_sensor] + inverted_slip_condition[simulationParameters.Drillstring.IndexSensor] * theta_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                    u_x = state.Xc[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.Theta_S[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.Theta_S[idx_sleeve_sensor]);
                    u_y = state.Yc[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.Theta_S[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.Theta_S[idx_sleeve_sensor]);
                    u_x_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.Theta_S[idx_sleeve_sensor]) -
                        OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.Theta_S[idx_sleeve_sensor]) -
                        OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor]));
                    u_y_ddot = Yc_ddot[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.Theta_S[idx_sleeve_sensor]) - OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.Theta_S[idx_sleeve_sensor]) - OS_dot[idx_sleeve_sensor] * Math.Cos(state.Theta_S[idx_sleeve_sensor]));
                }
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor - 1))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor - 1] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_iMinus1 = state.Xc[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) - phi0_sensor * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_y_iMinus1 = state.Yc[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) + r0_sensor * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_ddot_iMinus1 = Xc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) -
                        OL_dot[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1])) + phi0_sensor * (Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) -
                        OL_dot[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]));
                    u_y_ddot_iMinus1 = Yc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) - OL_dot[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1])) -
                       r0_sensor * (Math.Pow(state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]) - OL_dot[simulationParameters.Drillstring.IndexSensor - 1] * Math.Cos(state.Theta[simulationParameters.Drillstring.IndexSensor - 1]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor - 1);
                    u_x_iMinus1 = state.Xc[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.Theta_S[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.Theta_S[idx_sleeve_sensor]);
                    u_y_iMinus1 = state.Yc[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.Theta_S[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.Theta_S[idx_sleeve_sensor]);
                    u_x_ddot_iMinus1 = Xc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.Theta_S[idx_sleeve_sensor]) -
                        OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.Theta_S[idx_sleeve_sensor]) -
                        OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor]));
                    u_y_ddot_iMinus1 = Yc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.Theta_S[idx_sleeve_sensor]) - OS_dot[idx_sleeve_sensor] * Math.Sin(state.Theta_S[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.Theta_S[idx_sleeve_sensor]) - OS_dot[idx_sleeve_sensor] * Math.Cos(state.Theta_S[idx_sleeve_sensor]));
                }

                // Bending angles
                Theta_x = -(u_y - u_y_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle x-component
                Theta_y = (u_x - u_x_iMinus1) / simulationParameters.LumpedCells.dxL;// Bending angle y-component
                Theta_x_ddot = -(u_y_ddot - u_y_ddot_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle second derivative x-component
                Theta_y_ddot = (u_x_ddot - u_x_ddot_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle second derivative y-component
            }

            output.BitVelocity = state.PipeAxialVelocity[state.PipeAxialVelocity.RowCount - 1, state.PipeAxialVelocity.ColumnCount - 1]; // Bit velocity

            // Parse outputs
            output.NormalForceProfileStiffString = F_N; // Pipe shear strain 
            output.NormalForceProfileSoftString = F_N_softstring;
            output.TensionProfile = tension; // Tension profile

            var tempMatrix = state.PipeShearStrain.PointwiseMultiply(scalingMatrix * elementWiseProductJG.ToRowMatrix()); // 5x136 matrix
            output.Torque = ToVector(tempMatrix.ToColumnMajorArray()); // Torque profile vs. depth
            output.BendingMomentX = Mb_x;// Bending moment x-component profile
            output.BendingMomentY = Mb_y;// Bending moment y-component profile
            output.TangentialForceProfile = Fc_t;// Bending moment y-component profile Tangential force profile
            output.WeightOnBit = wb;  // Weight on bit
            output.TorqueOnBit = tb;  // Torque on bit
            output.Depth = simulationParameters.LumpedCells.xL;
            output.SensorMb_x = Mb_x[simulationParameters.Drillstring.IndexSensor];
            output.SensorMb_y = Mb_y[simulationParameters.Drillstring.IndexSensor];
            output.RadialDisplacement = rc;
            output.WhirlAngle = phi;
            output.WhirlSpeed = phi_dot;
            output.BendingMoment = (Square(Mb_x) + Square(Mb_y)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

            if (configuration.UsePipeMovementReconstruction)
            {
                output.SensorAxialVelocity = state.LumpedElementAxialVelocity[simulationParameters.Drillstring.IndexSensor]; // sleeve angular displacement;
                output.SensorAxialDisplacement = output.SensorAxialDisplacement + output.SensorAxialVelocity * configuration.TimeStep;
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor)) //sleeve angular velocity
                {
                    output.SensorAngularPosition = state.Theta[simulationParameters.Drillstring.IndexSensor]; //pipe angular displacement
                    output.SensorAngularVelocity = state.LumpedElementAngularVelocity[simulationParameters.Drillstring.IndexSensor]; //pipe angular velocity
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor - 1);
                    output.SensorAngularPosition = state.Theta_S[idx_sleeve_sensor];
                    output.SensorAngularVelocity = state.SleeveAngularVelocity[idx_sleeve_sensor];
                }
                output.SensorRadialPosition = rc[simulationParameters.Drillstring.IndexSensor]; //radial position
                output.SensorWhirlAngle = phi[simulationParameters.Drillstring.IndexSensor]; //whirl angle
                output.SensorRadialSpeed = r_dot[simulationParameters.Drillstring.IndexSensor]; //radial velocity
                output.SensorWhirlSpeed = phi_dot[simulationParameters.Drillstring.IndexSensor]; //whirl velocity
                output.SensorAxialAcceleration = VL_dot[simulationParameters.Drillstring.IndexSensor]; //axial acceleration
                output.SensorAngularAcceleration = theta_ddot; // angular acceleration
                output.SensorRadialAcceleration = r_ddot; // radial acceleration
                output.SensorWhirlAcceleration = phi_ddot; // whirl acceleration
                output.SensorBendingAngleX = Theta_x; // bending angle x-component
                output.SensorBendingAngleY = Theta_y; // bending angle y-component
                output.SecondDerivativeSensorBendingAngleX = Theta_x_ddot; // bending angle second derivative x-component
                output.SecondDerivativeSensorBendingAngleY = Theta_y_ddot; // bending angle second derivative y-component
                output.SensorPipeInclination = simulationParameters.Trajectory.thetaVec[simulationParameters.Drillstring.IndexSensor];
                output.SensorPipeAzimuthAt = simulationParameters.Trajectory.phiVec[simulationParameters.Drillstring.IndexSensor];
                output.SensorTension = tension[simulationParameters.Drillstring.IndexSensor];
                output.SensorTorque = output.Torque[simulationParameters.Drillstring.IndexSensor];

                CalculateInLocalFrame();
            }
        }

        public void AddNewLumpedElement() // TODO sjekk
        {
            simulationParameters.AddNewLumpedElement();

            state.AddNewLumpedElement();
        }
    }
}
