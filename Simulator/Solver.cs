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
                simulationParameters.LumpedCells.xL.SetSubVector(1, simulationParameters.LumpedCells.xL.Count() - 1, simulationParameters.LumpedCells.xL.SubVector(1, simulationParameters.LumpedCells.xL.Count - 1) + state.AxialVelocity * configuration.TimeStep);

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

            double dtTemp = simulationParameters.DistributedCells.dxM / Math.Max(simulationParameters.Drillstring.TorsionalWaveSpeed, simulationParameters.Drillstring.AxialWaveSpeed) * .4;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
            
            // Wave equations are transformed into their Riemann invariants
            AccelerationCalculation.PrepareAxialTorsional(axialTorsionalModel, state, simulationParameters);            
            AccelerationCalculation.PreprareLateral(lateralModel, state, simulationParameters);            
            
            
            // Force vectors
            

            Vector<double> Mb_x;//Bending Moments
            Vector<double> Mb_y;//Bending Moments
            
            // Solve lumped and distributed equations

            for (int innerIterationNo = 0; innerIterationNo < simulationParameters.InnerLoopIterations; innerIterationNo++)
            {
                // Calculate axial-torsional pde properties
                AccelerationCalculation.AxialTorsionalSystem(axialTorsionalModel, simulationInput, configuration, state, simulationParameters);                 
                // Update axial-torsional state using upwind scheme            
                UpwindScheme.IntegrationStep(axialTorsionalModel, simulationParameters);
                // Calculate lateral accelerations    
                AccelerationCalculation.LateralSystem(lateralModel, axialTorsionalModel, simulationInput, configuration, state, simulationParameters);
                
                //  Joined everything together inside a single loop because it facilitates future
                // parallelization
                for (int i = 0; i < state.XDisplacement.Count; i++)
                {
                    #region Polar Coordinates Conversion
                    // Get the radial displacement 
                    double radialDisplacement = Math.Sqrt(state.XDisplacement[i] * state.XDisplacement[i] + state.YDisplacement[i] * state.YDisplacement[i]);   
                    // Calculate the whirl angle 
                    double whirlAngle = Math.Atan2(state.YDisplacement[i], state.XDisplacement[i]);
                    // Pre-calculate whirl angle sin and cos to avoid multiple calculations
                    double cosWhirlAngle = whirlAngle == 0.0 ? 1.0 : state.XDisplacement[i]/radialDisplacement; // Math.Cos(whirlAngle);
                    double sinWhirlAngle = whirlAngle == 0.0 ? 0.0 : state.YDisplacement[i]/radialDisplacement; // Math.Sin(whirlAngle);
                    // Calculate the radial velocity
                    double radialVelocity = state.XVelocity[i] * cosWhirlAngle 
                        + state.YVelocity[i] * sinWhirlAngle;
                    // Calculate whirl velocity
                    double whirlVelocity = (state.YVelocity[i] * state.XDisplacement[i] - state.XVelocity[i] * state.YDisplacement[i])/(radialDisplacement * radialDisplacement + Constants.eps);
                    #endregion
                    #region Collision calculation
                    // Check if there is collision or not and store the Heaveside Step Function
                    double HeavesideStep = radialDisplacement >= simulationParameters.Wellbore.rc[i] ? 1.0 : 0.0;                                    
                    // Calculate normal force
                    double normalCollisionForce =  HeavesideStep * 
                        (
                            simulationParameters.Wellbore.kw * (radialDisplacement - simulationParameters.Wellbore.rc[i]) 
                           + simulationParameters.Wellbore.dw * radialVelocity
                        );  
                                            
                    if (lateralModel.NormalCollisionForce.Count > 1 && i == lateralModel.NormalCollisionForce.Count - 2)
                        normalCollisionForce += simulationInput.ForceToInduceBitWhirl;    
                    #endregion
                    #region Elastic Force Calculation
                    // If it is the first element, get the pinned boundary condition
                    double XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                    double YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                    // If it is the last element, get the pinned boundary condition
                    double XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                    double YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                    // Same with the stiffness
                    double kMinus1 = (i == 0) ? 1E8 : lateralModel.BendingStiffness[i - 1];
                    double kPlus1 = (i == state.XDisplacement.Count - 1) ? 1E8 : lateralModel.BendingStiffness[i + 1];
                    double ElasticForceX = kMinus1 * XiMinus1  - (kMinus1 + kPlus1) * state.XDisplacement[i] + kPlus1 * XiPlus1;
                    double ElasticForceY = kMinus1 * YiMinus1  - (kMinus1 + kPlus1) * state.YDisplacement[i] + kPlus1 * YiPlus1;
                    double PreStressForceX = lateralModel.PreStressNormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) + 
                                    lateralModel.PreStressBinormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) ;
                    double PreStressForceY = lateralModel.PreStressNormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) - 
                                    lateralModel.PreStressBinormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) ;                                    
                    #endregion
                    #region Fluid Force Calculation
                    //Extracts angular speed depending on if it is a sleeve or not
                    bool hasSleeve = state.SleeveToLumpedIndex[i] != -1;
                    //If it is not used properly, it should crash the code.
                    int sleeveIndex = hasSleeve ?  state.SleeveToLumpedIndex[i] : -1;
                    //Select between sleeve and non-sleeve nodes
                    double rotationSpeed = hasSleeve ? state.SleeveAngularVelocity[sleeveIndex] : state.AngularVelocity[i];                            
                    double rotationSpeedSquared = rotationSpeed * rotationSpeed;
                    double fluidDampingCoefficient = simulationParameters.Wellbore.Df / simulationParameters.Drillstring.FluidAddedMass[i];

                    double fluidForceX = - simulationParameters.Drillstring.FluidAddedMass[i] * 
                                        (
                                            fluidDampingCoefficient * state.XVelocity[i]
                                            - 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                            + rotationSpeed * state.YVelocity[i]
                                            + 0.5 * fluidDampingCoefficient * rotationSpeed * state.YDisplacement[i]
                                        );
                    double fluidForceY = - simulationParameters.Drillstring.FluidAddedMass[i] * 
                                        (
                                            fluidDampingCoefficient * state.YVelocity[i]
                                            - 0.25 * rotationSpeedSquared * state.YDisplacement[i]
                                            - rotationSpeed * state.XVelocity[i]
                                            - 0.5 * fluidDampingCoefficient * rotationSpeed * state.XDisplacement[i]
                                        );
                    #endregion
                    // Sleeve braking force
                    double sleeveBrakeForce = hasSleeve ? simulationParameters.Drillstring.SleeveTorsionalDamping[sleeveIndex] * (whirlVelocity - state.SleeveAngularVelocity[sleeveIndex]) : 0;
                    #region Unbalance Forces
                    // lateral forces due to mass imbalance
                    // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center, 
                    // which causes a lateral force and a torque as the pipe is displaced                
                    double rotationAngle = hasSleeve ? state.SleeveAngularDisplacement[sleeveIndex] : state.AngularDisplacement[i];                                        
                    double rotationAcceleration = hasSleeve ? state.SleeveAngularAcceleration[sleeveIndex] : state.AngularAcceleration[i];                                        
                    double unbalanceForceX = simulationParameters.Drillstring.EccentricMass[i] * simulationParameters.Drillstring.Eccentricity[i]*
                        (
                            rotationSpeedSquared * Math.Cos(rotationAngle)
                            + state.AngularAcceleration[i] * Math.Sin(rotationAngle)
                        );
                    double unbalanceForceY = simulationParameters.Drillstring.EccentricMass[i] * simulationParameters.Drillstring.Eccentricity[i]* 
                        (
                            rotationSpeedSquared * Math.Sin(rotationAngle)
                            - rotationAcceleration * Math.Cos(rotationAngle)
                        );                                                            
                    #endregion
                    
                    
                    #region Coulomb Friction
                    // Axial velocity
                    double axialVelocity = state.AxialVelocity[i];
                    // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                    double outerRadius = hasSleeve ? simulationParameters.Drillstring.SleeveOuterRadius : simulationParameters.Drillstring.OuterRadius[i];
                    double innerRadius = hasSleeve ? simulationParameters.Drillstring.SleeveInnerRadius : simulationParameters.Drillstring.OuterRadius[i];  
                    double inertia = hasSleeve ? simulationParameters.Drillstring.SleeveMassMomentOfInertia : simulationParameters.Drillstring.LumpedElementMomentOfInertia[i];
                    double tangentialVelocity = whirlVelocity * radialDisplacement + rotationSpeed * outerRadius;
                    double axialStaticForce = 1.0 / simulationParameters.InnerLoopTimeStep * state.AxialVelocity[i] * simulationParameters.Drillstring.LumpedElementMass[i] 
                            + lateralModel.ForceM[i] - lateralModel.ForceDistribution[i] - simulationParameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i];                    
                    double sumForcesX = ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX;
                    double sumForcesY = ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY;

                    double[] totalForce = new double[3] { sumForcesX, sumForcesY, axialStaticForce }; 
                    double[] normalVector = new double[3] { cosWhirlAngle, sinWhirlAngle, 0 };
                    
                    
                    // ----------------------------- Needs to be corrected! ---------------------------- 
                    // The tangential velocity must be the difference of the total velocity from the normal velocity.
                    // The total velocity is calculated by [dx, dy, dz] + omega x R.
                    // The normal velocity is the projection of the total on the direction of the collision [cos(whirlAngle); sin(whirlAngle); 0]                                                                                                    
                    double term1 = (
                                    -2 * (simulationParameters.Drillstring.LumpedElementMass[i] 
                                    + simulationParameters.Drillstring.FluidAddedMass[i]) 
                                    - inertia / (outerRadius * outerRadius) 
                                    + simulationParameters.Drillstring.EccentricMass[i] 
                                    * simulationParameters.Drillstring.Eccentricity[i] 
                                    / outerRadius * Math.Cos(state.AngularDisplacement[i] - whirlAngle)
                                ) 
                                * radialVelocity * whirlVelocity;                   
                    double term2 = outerRadius * (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                    double term3 = - outerRadius * (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;                                        
                    double term4 = - (
                                          lateralModel.TauM[i] 
                                        - lateralModel.TorqueDistribution[i]
                                        - simulationParameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed
                                    ) / outerRadius;
                    double term5 = simulationParameters.Drillstring.EccentricMass[i] 
                                    * simulationParameters.Drillstring.Eccentricity[i] 
                                    * rotationSpeed 
                                    * rotationSpeed 
                                    * Math.Sin(rotationAngle - whirlAngle);
                    double term6 = hasSleeve ? sleeveBrakeForce * innerRadius : 0.0;
                    double denominator = simulationParameters.Drillstring.LumpedElementMass[i] 
                                        + simulationParameters.Drillstring.FluidAddedMass[i] 
                                        + inertia / (outerRadius * outerRadius) 
                                        - (
                                            simulationParameters.Drillstring.EccentricMass[i] 
                                            * simulationParameters.Drillstring.Eccentricity[i] 
                                            / simulationParameters.Drillstring.OuterRadius[i] 
                                            * Math.Cos(rotationAngle - whirlAngle)
                                        );
                    double thetaDotNoSlip = (term1 + term2 + term3 + term4 + term5 + term6) / denominator;
                    double phiDdotNoSlip = 0.0;      

                    if (!hasSleeve)
                    {                        
                        double denominator_ = simulationParameters.Drillstring.LumpedElementMass[i] 
                                        + simulationParameters.Drillstring.FluidAddedMass[i] 
                                        + inertia / (outerRadius * outerRadius) 
                                        - (simulationParameters.Drillstring.EccentricMass[i] * simulationParameters.Drillstring.Eccentricity[i]
                                         / outerRadius * Math.Cos(rotationAngle - whirlAngle));
                        double term1_ = (-2 * (simulationParameters.Drillstring.LumpedElementMass[i] + simulationParameters.Drillstring.FluidAddedMass[i]) 
                                            - inertia / (outerRadius * outerRadius) 
                                            + simulationParameters.Drillstring.EccentricMass[i] * simulationParameters.Drillstring.Eccentricity[i] 
                                            / outerRadius 
                                            * Math.Cos(rotationAngle - whirlAngle)) 
                                            * radialVelocity 
                                            * whirlVelocity;
                        double term2_ = - (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                        double term3_ =   (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;
                        double term4_ = - (
                                            lateralModel.TauM[i] 
                                            - lateralModel.TorqueDistribution[i]
                                            - simulationParameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed)
                                            / outerRadius;
                        double term5_ = simulationParameters.Drillstring.EccentricMass[i] 
                                        * simulationParameters.Drillstring.Eccentricity[i]
                                        * state.AngularVelocity[i] 
                                        * state.AngularVelocity[i] * Math.Sin(rotationAngle - whirlAngle);              
                        phiDdotNoSlip = (term1_ + term2_ + term3_ + term4_ + term5_)/denominator_;
                    }
                    else
                    {                        
                        double denominator_ = simulationParameters.Drillstring.LumpedElementMass[i]
                                             + simulationParameters.Drillstring.FluidAddedMass[i] 
                                             + inertia / (outerRadius*outerRadius) 
                                             - simulationParameters.Drillstring.EccentricMass[i] 
                                             * simulationParameters.Drillstring.Eccentricity[i] / outerRadius 
                                             * Math.Cos(rotationAngle - whirlAngle);

                        double term1_ = (
                                        - 2 * (simulationParameters.Drillstring.LumpedElementMass[i] + simulationParameters.Drillstring.FluidAddedMass[i]) 
                                        - inertia / (outerRadius * outerRadius) 
                                        + simulationParameters.Drillstring.EccentricMass[i] * simulationParameters.Drillstring.Eccentricity[i] 
                                        / outerRadius * Math.Cos(rotationAngle - whirlAngle))
                                        * radialVelocity 
                                        * whirlVelocity;
                        double term2_ = - (ElasticForceX + PreStressForceX + fluidForceX) * sinWhirlAngle;
                        double term3_ =   (ElasticForceY + PreStressForceY + fluidForceY) * cosWhirlAngle;
                        double term4_ = - innerRadius / outerRadius * sleeveBrakeForce;
                        double term5_ = simulationParameters.Drillstring.EccentricMass[i] 
                                        * simulationParameters.Drillstring.Eccentricity[i]
                                        * rotationSpeedSquared * Math.Sin(rotationAngle - whirlAngle);
                        phiDdotNoSlip = (term1 + term2 + term3 + term4 + term5) / denominator;                                                            
                    }
                    //Tangetial force
                    double tangentialStaticForce = inertia/outerRadius * radialVelocity * whirlVelocity
                        + radialDisplacement * phiDdotNoSlip 
                        + (
                            lateralModel.TauM[i] - lateralModel.TorqueDistribution[i] 
                            - simulationParameters.Drillstring.CalculatedTorsionalDamping * whirlVelocity
                        )/outerRadius;
                  
                    double velocityMagnitude = Math.Sqrt(axialVelocity * axialVelocity  + tangentialVelocity * tangentialVelocity) + Constants.eps;
                    double staticFrictionForceCalculated = Math.Sqrt(axialStaticForce * axialStaticForce + tangentialStaticForce * tangentialStaticForce) + Constants.eps;
                    double staticFrictionForceCriteria = normalCollisionForce * simulationParameters.Friction.mu_s[i];
                    //Check for slip
                    if (state.slip_condition[i] == 0)
                        // If the previous state was a not a slip condition, re-evaluate by comparing forces.  
                        state.slip_condition[i] = staticFrictionForceCalculated > staticFrictionForceCriteria ? 1:0;
                    else
                        // If the previous state was a slip condition  
                        state.slip_condition[i] = velocityMagnitude < simulationParameters.Friction.v_c ? 0 : state.slip_condition[i];
                    //Masking system for no slip
                    double invertedSlipCondition = 1.0 - state.slip_condition[i];
                    double coulombForce;
                    double axialCoulombFrictionForce;
                    double tangentialCoulombFrictionForce;

                    if (state.slip_condition[i] == 0)
                    {
                        coulombForce = Math.Max(Math.Min(staticFrictionForceCalculated, staticFrictionForceCriteria), - staticFrictionForceCriteria);
                        axialCoulombFrictionForce = coulombForce * axialStaticForce / staticFrictionForceCalculated;
                        tangentialCoulombFrictionForce = coulombForce * tangentialStaticForce / staticFrictionForceCalculated;
                    }
                    else
                    {
                        coulombForce = normalCollisionForce * (simulationParameters.Friction.mu_k[i] + (simulationParameters.Friction.mu_s[i] - simulationParameters.Friction.mu_k[i]) * Math.Exp( - simulationParameters.Friction.stribeck *  velocityMagnitude));
                        axialCoulombFrictionForce = coulombForce * axialVelocity / velocityMagnitude;
                        tangentialCoulombFrictionForce = coulombForce * tangentialVelocity / velocityMagnitude;                        
                    }
                    if (hasSleeve)
                    {
                        axialCoulombFrictionForce = axialCoulombFrictionForce *  (1 - simulationParameters.Drillstring.AxialFrictionReduction);
                        tangentialCoulombFrictionForce = Math.Sqrt(coulombForce * coulombForce - axialCoulombFrictionForce * axialCoulombFrictionForce) * Math.Sign (tangentialCoulombFrictionForce);
                    }                    
                    #endregion                   
                    #region Accelerations
                    //If there is a sleeve, no torque is actually used
                    double frictionTorque = hasSleeve ? sleeveBrakeForce * outerRadius : tangentialCoulombFrictionForce * outerRadius;
                    state.AngularAcceleration[i] = (
                                    lateralModel.TauM[i]
                                     - lateralModel.TorqueDistribution[i] 
                                     - simulationParameters.Drillstring.CalculatedTorsionalDamping * whirlVelocity
                                     - frictionTorque
                                )/inertia;

                    // It needs to be zeroed before, as there might have changes in the sleeve position when the number of lumped parameters change 
                    state.SleeveForces[i] = 0;
                    if (hasSleeve)
                    {
                        state.SleeveForces[i] = hasSleeve ? tangentialCoulombFrictionForce : 0;
                        // Why is there a TimeStep in here?                    
                        state.SleeveAngularAcceleration[sleeveIndex] = simulationParameters.InnerLoopTimeStep * (sleeveBrakeForce * simulationParameters.Drillstring.SleeveInnerRadius - simulationParameters.Drillstring.SleeveOuterRadius * state.SleeveForces[i]) / simulationParameters.Drillstring.SleeveMassMomentOfInertia;;
                    }
                    //Used for debugging purposes
                    lateralModel.NormalCollisionForce[i] = normalCollisionForce;

                    state.RadialDisplacement[i] = radialDisplacement;
                    state.RadialVelocity[i] = radialVelocity;
                    state.WhirlAngle[i] = whirlAngle;
                    state.WhirlVelocity[i] = whirlVelocity;
                    double ZAcceleration = (lateralModel.ForceM[i] - lateralModel.ForceDistribution[i] - simulationParameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i] - axialCoulombFrictionForce)/simulationParameters.Drillstring.LumpedElementMass[i];
                    double XAcceleration = (ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - simulationParameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - lateralModel.NormalCollisionForce[i] * cosWhirlAngle
                        + tangentialCoulombFrictionForce * sinWhirlAngle) / (simulationParameters.Drillstring.LumpedElementMass[i] + simulationParameters.Drillstring.FluidAddedMass[i]);
                    double YAcceleration= (ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - simulationParameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - lateralModel.NormalCollisionForce[i] * sinWhirlAngle
                        - tangentialCoulombFrictionForce * cosWhirlAngle) / (simulationParameters.Drillstring.LumpedElementMass[i] + simulationParameters.Drillstring.FluidAddedMass[i]);

                    state.AxialAcceleration[i] = ZAcceleration;
                    state.XAcceleration[i] = XAcceleration;
                    state.YAcceleration[i] = YAcceleration;
                    


                    #endregion
                }                                        
                //TO BE KEPT SO FAR
                // Solve Lumped ODEs -check for numerical instability (fix by decreasing simulationParameters.InnerLoopTimeStep, increasing I_L, M_L, or increasing simulationParameters.Drillstring.kt, simulationParameters.ka, simulationParameters.kl)
                state.TopDriveAngularVelocity = state.TopDriveAngularVelocity + 1.0 / simulationParameters.Wellbore.I_TD * simulationParameters.InnerLoopTimeStep * (simulationInput.TopDriveTorque - lateralModel.TauTD);                
                
                //NumericalIntegration
                //Torsional DoF
                state.AngularDisplacement = state.AngularDisplacement + state.AngularVelocity * simulationParameters.InnerLoopTimeStep;
                state.AngularVelocity     = state.AngularVelocity + state.AngularAcceleration * simulationParameters.InnerLoopTimeStep;
                //Sleeve DoF
                state.SleeveAngularDisplacement = state.SleeveAngularDisplacement + state.SleeveAngularVelocity * simulationParameters.InnerLoopTimeStep;
                state.SleeveAngularVelocity = state.SleeveAngularVelocity + state.SleeveAngularAcceleration * simulationParameters.InnerLoopTimeStep;
                //Axial DoF
                state.AxialVelocity = state.AxialVelocity + state.AxialAcceleration * simulationParameters.InnerLoopTimeStep;
                //X DoF
                state.XDisplacement = state.XDisplacement + state.XVelocity * simulationParameters.InnerLoopTimeStep;
                state.XVelocity = state.XVelocity + state.XAcceleration * simulationParameters.InnerLoopTimeStep;
                //Y DoF
                state.YDisplacement = state.YDisplacement + state.YVelocity * simulationParameters.InnerLoopTimeStep;
                state.YVelocity = state.YVelocity + state.YAcceleration * simulationParameters.InnerLoopTimeStep;

                // Mud motor
                if (configuration.UseMudMotor)
                {
                    state.MudStatorAngularVelocity = state.WhirlVelocity[state.WhirlVelocity.Count - 1];
                    state.MudRotorAngularVelocity = state.MudRotorAngularVelocity + simulationParameters.InnerLoopTimeStep / (simulationParameters.MudMotor.I_rotor + simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * Math.Pow(simulationParameters.MudMotor.N_rotor, 2)) *
                        (state.AngularAcceleration[state.WhirlVelocity.Count - 1] * simulationParameters.MudMotor.M_rotor * Math.Pow(simulationParameters.MudMotor.delta_rotor, 2) * simulationParameters.MudMotor.N_stator * simulationParameters.MudMotor.N_rotor + lateralModel.MudTorque - axialTorsionalModel.TorqueOnBit);
                }
            }

            // Compute states from Riemann invariants
            state.PipeAngularVelocity = 1.0 / 2.0 * (axialTorsionalModel.DownwardTorsionalWave + axialTorsionalModel.UpwardTorsionalWave);
            state.PipeShearStrain = 1.0 / (2.0 * simulationParameters.Drillstring.TorsionalWaveSpeed) * (axialTorsionalModel.DownwardTorsionalWave - axialTorsionalModel.UpwardTorsionalWave);

            state.PipeAxialVelocity = 1.0 / 2.0 * (axialTorsionalModel.DownwardAxialWave + axialTorsionalModel.UpwardAxialWave);
            state.PipeAxialStrain = 1.0 / (2.0 * simulationParameters.Drillstring.AxialWaveSpeed) * (axialTorsionalModel.DownwardAxialWave - axialTorsionalModel.UpwardAxialWave);
            /*
            // Bending moments
            Mb_x = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeInertia).PointwiseMultiply(Xc_iPlus1 - 2 * state.XDisplacement + Xc_iMinus1) / (simulationParameters.LumpedCells.dxL * simulationParameters.LumpedCells.dxL); // Bending moment x-component
            Mb_y = simulationParameters.Drillstring.YoungModuli.PointwiseMultiply(simulationParameters.Drillstring.PipeInertia).PointwiseMultiply(Yc_iPlus1 - 2 * state.YDisplacement + Yc_iMinus1) / (simulationParameters.LumpedCells.dxL * simulationParameters.LumpedCells.dxL); // Bending moment y-component

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
                r_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) + Yc_ddot[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) -
                                state.XVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]) +
                                state.YVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.WhirlAngle[simulationParameters.Drillstring.IndexSensor]);

                phi_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * (1.0 / (Math.Pow(state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor], 2) + Constants.eps) * (Yc_ddot[simulationParameters.Drillstring.IndexSensor] * state.XDisplacement[simulationParameters.Drillstring.IndexSensor] -
                    Xc_ddot[simulationParameters.Drillstring.IndexSensor] * state.YDisplacement[simulationParameters.Drillstring.IndexSensor]) - 2.0 * state.RadialVelocity[simulationParameters.Drillstring.IndexSensor] * state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor] / (state.RadialDisplacement[simulationParameters.Drillstring.IndexSensor] + Constants.eps)) +
                    invertedSlipCondition[simulationParameters.Drillstring.IndexSensor] * phi_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                double phi0_sensor = 0;
                phi_ddot = 0.0;

                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor]); // radial position of accelerometer relative to pipe centerline
                    theta_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] + invertedSlipCondition[simulationParameters.Drillstring.IndexSensor] * theta_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                    u_x = state.XDisplacement[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - phi0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]);
                    u_y = state.YDisplacement[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) + r0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]);
                    u_x_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor])) +
                        phi0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]));
                    u_y_ddot = Yc_ddot[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor])) -
                        r0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor] * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor);
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.SleeveInnerRadius + simulationParameters.Drillstring.SleeveOuterRadius);

                    theta_ddot = state.slip_condition[simulationParameters.Drillstring.IndexSensor] * state.SleeveAngularAcceleration[idx_sleeve_sensor] + invertedSlipCondition[simulationParameters.Drillstring.IndexSensor] * theta_ddot_noslip[simulationParameters.Drillstring.IndexSensor];
                    u_x = state.XDisplacement[simulationParameters.Drillstring.IndexSensor] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y = state.YDisplacement[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot = Xc_ddot[simulationParameters.Drillstring.IndexSensor] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot = Yc_ddot[simulationParameters.Drillstring.IndexSensor] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }
                if (!simulationParameters.Drillstring.SleeveIndexPosition.Contains(simulationParameters.Drillstring.IndexSensor - 1))
                {
                    r0_sensor = 0.5 * (simulationParameters.Drillstring.InnerRadius[simulationParameters.Drillstring.IndexSensor - 1] + simulationParameters.Drillstring.OuterRadius[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_iMinus1 = state.XDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - phi0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_y_iMinus1 = state.YDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) + r0_sensor * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]);
                    u_x_ddot_iMinus1 = Xc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1])) + phi0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) -
                        state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]));
                    u_y_ddot_iMinus1 = Yc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1])) -
                       r0_sensor * (Math.Pow(state.WhirlVelocity[simulationParameters.Drillstring.IndexSensor - 1], 2) * Math.Sin(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]) - state.AngularAcceleration[simulationParameters.Drillstring.IndexSensor - 1] * Math.Cos(state.AngularDisplacement[simulationParameters.Drillstring.IndexSensor - 1]));
                }
                else
                {
                    int idx_sleeve_sensor = simulationParameters.Drillstring.SleeveIndexPosition.ToList().FindIndex(x => x == simulationParameters.Drillstring.IndexSensor - 1);
                    u_x_iMinus1 = state.XDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - phi0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_y_iMinus1 = state.YDisplacement[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) + r0_sensor * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]);
                    u_x_ddot_iMinus1 = Xc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + r0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) + phi0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) -
                        state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                    u_y_ddot_iMinus1 = Yc_ddot[simulationParameters.Drillstring.IndexSensor - 1] + phi0_sensor * (-Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor])) -
                        r0_sensor * (Math.Pow(state.SleeveAngularVelocity[idx_sleeve_sensor], 2) * Math.Sin(state.SleeveAngularDisplacement[idx_sleeve_sensor]) - state.SleeveAngularAcceleration[idx_sleeve_sensor] * Math.Cos(state.SleeveAngularDisplacement[idx_sleeve_sensor]));
                }

                // Bending angles
                Theta_x = -(u_y - u_y_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle x-component
                Theta_y = (u_x - u_x_iMinus1) / simulationParameters.LumpedCells.dxL;// Bending angle y-component
                Theta_x_ddot = -(u_y_ddot - u_y_ddot_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle second derivative x-component
                Theta_y_ddot = (u_x_ddot - u_x_ddot_iMinus1) / simulationParameters.LumpedCells.dxL; // Bending angle second derivative y-component
            }
            */
            output.BitVelocity = state.PipeAxialVelocity[state.PipeAxialVelocity.RowCount - 1, state.PipeAxialVelocity.ColumnCount - 1]; // Bit velocity

            // Parse outputs
            output.NormalForceProfileStiffString = lateralModel.NormalCollisionForce; // Pipe shear strain 
            output.NormalForceProfileSoftString = lateralModel.SoftStringNormalForce;
            output.TensionProfile = lateralModel.Tension; // Tension profile

            var tempMatrix = state.PipeShearStrain.PointwiseMultiply(lateralModel.ScalingMatrix * lateralModel.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            output.Torque = ToVector(tempMatrix.ToColumnMajorArray()); // Torque profile vs. depth
            //output.BendingMomentX = Mb_x;// Bending moment x-component profile
            //output.BendingMomentY = Mb_y;// Bending moment y-component profile
            //output.TangentialForceProfile = TangentialCoulombFrictionForce;// Bending moment y-component profile Tangential force profile
            output.WeightOnBit = axialTorsionalModel.WeightOnBit;  // Weight on bit
            output.TorqueOnBit = axialTorsionalModel.TorqueOnBit;  // Torque on bit
            output.Depth = simulationParameters.LumpedCells.xL;
            //output.SensorMb_x = Mb_x[simulationParameters.Drillstring.IndexSensor];
            //output.SensorMb_y = Mb_y[simulationParameters.Drillstring.IndexSensor];
            output.RadialDisplacement = state.RadialDisplacement;
            output.WhirlAngle = state.WhirlAngle;
            output.WhirlSpeed = state.WhirlVelocity;
            //output.BendingMoment = (Square(Mb_x) + Square(Mb_y)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

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
                //output.SensorAngularAcceleration = theta_ddot; // angular acceleration
                //output.SensorRadialAcceleration = r_ddot; // radial acceleration
                //output.SensorWhirlAcceleration = phi_ddot; // whirl acceleration
                //output.SensorBendingAngleX = Theta_x; // bending angle x-component
                //output.SensorBendingAngleY = Theta_y; // bending angle y-component
                //output.SecondDerivativeSensorBendingAngleX = Theta_x_ddot; // bending angle second derivative x-component
                //output.SecondDerivativeSensorBendingAngleY = Theta_y_ddot; // bending angle second derivative y-component
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
        }
    }
}
