using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using System.Security.Cryptography.X509Certificates;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public static class AccelerationCalculation
    {
       

        public static void PrepareAxialTorsional(AxialTorsionalModel model, State state, SimulationParameters parameters)
        {
            
            for (int i = 0; i < parameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {
                for (int j = 0; j < parameters.LumpedCells.NumberOfLumpedElements; j++)          
                {
                    // --- Update Torsional waves                   
                    //Downward torsional wave
                    model.DownwardTorsionalWave[i, j] = state.PipeAngularVelocity[i, j] 
                        + parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain[i, j];
                    //Upward torsional wave
                    model.UpwardTorsionalWave[i, j]   = state.PipeAngularVelocity[i, j] 
                        - parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain[i, j]; 
                    // --- Update Axial waves
                    // Downward axial wave
                    model.DownwardAxialWave[i, j] = state.PipeAxialVelocity[i, j] 
                        + parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain[i, j]; 
                    //Upward axial wave
                    model.UpwardAxialWave[i, j] = state.PipeAxialVelocity[i, j] 
                        - parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain[i, j];                        
                }
            }
        }
        public static void PreprareLateral(LateralModel model, State state, SimulationParameters parameters)
        {
            
        
            // Initialize scaling matrix and compute element-wise product    
            Vector<double> elementWiseProduct = parameters.Drillstring.YoungModuli.PointwiseMultiply(parameters.Drillstring.PipeArea);
            //Column matrix with ones
            model.ScalingMatrix = Vector<double>.Build.Dense(parameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            //Matrix with YoungModulus * Area repeated in each column
            /*
            |EA1 EA2 EA3 ... EAn|
            |EA1 EA2 EA3 ... EAn|
            |...                |
            |EA1 EA2 EA3 ... EAn|
            */
            Matrix<double> dragMatrix = model.ScalingMatrix * elementWiseProduct.ToRowMatrix();
            dragMatrix = state.PipeAxialStrain.PointwiseMultiply(dragMatrix);
        
            Vector<double> drag_flattened = ToVector(dragMatrix.ToColumnMajorArray());
            Vector<double> drag = LinearInterpolate(parameters.DistributedCells.x, drag_flattened, parameters.LumpedCells.ElementLength);
      
            
            
            Vector<double> phiVec_dote = ExtendVectorStart(0, parameters.Trajectory.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, parameters.Trajectory.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, parameters.Trajectory.thetaVec);
            Vector<double> trapezoidalsIntegration = CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, Reverse(parameters.Buoyancy.dsigma_dx));
            model.Tension = Reverse(trapezoidalsIntegration) + parameters.Buoyancy.axialBuoyancyForceChangeOfDiameters - drag;
            Vector<double> fN_softstring = (Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - parameters.Buoyancy.Wb.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((model.Tension + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();
            Vector<double> I_fN_softstring = Utilities.CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, fN_softstring);
            model.SoftStringNormalForce = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)

            Vector<double> AiExtended = ExtendVectorStart(parameters.Drillstring.InnerArea[0], parameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(parameters.Drillstring.OuterArea[0], parameters.Drillstring.OuterArea);
            
            /*Vector<double> F_comp = AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                    - AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) * (1 - 2 * parameters.Drillstring.PoissonRatio)
                                  - Tension;
            Tension = -F_comp;*/
            
            model.Tension += (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                (  
                    AoExtended.PointwiseMultiply(parameters.Buoyancy.annularPressure - parameters.Buoyancy.hydrostaticAnnularPressure) 
                    - AiExtended.PointwiseMultiply(parameters.Buoyancy.stringPressure - parameters.Buoyancy.hydrostaticStringPressure)
                );   

            Vector<double> bendingStiffness = ExtendVectorStart(parameters.Drillstring.BendingStiffness[0], parameters.Drillstring.BendingStiffness); 
            model.BendingStiffness = - (bendingStiffness - Math.Pow(Math.PI, 2) * model.Tension / (2 * parameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);                                            
            model.PolarMomentTimesShearModuli = parameters.Drillstring.PipePolarMoment.PointwiseMultiply(parameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> torqueMatrix = state.PipeShearStrain.PointwiseMultiply(model.ScalingMatrix * model.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(torqueMatrix.ToColumnMajorArray());
            Vector<double> torque = LinearInterpolate(parameters.DistributedCells.x, torqueFlattened, parameters.LumpedCells.ElementLength);

            //Allocate variables for the loop. For synchronous computing
            double normalForce;
            double binormalForce;
            double oldNormalForce = 0;
            double oldBinormalForce = 0;
            double differentialTorque;
            double InertiaTimesYoungModulus;
            double hVectorProductNormalDorProductTangent;
            double signToolFace;
            double dotProduct;            
            for (int i = 0; i < parameters.LumpedCells.NumberOfLumpedElements; i++)
            {               
                // Normal force components in Frenet-Serret coordinate system
                differentialTorque = (i==0) ?  0 : (torque[i+1] - torque[i]) / parameters.LumpedCells.DistanceBetweenElements;
                InertiaTimesYoungModulus = parameters.Drillstring.YoungModuli[i] * parameters.Drillstring.PipeInertia[i];
                binormalForce = parameters.Buoyancy.Wb[i] * parameters.Trajectory.bz[i] 
                    + parameters.Trajectory.Curvature[i] * differentialTorque 
                    + parameters.Trajectory.CurvatureDerivative[i] * torque[i+1] 
                    - 2 * InertiaTimesYoungModulus * parameters.Trajectory.CurvatureDerivative[i] * parameters.Trajectory.Torsion[i] 
                    - InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * parameters.Trajectory.TorsionDerivative[i];
                normalForce = parameters.Trajectory.Curvature[i] * (model.Tension[i+1] + parameters.Buoyancy.normalBuoyancyForceChangeOfDiameters[i+1] - parameters.Trajectory.Torsion[i] * torque[i+1]) 
                    + parameters.Buoyancy.Wb[i] * parameters.Trajectory.nz[i] 
                    - InertiaTimesYoungModulus * parameters.Trajectory.CurvatureSecondDerivative[i] 
                    + InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * (parameters.Trajectory.Torsion[i] * parameters.Trajectory.Torsion[i]);
                //At the first step, repeat the values
                if (i == 0)
                {
                    oldNormalForce = normalForce;
                    oldBinormalForce = binormalForce;
                }                                
                hVectorProductNormalDorProductTangent = 
                (parameters.Trajectory.hy[i] * parameters.Trajectory.nz[i] - (parameters.Trajectory.hz[i] * parameters.Trajectory.ny[i])) * parameters.Trajectory.tx[i] +
                (parameters.Trajectory.hz[i] * parameters.Trajectory.nx[i] - (parameters.Trajectory.hx[i] * parameters.Trajectory.nz[i])) * parameters.Trajectory.ty[i] +
                (parameters.Trajectory.hx[i] * parameters.Trajectory.ny[i] - (parameters.Trajectory.hy[i] * parameters.Trajectory.nx[i])) * parameters.Trajectory.tz[i];
                signToolFace = Math.Sign(hVectorProductNormalDorProductTangent);
                dotProduct = parameters.Trajectory.hx[i] * parameters.Trajectory.nx[i] +
                                parameters.Trajectory.hy[i] * parameters.Trajectory.ny[i] +
                                parameters.Trajectory.hz[i] * parameters.Trajectory.nz[i];
                dotProduct = Math.Max(-1, dotProduct);
                dotProduct = Math.Min(1, dotProduct);
                // ========== Update relevant model variables ==========
                model.ToolFaceAngle[i] = Math.Acos(dotProduct) * signToolFace;
                //This is equivalent of integrating with the trapezoidal rule and then getting the difference.                    
                model.PreStressNormalForce[i] = 0.5 * (oldNormalForce + normalForce) * (parameters.LumpedCells.ElementLength[i+1] - parameters.LumpedCells.ElementLength[i]);
                model.PreStressBinormalForce[i] = 0.5 * (oldBinormalForce + binormalForce) * (parameters.LumpedCells.ElementLength[i+1] - parameters.LumpedCells.ElementLength[i]);
                //Update normal and binormal values from the 
                oldNormalForce = normalForce;
                oldBinormalForce = binormalForce;     
            }            
            
            
        }
        public static void AxialTorsionalSystem(AxialTorsionalModel torsionalModel, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {                                        
            torsionalModel.UpdateBoundaryConditions(state, parameters, simulationInput);               
            state.BitVelocity = 0.5 * 
                (
                    torsionalModel.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.DownwardAxialWave.ColumnCount - 1] 
                    + torsionalModel.UpwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.UpwardAxialWave.ColumnCount - 1]
                );
           
            double angularVelocityBottom;
            if (!configuration.UseMudMotor)
                angularVelocityBottom = 0.5 * 
                    (
                        torsionalModel.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.DownwardTorsionalWave.ColumnCount - 1] 
                        + torsionalModel.UpwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, torsionalModel.UpwardTorsionalWave.ColumnCount - 1]
                    );
            else
                angularVelocityBottom = state.MudRotorAngularVelocity;

            double[] bitForces = parameters.BitRock.
                CalculateInteractionForce(torsionalModel, state, angularVelocityBottom, parameters);
            torsionalModel.TorqueOnBit = bitForces[0];
            torsionalModel.WeightOnBit = bitForces[1];
           
            // manage the bit sticking off bottom condition
            if (!state.onBottom)
            {
                double omega_ = state.AngularVelocity[state.AngularVelocity.Count - 1];
                if (simulationInput.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;             
                    double torsionalAcceleration = torsionalModel.UpwardTorsionalWave[torsionalModel.UpwardTorsionalWave.RowCount - 1, torsionalModel.UpwardTorsionalWave.ColumnCount - 1];
                    double axialAcceleration = torsionalModel.UpwardAxialWave[torsionalModel.UpwardAxialWave.RowCount - 1, torsionalModel.UpwardAxialWave.ColumnCount - 1];                                            
                    torsionalModel.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] / parameters.Drillstring.TorsionalWaveSpeed * torsionalAcceleration;
                    torsionalModel.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] / parameters.Drillstring.AxialWaveSpeed * axialAcceleration;
                }
                else
                {
                    double normalForce_ = simulationInput.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.AxialVelocity[state.AxialVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            torsionalModel.TorqueOnBit = 0;
                            torsionalModel.WeightOnBit = 0;
                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            torsionalModel.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            torsionalModel.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        torsionalModel.TorqueOnBit = 0;
                        torsionalModel.WeightOnBit = 0;
                    }
                }
            }   
       }        
        
        public static void LateralSystem(LateralModel lateralModel, AxialTorsionalModel axialTorsionalModel, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {
            #region  Allocation
            // State variables -> can be simplified but decided to leave for the sake of debugging and readability. Can be optimized later if needed.
            double radialDisplacement;
            double whirlAngle;
            double cosWhirlAngle;
            double sinWhirlAngle;
            double radialVelocity;
            double whirlVelocity;
            double angularAcceleration;
            double XAcceleration;
            double YAcceleration;
            double ZAcceleration;
            double rotationSpeed;
            double rotationSpeedSquared;
            double rotationAngle;
            double rotationAcceleration;
            double axialVelocity;   
            // Axial and torsional data for coupling.           
            double torqueOnBit;
            double deltaTorsionalWave;
            double deltaAxialWave;
            double torqueElement;
            double forceElement;
            double torqueNextElement;
            double forceNextElement;
            double torqueDifference;
            double axialForceDifference;
            // Normal force variables
            double heavesideStep;
            double normalCollisionForce;
            // Friction variables
            double[] totalForce = new double[3];
            double[] normalVector = new double[3];
            double[] velocityVector = new double[3];
            double[] velocityTangential = new double[3];
            double[] tangentialVector = new double[3];
            double velocityTangentialMagnitude;
            double normalForceMagnitude;                
            double[] tangentialStaticForce = new double[3];
            double[] stopForce = new double[3];
            double stopForceMagnitude;
            double coulombStaticForceMagnitude;
            double coulombForceTemp;
            double axialCoulombFrictionForce;
            double[] coulombFrictionForce;
            double coulombFrictionX;
            double coulombFrictionY;
            double coulombFrictionZ;
            double velocityMagnitude;                                    
            double frictionTorque;
            double outerRadius;
            double inertia;
            double axialStaticForce;
            double sumForcesX;
            double sumForcesY;
            double sleeveBrakeForce;            
            //Elastic force-related variables
            double XiMinus1;
            double YiMinus1;
            double XiPlus1;
            double YiPlus1;
            double kMinus1;
            double kPlus1;
            double ElasticForceX;
            double ElasticForceY;
            double PreStressForceX;
            double PreStressForceY;
            // Fluid force-related variables
            double fluidDampingCoefficient;
            double fluidForceX;
            double fluidForceY;
            // Unbalance force-related variables
            double unbalanceForceX;
            double unbalanceForceY;            
            #endregion            

            if (!configuration.UseMudMotor)
            {
                lateralModel.MudTorque = 0;
            }
            else
            {
                double speedRatio = (state.MudRotorAngularVelocity - state.MudStatorAngularVelocity) / parameters.MudMotor.omega0_motor;              
                if (speedRatio <= 1)
                    lateralModel.MudTorque = parameters.MudMotor.T_max_motor * Math.Pow(1 - speedRatio, 1.0 / parameters.MudMotor.alpha_motor);
                else
                    lateralModel.MudTorque = -parameters.MudMotor.P0_motor * parameters.MudMotor.V_motor * (speedRatio - 1);                
            }
            
            
            // =============================== Main loop for calculating forces and accelerations in the lateral model ===============================
            for (int i = 0; i < state.XDisplacement.Count; i++)
            {
                #region Polar Coordinates Conversion
                // Get the radial displacement 
                radialDisplacement = Math.Sqrt(state.XDisplacement[i] * state.XDisplacement[i] + state.YDisplacement[i] * state.YDisplacement[i]);   
                // Calculate the whirl angle 
                whirlAngle = Math.Atan2(state.YDisplacement[i], state.XDisplacement[i]);
                // Pre-calculate whirl angle sin and cos to avoid multiple calculations
                cosWhirlAngle = whirlAngle == 0.0 ? 1.0 : state.XDisplacement[i]/radialDisplacement; // Math.Cos(whirlAngle);
                sinWhirlAngle = whirlAngle == 0.0 ? 0.0 : state.YDisplacement[i]/radialDisplacement; // Math.Sin(whirlAngle);
                // Calculate the radial velocity
                radialVelocity = state.XVelocity[i] * cosWhirlAngle + state.YVelocity[i] * sinWhirlAngle;
                // Calculate whirl velocity
                whirlVelocity = radialDisplacement == 0.0 ? 1E6 : (state.YVelocity[i] * state.XDisplacement[i] - state.XVelocity[i] * state.YDisplacement[i])/(radialDisplacement * radialDisplacement);
                #endregion
                #region Axial-Torsional Distributed Forces Forces
                torqueOnBit = configuration.UseMudMotor ? lateralModel.MudTorque : axialTorsionalModel.TorqueOnBit;
                //Calculated the torque and force difference in each element (TorqueElement and TorqueNextElement)                
                deltaTorsionalWave = axialTorsionalModel.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] - axialTorsionalModel.UpwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i];                
                deltaAxialWave = axialTorsionalModel.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] - axialTorsionalModel.UpwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i];                
                
                torqueElement = parameters.Drillstring.PipePolarMoment[i] * parameters.Drillstring.ShearModuli[i] / (2.0 * parameters.Drillstring.TorsionalWaveSpeed) * deltaTorsionalWave;                
                forceElement = parameters.Drillstring.PipeArea[i] * parameters.Drillstring.YoungModuli[i] / (2.0 * parameters.Drillstring.AxialWaveSpeed) * deltaAxialWave;

                //If it is the last element, use the torque on bit
                torqueNextElement = (i == state.XDisplacement.Count - 1) ? 
                    torqueOnBit 
                    : 
                    parameters.Drillstring.PipePolarMoment[i + 1] * parameters.Drillstring.ShearModuli[i + 1] / (2.0 * parameters.Drillstring.TorsionalWaveSpeed) * 
                        (
                            axialTorsionalModel.DownwardTorsionalWave[0, i + 1] 
                          - axialTorsionalModel.UpwardTorsionalWave[0, i + 1]
                        );
                
                forceNextElement = (i == state.XDisplacement.Count - 1) ? 
                    axialTorsionalModel.WeightOnBit 
                    : 
                    parameters.Drillstring.PipeArea[i + 1] * parameters.Drillstring.YoungModuli[i + 1] / (2.0 * parameters.Drillstring.AxialWaveSpeed) * 
                        (
                            axialTorsionalModel.DownwardAxialWave[0, i + 1] 
                          - axialTorsionalModel.UpwardAxialWave[0, i + 1]
                        );            

                torqueDifference = torqueElement - torqueNextElement;
                axialForceDifference = forceElement - forceNextElement;                      
                #endregion
                #region Collision calculation
                // Check if there is collision or not and store the Heaveside Step Function
                heavesideStep = radialDisplacement >= parameters.Wellbore.DrillStringClearance[i] ? 1.0 : 0.0;                                    
                // Calculate normal force
                normalCollisionForce = heavesideStep * 
                    (
                        parameters.Wellbore.WallStiffness * (radialDisplacement - parameters.Wellbore.DrillStringClearance[i]) 
                       + parameters.Wellbore.WallDamping * radialVelocity
                    );  
                                        
                if (lateralModel.NormalCollisionForce.Count > 1 && i == lateralModel.NormalCollisionForce.Count - 2)
                    normalCollisionForce += simulationInput.ForceToInduceBitWhirl;    
                #endregion
                #region Elastic Force Calculation
                // If it is the first element, get the pinned boundary condition
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Same with the stiffness
                kMinus1 = lateralModel.BendingStiffness[i];
                kPlus1 = lateralModel.BendingStiffness[i + 1];
                ElasticForceX = kMinus1 * XiMinus1  - (kMinus1 + kPlus1) * state.XDisplacement[i] + kPlus1 * XiPlus1;
                ElasticForceY = kMinus1 * YiMinus1  - (kMinus1 + kPlus1) * state.YDisplacement[i] + kPlus1 * YiPlus1;
                #endregion
                #region Pre-Stress Force Calculation
                PreStressForceX = lateralModel.PreStressNormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) + 
                                lateralModel.PreStressBinormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) ;
                PreStressForceY = lateralModel.PreStressNormalForce[i] * Math.Cos(lateralModel.ToolFaceAngle[i]) - 
                                lateralModel.PreStressBinormalForce[i] * Math.Sin(lateralModel.ToolFaceAngle[i]) ;                                    
                #endregion
                #region Fluid Force Calculation
                //Extracts angular speed depending on if it is a sleeve or not
                bool hasSleeve = state.SleeveToLumpedIndex[i] != -1;
                //If it is not used properly, it should crash the code.
                int sleeveIndex = hasSleeve ?  state.SleeveToLumpedIndex[i] : -1;
                //Select between sleeve and non-sleeve nodes
                rotationSpeed = hasSleeve ? state.SleeveAngularVelocity[sleeveIndex] : state.AngularVelocity[i];                            
                rotationSpeedSquared = rotationSpeed * rotationSpeed;
                fluidDampingCoefficient = parameters.Wellbore.FluidDampingCoefficient / parameters.Drillstring.FluidAddedMass[i];                    
                fluidForceX = - parameters.Drillstring.FluidAddedMass[i] * 
                                    (
                                        fluidDampingCoefficient * state.XVelocity[i]
                                        - 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                        + rotationSpeed * state.YVelocity[i]
                                        + 0.5 * fluidDampingCoefficient * rotationSpeed * state.YDisplacement[i]
                                    );
                fluidForceY = - parameters.Drillstring.FluidAddedMass[i] * 
                                    (
                                        fluidDampingCoefficient * state.YVelocity[i]
                                        - 0.25 * rotationSpeedSquared * state.YDisplacement[i]
                                        - rotationSpeed * state.XVelocity[i]
                                        - 0.5 * fluidDampingCoefficient * rotationSpeed * state.XDisplacement[i]
                                    );
                #endregion
                // Sleeve braking force
                sleeveBrakeForce = hasSleeve ? parameters.Drillstring.SleeveTorsionalDamping[sleeveIndex] * (rotationSpeed - state.SleeveAngularVelocity[sleeveIndex]) : 0;
                #region Unbalance Forces
                // lateral forces due to mass imbalance
                // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center, 
                // which causes a lateral force and a torque as the pipe is displaced                
                rotationAngle = hasSleeve ? state.SleeveAngularDisplacement[sleeveIndex] : state.AngularDisplacement[i];                                        
                rotationAcceleration = hasSleeve ? state.SleeveAngularAcceleration[sleeveIndex] : state.AngularAcceleration[i];                                        
                unbalanceForceX = parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i]*
                    (
                        rotationSpeedSquared * Math.Cos(rotationAngle)
                        + state.AngularAcceleration[i] * Math.Sin(rotationAngle)
                    );
                unbalanceForceY = parameters.Drillstring.EccentricMass[i] * parameters.Drillstring.Eccentricity[i]* 
                    (
                        rotationSpeedSquared * Math.Sin(rotationAngle)
                        - rotationAcceleration * Math.Cos(rotationAngle)
                    );                                                            
                #endregion                                        
                #region Coulomb Friction
                // Axial velocity
                axialVelocity = state.AxialVelocity[i];

                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.OuterRadius[i];
                //double innerRadius = hasSleeve ? parameters.Drillstring.SleeveInnerRadius : parameters.Drillstring.OuterRadius[i];  
                inertia = hasSleeve ? parameters.Drillstring.SleeveMassMomentOfInertia : parameters.Drillstring.LumpedElementMomentOfInertia[i];
                axialStaticForce = state.AxialVelocity[i] * parameters.Drillstring.LumpedElementMass[i]/parameters.InnerLoopTimeStep 
                        + axialForceDifference - parameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i];                    
                // Skip the calculation if there is no collision 
                if (heavesideStep == 1.0)
                {
                    // The total normal force is the sum of elastic, pre-stress, fluid, and unbalance forces                        
                    sumForcesX = ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX;
                    sumForcesY = ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY;
                    totalForce = new double[3] { sumForcesX, sumForcesY, axialStaticForce };                                     
                    // Estaimate principal direction in the used coordinate system 
                    normalVector = new double[3] {cosWhirlAngle, sinWhirlAngle, 0}; // In the direction of the radial displacement, which is the normal direction in the case of a cylindrical wellbore
                
                    velocityVector = new double[3] 
                    { state.XVelocity[i] + rotationSpeed * outerRadius * normalVector[1], 
                        state.YVelocity[i] - rotationSpeed * outerRadius * normalVector[0], 
                        axialVelocity
                    };
                    velocityMagnitude = Math.Sqrt(
                        velocityVector[0] * velocityVector[0] +
                        velocityVector[1] * velocityVector[1] +
                        velocityVector[2] * velocityVector[2]
                    );                
                    velocityTangential = new double[3]
                    {
                        velocityVector[0] - (velocityMagnitude * normalVector[0]),
                        velocityVector[1] - (velocityMagnitude * normalVector[1]),
                        velocityVector[2] - (velocityMagnitude * normalVector[2])
                    };
                    velocityTangentialMagnitude = Math.Sqrt(
                        velocityTangential[0] * velocityTangential[0] +
                        velocityTangential[1] * velocityTangential[1] +
                        velocityTangential[2] * velocityTangential[2]
                    );
                    tangentialVector = velocityTangentialMagnitude == 0.0 ? new double[3] { 0.0, 0.0, 0.0 } : new double[3]
                    {
                        velocityTangential[0] / velocityTangentialMagnitude,
                        velocityTangential[1] / velocityTangentialMagnitude,
                        velocityTangential[2] / velocityTangentialMagnitude
                    };

                    normalForceMagnitude = (totalForce[0] * normalVector[0] + totalForce[1] * normalVector[1] + totalForce[2] * normalVector[2]);                

                    tangentialStaticForce = new double[3]
                    {
                        parameters.Friction.mu_s[i] * normalForceMagnitude * tangentialVector[0],
                        parameters.Friction.mu_s[i] * normalForceMagnitude * tangentialVector[1],
                        parameters.Friction.mu_s[i] * normalForceMagnitude * tangentialVector[2]
                    };               

                    stopForce = new double[3]
                    {
                        tangentialStaticForce[0],
                        tangentialStaticForce[1],
                        axialStaticForce + tangentialStaticForce[2] 
                    };
                    stopForceMagnitude = Math.Sqrt(
                        stopForce[0] * stopForce[0] +
                        stopForce[1] * stopForce[1] +
                        stopForce[2] * stopForce[2]
                    ) + Constants.eps;

                    coulombStaticForceMagnitude = parameters.Friction.mu_k[i] * normalForceMagnitude;

                    if (state.SlipCondition[i] == 0)
                        // If the previous state was a not a slip condition, re-evaluate by comparing forces.  
                        state.SlipCondition[i] = stopForceMagnitude > coulombStaticForceMagnitude ? 1:0;
                    else
                        // If the previous state was a slip condition  
                        state.SlipCondition[i] = velocityMagnitude < parameters.Friction.v_c ? 0 : state.SlipCondition[i];
                    
                    if (state.SlipCondition[i] == 0)
                    {
                        //Project in the direction of the actual force
                        coulombForceTemp = heavesideStep * Math.Min(stopForceMagnitude, Math.Abs(coulombStaticForceMagnitude));
                        coulombFrictionForce = new double[3];
                        for (int j = 0; j < 3; j++)
                        {
                            coulombFrictionForce[j] = - coulombForceTemp * stopForce[j]/stopForceMagnitude;
                        }
                    }
                    else
                    {
                        //Project on the tangential velocity direction  
                        coulombForceTemp =  normalCollisionForce * (parameters.Friction.mu_k[i] + (parameters.Friction.mu_s[i] - parameters.Friction.mu_k[i]) * Math.Exp( - parameters.Friction.stribeck *  velocityMagnitude));
                        coulombFrictionForce = new double[3];
                        for (int j = 0; j < 3; j++)
                        {
                            coulombFrictionForce[j] = - coulombForceTemp * tangentialVector[j];
                        }                
                    }
                    //Update relevant forces and torque with the calculated coulomb friction force
                    coulombFrictionX = coulombFrictionForce[0];
                    coulombFrictionY = coulombFrictionForce[1];
                    coulombFrictionZ = coulombFrictionForce[2];
                    frictionTorque = hasSleeve ? sleeveBrakeForce * outerRadius : 
                    (
                        outerRadius * (coulombFrictionForce[1] * normalVector[0] - coulombFrictionForce[0] * normalVector[1]) 
                    );
                    if (hasSleeve)
                    {
                        axialCoulombFrictionForce = coulombFrictionForce[2] *  (1 - parameters.Drillstring.AxialFrictionReduction);
                        coulombFrictionZ = Math.Sqrt(coulombForceTemp * coulombForceTemp - axialCoulombFrictionForce * axialCoulombFrictionForce) * Math.Sign(coulombFrictionForce[2]);
                        if (i == parameters.Drillstring.IndexSensor)
                        {
                            lateralModel.PhiDdotNoSlipSensor = stopForceMagnitude;
                            // Needs to be checked!
                            lateralModel.ThetaDotNoSlipSensor = sumForcesY * normalVector[0] - sumForcesX * normalVector[1];
                        }
                        // It needs to be zeroed before, as there might have changes in the sleeve position when the number of lumped parameters change 
                        state.SleeveForces[i] = 0;                
                        state.SleeveForces[i] = hasSleeve ? Math.Sqrt(
                            coulombFrictionForce[0]*coulombFrictionForce[0] + 
                            coulombFrictionForce[1]*coulombFrictionForce[1]
                        ): 0;
                    }                
                }
                else
                {
                    coulombFrictionX = 0.0;
                    coulombFrictionY = 0.0;
                    coulombFrictionZ = 0.0;
                    frictionTorque = 0.0;
                    state.SleeveForces[i] = 0;
                    lateralModel.PhiDdotNoSlipSensor = 0.0;
                    lateralModel.ThetaDotNoSlipSensor = 0.0;
                }                            
                #endregion                   
                #region Accelerations
                              
                if (hasSleeve)
                {                    
                    // Why is there a TimeStep in here?                    
                    state.SleeveAngularAcceleration[sleeveIndex] = parameters.InnerLoopTimeStep * (sleeveBrakeForce * parameters.Drillstring.SleeveInnerRadius - parameters.Drillstring.SleeveOuterRadius * state.SleeveForces[i]) / parameters.Drillstring.SleeveMassMomentOfInertia;;
                }
                //Used for debugging purposes
                
                angularAcceleration = 
                        (
                            torqueDifference
                             - parameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed
                             - frictionTorque
                        )/inertia;      
                // Variables are generated locally to facilitate debugging only.
                XAcceleration = (ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - parameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - lateralModel.NormalCollisionForce[i] * cosWhirlAngle
                    + coulombFrictionX) / (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]);
                YAcceleration= (ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - parameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - lateralModel.NormalCollisionForce[i] * sinWhirlAngle
                    + coulombFrictionY) / (parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i]);
                ZAcceleration = ( axialForceDifference - parameters.Drillstring.CalculatedAxialDamping * axialVelocity + coulombFrictionZ)/parameters.Drillstring.LumpedElementMass[i];                

                //Update state
                lateralModel.NormalCollisionForce[i] = normalCollisionForce;
                state.RadialDisplacement[i] = radialDisplacement;
                state.RadialVelocity[i] = radialVelocity;
                state.WhirlAngle[i] = whirlAngle;
                state.WhirlVelocity[i] = whirlVelocity;
                state.AxialAcceleration[i] = ZAcceleration;
                state.XAcceleration[i] = XAcceleration;
                state.YAcceleration[i] = YAcceleration;   
                state.AngularAcceleration[i] = angularAcceleration;       
                
                #endregion
                
                #region  Debbugging Outputs
                //===============================================================================================
                //                                 UNCOMMENT FOR DEBUGGING PURPOSES
                //===============================================================================================
                //
                //    if (Math.Abs(rotationSpeed) > 20)
                //    {
                //        int debug = 0;
                //    }
                //    if (Math.Abs(angularAcceleration) > 10000)
                //    {
                //        int debug = 0;
                //    }
                //    if (double.IsNaN(XAcceleration) || double.IsNaN(YAcceleration) || double.IsNaN(ZAcceleration) || double.IsNaN(state.AngularAcceleration[i]))
                //    {
                //        Console.WriteLine("NaN detected in lateral calculations at element " + i.ToString() +
                //            " XAcc: " + XAcceleration.ToString() +
                //            " YAcc: " + YAcceleration.ToString() +
                //            " ZAcc: " + ZAcceleration.ToString() +
                //            " AngularAcc: " + state.AngularAcceleration[i].ToString() +
                //            " NormalForce: " + normalCollisionForce.ToString() +
                //            " RadialDisp: " + radialDisplacement.ToString() +
                //            " RadialVel: " + radialVelocity.ToString() +
                //            " WhirlVel: " + whirlVelocity.ToString() +
                //            " TangentialVel: " + tangentialVelocity.ToString() +
                //            " AxialVel: " + axialVelocity.ToString() +
                //            " SlipCondition: " + state.SlipCondition[i].ToString()
                //            );
                //    }                
                #endregion

            }             
        }
    }
}