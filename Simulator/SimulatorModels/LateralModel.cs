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

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class LateralModel : IModel<LateralModel>
    {       
        private Vector<double> BendingStiffness;
        private Vector<double> PolarMomentTimesShearModuli;
        private Vector<double> PreStressNormalForce;
        private Vector<double> PreStressBinormalForce;                  
        private Matrix<double> ScalingMatrix;
        private Vector<double> ToolFaceAngle;
        // State variables -> can be simplified but decided to leave for the sake of debugging and readability. Can be optimized later if needed.
        private double radialDisplacement;
        private double whirlAngle;
        private double cosWhirlAngle;
        private double sinWhirlAngle;
        private double radialVelocity;
        private double whirlVelocity;
        private double angularAcceleration;
        private double XAcceleration;
        private double YAcceleration;
        private double ZAcceleration;
        private double rotationSpeed;
        private double rotationSpeedSquared;
        private double rotationAngle;
        private double rotationAcceleration;
        private double axialVelocity;   
        // Axial and torsional data for coupling.           
        private double torqueOnBit;
        private double deltaTorsionalWave;
        private double deltaAxialWave;
        private double torqueElement;
        private double forceElement;
        private double torqueNextElement;
        private double forceNextElement;
        private double torqueDifference;
        private double axialForceDifference;
        // Normal force variables
        private double heavesideStep;
        private double normalCollisionForce;
        // Friction variables
        private double axialCoulombFrictionForce;
        private double frictionTorque;
        private double outerRadius;
        private double inertia;
        private double sumForcesX;
        private double sumForcesY;
        private double sumForcesZ;            
        private double sleeveBrakeForce;            
        //Elastic force-related variables
        private double XiMinus1;
        private double YiMinus1;
        private double XiPlus1;
        private double YiPlus1;
        private double kMinus1;
        private double kPlus1;
        private double ElasticForceX;
        private double ElasticForceY;
        private double PreStressForceX;
        private double PreStressForceY;        
        // Fluid force-related variables
        private double fluidDampingCoefficient;
        private double fluidForceX;
        private double fluidForceY;
        // Unbalance force-related variables
        private double unbalanceForceX;
        private double unbalanceForceY;   
        
        public LateralModel(SimulationParameters simulationParameters, State state)
        {
            state.Torque = Vector<double>.Build.Dense(state.XDisplacement.Count + 1);
            BendingStiffness = Vector<double>.Build.Dense(state.XDisplacement.Count+1);
            PolarMomentTimesShearModuli = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressNormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressBinormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            ScalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            ToolFaceAngle = Vector<double>.Build.Dense(state.XDisplacement.Count);
        }
        public void UpdateBendingMoments(State state, SimulationParameters simulationParameters)
        {
            double XiMinus1, YiMinus1, XiPlus1, YiPlus1;
            double invElementLengthSquared = 1.0 / (simulationParameters.LumpedCells.ElementLength * simulationParameters.LumpedCells.ElementLength);
            double momentX, momentY;
            
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.YDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                //Calcualte the bending moments using the central difference scheme
                momentX = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (XiPlus1 - 2 * state.XDisplacement[i] + XiMinus1) * invElementLengthSquared; // Bending moment x-component
                momentY = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (YiPlus1 - 2 * state.YDisplacement[i] + YiMinus1) * invElementLengthSquared; //
                state.BendingMomentX[i] =  momentX;
                state.BendingMomentY[i] =  momentY;
            }
        }
        public void PrepareModel(LateralModel model, State state, SimulationParameters parameters)
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

            Vector<double> phiVec_dote = ExtendVectorStart(0, parameters.Trajectory.DiffPhiInterpolated);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, parameters.Trajectory.DiffThetaInterpolated);
            Vector<double> thetaVece = ExtendVectorStart(0, parameters.Trajectory.InterpolatedTheta);
            Vector<double> trapezoidalsIntegration = CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, Reverse(parameters.Buoyancy.dSigmaDx));
            state.Tension = Reverse(trapezoidalsIntegration) + parameters.Buoyancy.AxialBuoyancyForceChangeOfDiameters - drag;
            Vector<double> fN_softstring = (Square((state.Tension + parameters.Buoyancy.NormalBuoyancyForceChangeOfDiameters).PointwiseMultiply(thetaVec_dote) - parameters.Buoyancy.BuoyantWeightPerLength.PointwiseMultiply(thetaVece.PointwiseSin())) +
                                            Square((state.Tension + parameters.Buoyancy.NormalBuoyancyForceChangeOfDiameters).PointwiseMultiply(phiVec_dote).PointwiseMultiply(thetaVece.PointwiseSin()))).PointwiseSqrt();
            Vector<double> I_fN_softstring = Utilities.CummulativeTrapezoidal(parameters.LumpedCells.ElementLength, fN_softstring);
            state.SoftStringNormalForce = Diff(I_fN_softstring); // [N] Lumped normal force per element assuming soft - string model(not used in 4nDOF model)
            Vector<double> AiExtended = ExtendVectorStart(parameters.Drillstring.InnerArea[0], parameters.Drillstring.InnerArea);
            Vector<double> AoExtended = ExtendVectorStart(parameters.Drillstring.OuterArea[0], parameters.Drillstring.OuterArea);
            
            state.Tension += (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                (  
                    AoExtended.PointwiseMultiply(parameters.Buoyancy.AnnulusPressure - parameters.Buoyancy.HydrostaticAnnularPressure) 
                    - AiExtended.PointwiseMultiply(parameters.Buoyancy.StringPressure - parameters.Buoyancy.HydrostaticStringPressure)
                );   

            Vector<double> bendingStiffness = ExtendVectorStart(parameters.Drillstring.BendingStiffness[0], parameters.Drillstring.BendingStiffness); 
            model.BendingStiffness = - (bendingStiffness - Math.Pow(Math.PI, 2) * state.Tension / (2 * parameters.Drillstring.PipeLengthForBending)).PointwiseMaximum(0.0);                                            
            model.PolarMomentTimesShearModuli = parameters.Drillstring.PipePolarMoment.PointwiseMultiply(parameters.Drillstring.ShearModuli); // Element-wise multiplication
            Matrix<double> torqueMatrix = state.PipeShearStrain.PointwiseMultiply(model.ScalingMatrix * model.PolarMomentTimesShearModuli.ToRowMatrix()); // 5x136 matrix
            Vector<double> torqueFlattened = ToVector(torqueMatrix.ToColumnMajorArray());
            state.Torque = LinearInterpolate(parameters.DistributedCells.x, torqueFlattened, parameters.LumpedCells.ElementLength);

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
                differentialTorque = (i==0) ?  0 : (state.Torque[i+1] - state.Torque[i]) / parameters.LumpedCells.DistanceBetweenElements;
                InertiaTimesYoungModulus = parameters.Drillstring.YoungModuli[i] * parameters.Drillstring.PipeInertia[i];
                binormalForce = parameters.Buoyancy.BuoyantWeightPerLength[i] * parameters.Trajectory.bz[i] 
                    + parameters.Trajectory.Curvature[i] * differentialTorque 
                    + parameters.Trajectory.CurvatureDerivative[i] * state.Torque[i+1] 
                    - 2 * InertiaTimesYoungModulus * parameters.Trajectory.CurvatureDerivative[i] * parameters.Trajectory.Torsion[i] 
                    - InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * parameters.Trajectory.TorsionDerivative[i];
                normalForce = parameters.Trajectory.Curvature[i] * (state.Tension[i+1] + parameters.Buoyancy.NormalBuoyancyForceChangeOfDiameters[i+1] - parameters.Trajectory.Torsion[i] * state.Torque[i+1]) 
                    + parameters.Buoyancy.BuoyantWeightPerLength[i] * parameters.Trajectory.nz[i] 
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
        public void CalculateAccelerations(State state, in SimulationParameters parameters)
        {
            if (!parameters.UseMudMotor)
            {
                state.MudTorque = 0;
            }
            else
            {
                double speedRatio = (state.MudRotorAngularVelocity - state.MudStatorAngularVelocity) / parameters.MudMotor.omega0_motor;              
                if (speedRatio <= 1)
                    state.MudTorque = parameters.MudMotor.T_max_motor * Math.Pow(1 - speedRatio, 1.0 / parameters.MudMotor.alpha_motor);
                else
                    state.MudTorque = -parameters.MudMotor.P0_motor * parameters.MudMotor.V_motor * (speedRatio - 1);                
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
                torqueOnBit = parameters.UseMudMotor ? state.MudTorque : state.TorqueOnBit;
                //Calculated the torque and force difference in each element (TorqueElement and TorqueNextElement)                
                deltaTorsionalWave = state.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] - state.UpwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i];                
                deltaAxialWave = state.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i] - state.UpwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, i];                
                
                torqueElement = parameters.Drillstring.PipePolarMoment[i] * parameters.Drillstring.ShearModuli[i] / (2.0 * parameters.Drillstring.TorsionalWaveSpeed) * deltaTorsionalWave;                
                forceElement = parameters.Drillstring.PipeArea[i] * parameters.Drillstring.YoungModuli[i] / (2.0 * parameters.Drillstring.AxialWaveSpeed) * deltaAxialWave;

                //If it is the last element, use the torque on bit
                torqueNextElement = (i == state.XDisplacement.Count - 1) ? 
                    torqueOnBit 
                    : 
                    parameters.Drillstring.PipePolarMoment[i + 1] * parameters.Drillstring.ShearModuli[i + 1] / (2.0 * parameters.Drillstring.TorsionalWaveSpeed) * 
                        (
                            state.DownwardTorsionalWave[0, i + 1] 
                          - state.UpwardTorsionalWave[0, i + 1]
                        );                
                forceNextElement = (i == state.XDisplacement.Count - 1) ? 
                    state.WeightOnBit 
                    : 
                    parameters.Drillstring.PipeArea[i + 1] * parameters.Drillstring.YoungModuli[i + 1] / (2.0 * parameters.Drillstring.AxialWaveSpeed) * 
                        (
                            state.DownwardAxialWave[0, i + 1] 
                          - state.UpwardAxialWave[0, i + 1]
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
                                        
                if (state.NormalCollisionForce.Count > 1 && i == state.NormalCollisionForce.Count - 2)
                    normalCollisionForce += parameters.Input.ForceToInduceBitWhirl;    
                #endregion
                #region Elastic Force Calculation
                // If it is the first element, get the pinned boundary condition
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Same with the stiffness
                kMinus1 = this.BendingStiffness[i];
                kPlus1 = this.BendingStiffness[i + 1];
                ElasticForceX = kMinus1 * XiMinus1  - (kMinus1 + kPlus1) * state.XDisplacement[i] + kPlus1 * XiPlus1;
                ElasticForceY = kMinus1 * YiMinus1  - (kMinus1 + kPlus1) * state.YDisplacement[i] + kPlus1 * YiPlus1;
                #endregion
                #region Pre-Stress Force Calculation
                PreStressForceX = this.PreStressNormalForce[i] * Math.Sin(this.ToolFaceAngle[i]) + 
                                this.PreStressBinormalForce[i] * Math.Cos(this.ToolFaceAngle[i]) ;
                PreStressForceY = this.PreStressNormalForce[i] * Math.Cos(this.ToolFaceAngle[i]) - 
                                this.PreStressBinormalForce[i] * Math.Sin(this.ToolFaceAngle[i]) ;                                    
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
                  // The total normal force is the sum of elastic, pre-stress, fluid, unbalance forces, damping and colission forces                        
                sumForcesX = ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - parameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - normalCollisionForce * cosWhirlAngle;
                sumForcesY = ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - parameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - normalCollisionForce * sinWhirlAngle;
                sumForcesZ = axialForceDifference - parameters.Drillstring.CalculatedAxialDamping * state.AxialVelocity[i]; 
                    
                // Skip the calculation if there is no collision 
                if (heavesideStep == 1.0)
                {
                    // Several calculation can be seen in AuxiliarDevFiles/Symbolic_no_slip_condition
                                        
                    // With this 4nDoF model, no slip condition is only possible if vz = 0.
                    double noSlipWhirlAcceleration = 0;
                    double noSlipThetaDotDot = 0;
                    // Define tangential velocity
                    double[] tangentialVelocityVector = new double[3]
                    {
                        state.XVelocity[i] - outerRadius * rotationSpeed * sinWhirlAngle,
                        state.YVelocity[i] + outerRadius * rotationSpeed * cosWhirlAngle,
                        state.AxialVelocity[i]
                    };
                    double tangentialMagnitude = Math.Sqrt(tangentialVelocityVector[0] * tangentialVelocityVector[0] + tangentialVelocityVector[1] * tangentialVelocityVector[1] + tangentialVelocityVector[2] * tangentialVelocityVector[2]) + Constants.RegularizationCoefficient;
                    // Get tangential direction unit vector
                    double[] tangentialDirection = new double[3]
                    {
                        tangentialVelocityVector[0]/tangentialMagnitude,
                        tangentialVelocityVector[1]/tangentialMagnitude,
                        tangentialVelocityVector[2]/tangentialMagnitude
                    };
                    double coulombStaticForceMagnitude = parameters.Friction.StaticFrictionCoefficient[i] * normalCollisionForce;                                        
                    double coulombFrictionLateral = 0;  
                    double coulombFrictionX;
                    double coulombFrictionY;                    
                    double coulombFrictionZ;
                    double staticFriction;
                    double stribeckFriction = Math.Abs(
                        normalCollisionForce * 
                            (
                                parameters.Friction.KinematicFrictionCoefficient[i] + 
                                (
                                        parameters.Friction.StaticFrictionCoefficient[i] - parameters.Friction.KinematicFrictionCoefficient[i]
                                ) * Math.Exp( - parameters.Friction.stribeck *  tangentialMagnitude)
                            )
                        );
                    double kinematicFriction = Math.Abs( normalCollisionForce * parameters.Friction.KinematicFrictionCoefficient[i]);
            
                    //Calculate the no slip conditions 
                    if (Math.Abs(axialVelocity) < 1e-6)
                    {
                        // The no slip condition: noSlipThetaDot * Router + whirlVelocity * radialDisplacement = 0
                        // No slide acceleration: noSlipThetaDotDot = - (noSlipWhirlAcceleration * radialDisplacement + radialVelocity * whirlVelocity)/Router
                        // noSlipWhirlAcceleration = (xDotDot * sinWhirlAngle - yDotDot * cosWhirlAngle - 2 * radialVelocity * whirlVelocity) / radialDisplacement
                        // xDotDot = sumForceX/Mass, yDotDot = sumForceY/Mass
                        double xDotDot = sumForcesX / parameters.Drillstring.LumpedElementMass[i];
                        double yDotDot = sumForcesY / parameters.Drillstring.LumpedElementMass[i];
                        noSlipWhirlAcceleration = radialDisplacement == 0 ? 10E5 : (xDotDot * sinWhirlAngle - yDotDot * cosWhirlAngle - 2 * radialVelocity * whirlVelocity) / radialDisplacement;
                        noSlipThetaDotDot = (radialVelocity * whirlVelocity + yDotDot * cosWhirlAngle - xDotDot * sinWhirlAngle) / outerRadius;//- (noSlipWhirlAcceleration * radialDisplacement + radialVelocity * whirlVelocity)/outerRadius;                                            
                    }
                    //  Check for a static condition by evaluating the tangential speed at the contact point.
                    // Note that on a pure rolling condition, must be tangentialMagnitude = 0. This does not imply that there 
                    // the pipe is static. 
                    if (Math.Abs(tangentialMagnitude) < 1E-6)
                    {
                        state.SlipCondition[i] = 0;
                        double tangentialForceProjection = Math.Abs(sumForcesX * tangentialDirection[0] + sumForcesY * tangentialDirection[1] + sumForcesZ * tangentialDirection[2]);
                        //if the total of forces applied in the tangential direction is smaller than the maximum static friction force, then it is a no slip condition. Otherwise, it is a slip condition and the friction is equal to the stribeck friction.
                        if (tangentialForceProjection < coulombStaticForceMagnitude)
                        {
                            state.SlipCondition[i] = 0;                        
                            //Projects into the lateral and axial directions
                            coulombFrictionX = - tangentialForceProjection * tangentialDirection[0];
                            coulombFrictionY = - tangentialForceProjection * tangentialDirection[1];
                            coulombFrictionZ = - tangentialForceProjection * tangentialDirection[2];    
                        }
                        else
                        {
                            state.SlipCondition[i] = 1;
                            //Projects into the lateral and axial directions
                            coulombFrictionX = - kinematicFriction * tangentialDirection[0];
                            coulombFrictionY = - kinematicFriction * tangentialDirection[1];
                            coulombFrictionZ = - kinematicFriction * tangentialDirection[2];    
                        }
                    } 
                    else
                    {                                
                        //Projects into the lateral and axial directions
                        coulombFrictionX = - kinematicFriction * tangentialDirection[0];
                        coulombFrictionY = - kinematicFriction * tangentialDirection[1];
                        coulombFrictionZ = - kinematicFriction * tangentialDirection[2];    
                    }
                    //Store no slip data
                    if (i == parameters.Drillstring.IndexSensor)
                    {
                        state.PhiDdotNoSlipSensor = noSlipWhirlAcceleration;
                        state.ThetaDotNoSlipSensor = noSlipThetaDotDot;
                    }
                    // It needs to be zeroed before, as there might have changes in the sleeve position when the number of lumped parameters change 
                    state.SleeveForces[i] = 0;                
          
                    //Update relevant forces and torque with the calculated coulomb friction force
                    frictionTorque = hasSleeve ? sleeveBrakeForce * outerRadius : (outerRadius * (coulombFrictionX * sinWhirlAngle - coulombFrictionY * cosWhirlAngle));
                    if (hasSleeve)
                    {
                        axialCoulombFrictionForce = coulombFrictionZ *  (1 - parameters.Drillstring.AxialFrictionReduction);
                        double tangentialSleeveForce = Math.Sqrt(coulombFrictionX * coulombFrictionX + coulombFrictionY * coulombFrictionY - axialCoulombFrictionForce * axialCoulombFrictionForce) * Math.Sign(coulombFrictionLateral);
                        coulombFrictionX = - tangentialSleeveForce * tangentialDirection[0];
                        coulombFrictionY = - tangentialSleeveForce * tangentialDirection[1];
                        coulombFrictionZ = axialCoulombFrictionForce;                   
                        state.SleeveForces[i] = hasSleeve ? coulombFrictionLateral : 0;                        
                    }   
                    //Update forces
                    sumForcesX += coulombFrictionX;
                    sumForcesY += coulombFrictionY;
                    sumForcesZ += coulombFrictionZ;                                          
                }
                else
                {
                    frictionTorque = 0.0;
                    state.SleeveForces[i] = 0;
                    state.PhiDdotNoSlipSensor = 0.0;
                    state.ThetaDotNoSlipSensor = 0.0;
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
                XAcceleration = sumForcesX / parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i];
                YAcceleration= sumForcesY / parameters.Drillstring.LumpedElementMass[i] + parameters.Drillstring.FluidAddedMass[i];
                ZAcceleration = sumForcesZ / parameters.Drillstring.LumpedElementMass[i];                

                //Update state
                state.NormalCollisionForce[i] = normalCollisionForce;
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