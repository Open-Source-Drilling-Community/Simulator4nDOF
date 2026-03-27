using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class LateralModel : IModel<LateralModel>
    {       
        public int NumberOfElements; // Number of elements is one less than the number of nodes

        private Vector<double> tension;
        private Vector<double> torque;
        
        private Vector<double> bendingStiffness;
        private Vector<double> preStressNormalForce;
        private Vector<double> preStressBinormalForce;                  
        private Vector<double> tensionIntegral;        
        private Vector<double> toolFaceAngle;
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
        // Friction related variables
        private double coulombFrictionX;
        private double coulombFrictionY;                    
        private double coulombFrictionZ;

                   
        
        public LateralModel(SimulationParameters simulationParameters)
        {
            NumberOfElements = simulationParameters.LumpedCells.NumberOfLumpedElements;
            bendingStiffness = Vector<double>.Build.Dense(NumberOfElements+1);
            preStressNormalForce = Vector<double>.Build.Dense(NumberOfElements);
            preStressBinormalForce = Vector<double>.Build.Dense(NumberOfElements);
            //ScalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            toolFaceAngle = Vector<double>.Build.Dense(NumberOfElements);
            tensionIntegral = Vector<double>.Build.Dense(NumberOfElements);
            tension = Vector<double>.Build.Dense(NumberOfElements + 1);
            torque = Vector<double>.Build.Dense(NumberOfElements + 1);
        }
        public void UpdateBendingMoments(State state, SimulationParameters simulationParameters)
        {
            //double XiMinus1, YiMinus1, XiPlus1, YiPlus1;
            double invElementLengthSquared = 1.0 / (simulationParameters.LumpedCells.ElementLength * simulationParameters.LumpedCells.ElementLength);
            double momentX, momentY;
            double d2Xdz2, d2Ydz2; 
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                //XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                //YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                //XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                //YiPlus1 = (i == state.YDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];

                // Calculate the derivatives using the half-sine first mode hypothesis
                d2Xdz2 = - state.XDisplacement[i] * Math.PI * Math.PI * invElementLengthSquared; 
                d2Ydz2 = - state.YDisplacement[i] * Math.PI * Math.PI * invElementLengthSquared; 

                //Calcualte the bending moments using the central difference scheme
                momentX = simulationParameters.Drillstring.YoungModuli[i]
                         * simulationParameters.Drillstring.PipeInertia[i]
                         * d2Xdz2; // Bending moment x-component
                momentY = simulationParameters.Drillstring.YoungModuli[i]
                         * simulationParameters.Drillstring.PipeInertia[i]
                         * d2Ydz2; //
                
                state.BendingMomentX[i] =  momentX;
                state.BendingMomentY[i] =  momentY;
            }
        }
        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
        
            double axialForce;
            double differentialTrajectoryPhi;
            double differentialTrajectoryTheta;
            double trajectoryTheta;
            double Tension;
            double tensionIntegralTemp = 0;
            double softStringTempTerm1;
            double softStringTempTerm2;
            double differentialNormalForceSoftString;
            double normalForceSoftString = 0;
            double innerArea;
            double outerArea;
            double oldDifferencialNormalForceSoftString = 0.0;
            double localBendingStiffness;
            bool isFirst;
            int revIdx;
            for (int i = 0; i < parameters.LumpedCells.NumberOfLumpedElements; i++)
            {
                // Index for reverse loop
                revIdx = parameters.LumpedCells.NumberOfLumpedElements - i;
                tensionIntegralTemp += 0.5*(parameters.Flow.dSigmaDx[revIdx] + parameters.Flow.dSigmaDx[revIdx - 1])/ parameters.LumpedCells.ElementLength;
                tensionIntegral[i] = tensionIntegralTemp;
            }
            // Loop to compute the tensions and the stiffness of the model
            for (int i = 0; i < parameters.LumpedCells.NumberOfLumpedElements + 1; i++)
            {
                int torsionalToLateralIndex = (i == 0 || i == 1) ? 0 : i * parameters.DistributedCells.LateralModelToWaveRatio - 1;
                isFirst = i == 0;
                int offPhaseIndex = isFirst ? 0 : i - 1;
                // Calculate the axial elastic force: E * A 
                axialForce = parameters.Drillstring.YoungModuli[offPhaseIndex]
                            * parameters.Drillstring.PipeArea[offPhaseIndex]
                            * state.AxialStrain[torsionalToLateralIndex];
                
                // Get the current property and repeat the fist as a boundary condition
                differentialTrajectoryPhi = parameters.Trajectory.DiffPhiInterpolated[offPhaseIndex];
                differentialTrajectoryTheta = parameters.Trajectory.DiffThetaInterpolated[offPhaseIndex];    
                trajectoryTheta = parameters.Trajectory.InterpolatedTheta[offPhaseIndex];
                innerArea = parameters.Drillstring.InnerArea[offPhaseIndex];
                outerArea = parameters.Drillstring.OuterArea[offPhaseIndex];
                localBendingStiffness = isFirst ? parameters.Drillstring.BendingStiffness[0] : parameters.Drillstring.BendingStiffness[i-1];
                // Index for reverse loop
                revIdx = i == parameters.LumpedCells.NumberOfLumpedElements ? 0 : parameters.LumpedCells.NumberOfLumpedElements - i - 1;                
                Tension = tensionIntegral[revIdx] + parameters.Flow.AxialBuoyancyForceChangeOfDiameters[i] - axialForce;
                softStringTempTerm1 = (Tension + parameters.Flow.AxialBuoyancyForceChangeOfDiameters[i]) * differentialTrajectoryTheta - parameters.Flow.BuoyantWeightPerLength[i] * Math.Cos(trajectoryTheta); 
                softStringTempTerm2 = (Tension + parameters.Flow.AxialBuoyancyForceChangeOfDiameters[i]) * differentialTrajectoryPhi - parameters.Flow.BuoyantWeightPerLength[i] * Math.Sin(trajectoryTheta); 
                differentialNormalForceSoftString = Math.Sqrt(softStringTempTerm1 * softStringTempTerm1 + softStringTempTerm2 * softStringTempTerm2);
                // If it is the first step, assumes it is the same as the current
                if ( isFirst)
                {
                    oldDifferencialNormalForceSoftString = differentialNormalForceSoftString;
                }
                // Update the normal force
                normalForceSoftString += 0.5 * (differentialNormalForceSoftString + differentialNormalForceSoftString) * parameters.LumpedCells.ElementLength;
                Tension += (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                            (
                                outerArea * (parameters.Flow.AnnulusPressure[i] - parameters.Flow.HydrostaticAnnulusPressure[i])
                              - innerArea * (parameters.Flow.StringPressure[i] - parameters.Flow.HydrostaticStringPressure[i])
                            );
                // Update the model
                tension[i] = Tension;
                //This step checks if there is a negative element and sets to zero
                bendingStiffness[i] = - Math.Max(localBendingStiffness - Math.Pow(Math.PI, 2) * Tension / (2 * parameters.Drillstring.PipeLengthForBending), 0.0);                                            
                torque[i] = state.ShearStrain[torsionalToLateralIndex] 
                    * parameters.Drillstring.PipePolarMoment[offPhaseIndex] 
                    * parameters.Drillstring.ShearModuli[offPhaseIndex];
                //Roll over for integration
                oldDifferencialNormalForceSoftString = differentialNormalForceSoftString;
            }
            //bendingStiffness[0] = bendingStiffness[1];
            //torque[0] = torque[1];
            //tension[0] = tension[1];
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
            // Loop to compute the pre-stresses
            for (int i = 0; i < parameters.LumpedCells.NumberOfLumpedElements; i++)
            {     
                isFirst = i == 0;
                // Normal force components in Frenet-Serret coordinate system
                differentialTorque = isFirst ?  0 : (torque[i+1] - torque[i]) / parameters.LumpedCells.DistanceBetweenElements;
                InertiaTimesYoungModulus = parameters.Drillstring.YoungModuli[i] * parameters.Drillstring.PipeInertia[i];
                binormalForce = parameters.Flow.BuoyantWeightPerLength[i] * parameters.Trajectory.bz[i] 
                                + parameters.Trajectory.Curvature[i] * differentialTorque 
                                + parameters.Trajectory.CurvatureDerivative[i] * torque[i+1] 
                                - 2 * InertiaTimesYoungModulus * parameters.Trajectory.CurvatureDerivative[i] * parameters.Trajectory.Torsion[i] 
                                - InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * parameters.Trajectory.TorsionDerivative[i];
                normalForce = parameters.Trajectory.Curvature[i] * (tension[i+1] + parameters.Flow.NormalBuoyancyForceChangeOfDiameters[i+1] - parameters.Trajectory.Torsion[i] * torque[i+1]) 
                                + parameters.Flow.BuoyantWeightPerLength[i] * parameters.Trajectory.nz[i] 
                                - InertiaTimesYoungModulus * parameters.Trajectory.CurvatureSecondDerivative[i] 
                                + InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * (parameters.Trajectory.Torsion[i] * parameters.Trajectory.Torsion[i]);
                //At the first step, repeat the values
                if (isFirst)
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
                toolFaceAngle[i] = Math.Acos(dotProduct) * signToolFace;
                //This is equivalent of integrating with the trapezoidal rule and then getting the difference.                    
                preStressNormalForce[i] = 0.5 * (oldNormalForce + normalForce) * parameters.LumpedCells.ElementLength;
                preStressBinormalForce[i] = 0.5 * (oldBinormalForce + binormalForce) * parameters.LumpedCells.ElementLength;
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
                //  Calculated the torque and force difference in each element (TorqueElement and TorqueNextElement) using the strains 
                //obtained from the axial and the torsional models. As they heve different indexes, they need to be re-aligned with the
                //lateral elements.
                int idx = i == 0 ? 0 : i * parameters.DistributedCells.LateralModelToWaveRatio - 1;                          
                torqueElement = parameters.Drillstring.PipePolarMoment[i] 
                                 * parameters.Drillstring.ShearModuli[i] 
                                 * state.ShearStrain[idx];
                forceElement = parameters.Drillstring.PipeArea[i]
                                 * parameters.Drillstring.YoungModuli[i]
                                 * state.AxialStrain[idx];
                    
                //If it is the last element, use the torque on bit
                torqueNextElement = (i == state.XDisplacement.Count - 1) ?  torqueOnBit : 
                                parameters.Drillstring.PipePolarMoment[i + 1]
                                 * parameters.Drillstring.ShearModuli[i + 1]
                                 * state.ShearStrain[idx + 1];
                    
                forceNextElement = (i == state.XDisplacement.Count - 1) ? state.WeightOnBit : 
                                parameters.Drillstring.PipeArea[i + 1]
                                 * parameters.Drillstring.YoungModuli[i + 1]
                                 * state.AxialStrain[idx + 1];
                    
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
                kMinus1 = this.bendingStiffness[i];
                kPlus1 = this.bendingStiffness[i + 1];
                ElasticForceX = kMinus1 * XiMinus1  - (kMinus1 + kPlus1) * state.XDisplacement[i] + kPlus1 * XiPlus1;
                ElasticForceY = kMinus1 * YiMinus1  - (kMinus1 + kPlus1) * state.YDisplacement[i] + kPlus1 * YiPlus1;
                #endregion
                #region Pre-Stress Force Calculation
                PreStressForceX = this.preStressNormalForce[i] * Math.Sin(this.toolFaceAngle[i]) + 
                                this.preStressBinormalForce[i] * Math.Cos(this.toolFaceAngle[i]) ;
                PreStressForceY = this.preStressNormalForce[i] * Math.Cos(this.toolFaceAngle[i]) - 
                                this.preStressBinormalForce[i] * Math.Sin(this.toolFaceAngle[i]) ;                                    
                #endregion
                #region Fluid Force Calculation
                //Extracts angular speed depending on if it is a sleeve or not
                bool hasSleeve = state.SleeveToLumpedIndex[i] != -1;
                //If it is not used properly, it should crash the code.
                int sleeveIndex = hasSleeve ?  state.SleeveToLumpedIndex[i] : -1;
                //  Mask between sleeve and non-sleeve nodes
                // The speed comes from the torsional model, which is more detailed. Thus, the converted index must me used
                rotationSpeed = hasSleeve ? state.SleeveAngularVelocity[sleeveIndex] : state.AngularVelocity[i];                            
                rotationSpeedSquared = rotationSpeed * rotationSpeed;
                fluidDampingCoefficient = parameters.Flow.FluidDampingCoefficient / parameters.Drillstring.FluidAddedMass[i];                    
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
                axialVelocity = state.ZVelocity[i];
                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.OuterRadius[i];
                //double innerRadius = hasSleeve ? parameters.Drillstring.SleeveInnerRadius : parameters.Drillstring.OuterRadius[i];  
                inertia = hasSleeve ? parameters.Drillstring.SleeveMassMomentOfInertia : parameters.Drillstring.LumpedElementMomentOfInertia[i];
                // The total normal force is the sum of elastic, pre-stress, fluid, unbalance forces, damping and colission forces                        
                sumForcesX = ElasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - parameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - normalCollisionForce * cosWhirlAngle;
                sumForcesY = ElasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - parameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - normalCollisionForce * sinWhirlAngle;
                sumForcesZ = axialForceDifference - parameters.Drillstring.CalculatedAxialDamping * state.ZVelocity[i]; 
                coulombFrictionX = 0.0;
                coulombFrictionY = 0.0;
                coulombFrictionZ = 0.0;                   
                // Skip the calculation if there is no collision 
                if (heavesideStep == 1.0)
                {                                                        
                    // With this 4nDoF model, no slip condition is only possible if vz = 0.
                    double noSlipWhirlAcceleration = 0;
                    double noSlipThetaDotDot = 0;
                    // Define tangential velocity
                    double[] tangentialVelocityVector = new double[3]
                    {
                        state.XVelocity[i] - outerRadius * rotationSpeed * sinWhirlAngle,
                        state.YVelocity[i] + outerRadius * rotationSpeed * cosWhirlAngle,
                        state.ZVelocity[i]
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
                state.ZAcceleration[i] = ZAcceleration;
                state.XAcceleration[i] = XAcceleration;
                state.YAcceleration[i] = YAcceleration;   
                state.AngularAcceleration[i] = angularAcceleration;                       
                state.Tension[i+1] = tension[i];
                state.Torque[i+1] = torque[i];                
                #endregion
                
                #region  Debbugging Outputs                
                //===============================================================================================
                //                                 UNCOMMENT FOR DEBUGGING PURPOSES
                //===============================================================================================
                
                    if (Math.Abs(rotationSpeed) > 20)
                    {
                        int debug = 0;
                    }
                    if (Math.Abs(angularAcceleration) > 10000)
                    {
                        int debug = 0;
                    }
                    if (double.IsNaN(XAcceleration) || double.IsNaN(YAcceleration) || double.IsNaN(ZAcceleration) || double.IsNaN(state.AngularAcceleration[i]))
                    {
                        Console.WriteLine("NaN detected in lateral calculations at element " + i.ToString() +
                            " XAcc: " + XAcceleration.ToString() +
                            " YAcc: " + YAcceleration.ToString() +
                            " ZAcc: " + ZAcceleration.ToString() +
                            " AngularAcc: " + state.AngularAcceleration[i].ToString() +
                            " NormalForce: " + normalCollisionForce.ToString() +
                            " RadialDisp: " + radialDisplacement.ToString() +
                            " RadialVel: " + radialVelocity.ToString() +
                            " WhirlVel: " + whirlVelocity.ToString() +
                            " AxialVel: " + axialVelocity.ToString() +
                            " SlipCondition: " + state.SlipCondition[i].ToString()
                            );
                            return;
                    }                
                #endregion
            }
        }
    }
}