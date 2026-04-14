using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    /// <summary>
    ///     LumpedElementModel comes from the IModel interface. In here, it is expected to 
    /// have an "PrepareModel" method which is used to apply any necessary operations before
    /// it enters the integration loop. Those can be, for instance, updating the setpoints 
    /// from the simulator.
    /// 
    ///     CalculateAccelerations is another method where the force models are to be implemented
    /// note that the method for calculating accelerations can be shared through several other
    /// models. In other words, how friction/impact is calculated can be the same whether it is 
    /// a Finite Element, Finite Difference or a Lumped Element model.
    /// </summary>
    public class LumpedElementModel : IModel<LumpedElementModel>
    {       
        public int NumberOfElements; // Number of elements is one less than the number of nodes
        private Vector<double> tension;
        private Vector<double> torque;
        // Bore hole properties
        private double wallStiffness = 5e7;
        private double wallDamping = 5e4;
        
        // Structural properties
        private Matrix<double> lateralStiffnessMatrix;
        private Matrix<double> axialStiffnessMatrix;
        private Matrix<double> torsionalStiffnessMatrix;
        private double[] lateralStiffnessLeft;
        private double[] lateralStiffnessMid;
        private double[] lateralStiffnessRight;
        private double[] axialStiffnessLeft;
        private double[] axialStiffnessMid;
        private double[] axialStiffnessRight;
        private double[] torsionalStiffnessLeft;
        private double[] torsionalStiffnessMid;
        private double[] torsionalStiffnessRight;
        private double[] lateralLumpedMass;
        private double[] axialLumpedMass;
        private double[] torsionalLumpedInertia;
        
        // Active drill-string elements
        private Vector<double> bendingStiffness;
        // Bit-rock interaction related variables
        private IBitRock bitRockModel;        
        private BitInternalForces bitInternalForces;
        // Pre-stress forces in the normal and binormal direction, used for calculating the contact forces in the friction model. These are calculated in the PrepareModel step and then used in the CalculateAccelerations step.
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
        private double xAcceleration;
        private double yAcceleration;
        private double zAcceleration;
        private double rotationSpeed;
        private double rotationSpeedSquared;
        private double rotationAngle;
        private double rotationAcceleration;
        private double axialVelocity;   
        // Axial and torsional data for coupling.           
        private double torqueOnBit;
        double topDriveTorque;
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
        private double sumTorque;            
        private double sleeveBrakeForce;            
        //Elastic force-related variables
        private double XiMinus1;
        private double YiMinus1;
        private double XiPlus1;
        private double YiPlus1;
        private double kMinus1;
        private double kPlus1;
        private double elasticForceX;
        private double elasticForceY;
        private double elasticForceZ;
        private double elasticForcePhi;
        

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
        
        public LumpedElementModel(in SimulationParameters parameters, 
            in IBitRock bitRockModel)
        {
            // Initialize the bit-rock model
            this.bitRockModel = bitRockModel;
            bitInternalForces = new BitInternalForces();
            // Create a simplifiefd drill-string form the input
            NumberOfElements = parameters.Drillstring.ElementLength.Count;            
            #region Mass and stiffness matrices
            // In here, the global and element mass and stiffness matrices are created
            // After I populate the matrices, I want to split into 3 vector:
            // left of diagonal, diagonal and right of diagonal for all DoF.
            lateralStiffnessLeft = new double[NumberOfElements + 1];
            lateralStiffnessMid = new double[NumberOfElements + 1];
            lateralStiffnessRight = new double[NumberOfElements + 1];            
            lateralStiffnessLeft[0] = 0.0;
            lateralStiffnessRight[NumberOfElements] = 0.0;

            axialStiffnessLeft = new double[NumberOfElements + 1];
            axialStiffnessMid = new double[NumberOfElements + 1];
            axialStiffnessRight = new double[NumberOfElements + 1];
            
            torsionalStiffnessLeft = new double[NumberOfElements + 1];
            torsionalStiffnessMid = new double[NumberOfElements + 1];
            torsionalStiffnessRight = new double[NumberOfElements + 1];

            axialLumpedMass = new double[NumberOfElements + 1];
            lateralLumpedMass = new double[NumberOfElements + 1];
            torsionalLumpedInertia = new double[NumberOfElements + 1];
            // CHANGE TO SPARSE MATRICES??            
            lateralStiffnessMatrix = Matrix<double>.Build.Dense(NumberOfElements + 1, NumberOfElements + 1);
            axialStiffnessMatrix = Matrix<double>.Build.Dense(NumberOfElements+ 1, NumberOfElements + 1);
            torsionalStiffnessMatrix = Matrix<double>.Build.Dense(NumberOfElements + 1, NumberOfElements + 1);
            for (int i = 0; i < NumberOfElements; i++)
            {
                //  The equivalent mass for each DoF is dependent on the assumed mode shape. Axial and Torsional modes use a linear element, which is
                //compatible to a rigid body motion. The calculation can be foundEach element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx. 
                // The lateral modes use a half-sine shape, which is compatible with a simply supported beam with C1 continuity. When the mass element is lumped, it falls
                // back to the conventional 0.5 * rho * A * L for each node.
                double axialMass = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementArea[i] * parameters.Drillstring.ElementLength[i] / 2.0;
                double torsionalMass = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementInertia[i] * parameters.Drillstring.ElementLength[i];
                double lateralMass = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementArea[i] * parameters.Drillstring.ElementLength[i] / 2.0;                
                axialLumpedMass[i] += axialMass; 
                lateralLumpedMass[i] += lateralMass;
                torsionalLumpedInertia[i] += torsionalMass;
                double axialStiffness = parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementArea[i] / parameters.Drillstring.ElementLength[i];
                double lateralStiffness = Math.PI * Math.PI * Math.PI * parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementArea[i] 
                            / parameters.Drillstring.ElementLength[i];
                // The torsional stiffness is calculated as 2 * G * J / L, where G is the shear modulus, 
                // J is the polar moment of inertia and L is the length of the element. The polar moment of inertia is calculated as I * 2, 
                // where I is the area moment of inertia. Each element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx 
                double torsionalStiffness = 2.0 * parameters.Drillstring.ElementShearModuli[i] * parameters.Drillstring.ElementInertia[i] / parameters.Drillstring.ElementLength[i];;         
                //   Add the local elements to the global 
                // element matrix.
                // ---------- Lateral Stiffness -----------  
                lateralStiffnessLeft[i + 1] = - lateralStiffness;
                lateralStiffnessMid[i] += lateralStiffness;
                lateralStiffnessMid[i + 1] += lateralStiffness;
                lateralStiffnessRight[i] = - lateralStiffness;
                // ---------- Axial Stiffness -----------
                axialStiffnessLeft[i + 1] = - axialStiffness;
                axialStiffnessMid[i] += axialStiffness;
                axialStiffnessMid[i + 1] += axialStiffness;
                axialStiffnessRight[i] = - axialStiffness;
                // ---------- Torsional Stiffness -----------       
                torsionalStiffnessLeft[i + 1] = - torsionalStiffness;
                torsionalStiffnessMid[i] += torsionalStiffness;
                torsionalStiffnessMid[i + 1] += torsionalStiffness;
                torsionalStiffnessRight[i] = - torsionalStiffness;
                // ---------- Matrix notation --------------
                // Lateral stiffness matrix
                lateralStiffnessMatrix[i, i] += lateralStiffness;
                lateralStiffnessMatrix[i + 1, i] -= lateralStiffness;
                lateralStiffnessMatrix[i, i + 1] -= lateralStiffness;
                lateralStiffnessMatrix[i + 1, i + 1] += lateralStiffness;                
                // Axial stiffness matrix
                axialStiffnessMatrix[i, i] += axialStiffness;
                axialStiffnessMatrix[i + 1, i] -= axialStiffness;
                axialStiffnessMatrix[i, i + 1] -= axialStiffness;
                axialStiffnessMatrix[i + 1, i + 1] += axialStiffness;
                // Torsional stiffness matrix 
                torsionalStiffnessMatrix[i, i] += torsionalStiffness;
                torsionalStiffnessMatrix[i + 1, i] -= torsionalStiffness;
                torsionalStiffnessMatrix[i, i + 1] -= torsionalStiffness;
                torsionalStiffnessMatrix[i + 1, i + 1] += torsionalStiffness;                
            }
            //  Keeping the matricial notation of the mass and stiffness is not required.
            // However, it is very convenient to have them as it can be used for modal analysis,
            // to calculate natural frequencies, mode shapes, frequency domain responses and so forth.                      
            #endregion
            // Allocate main variables for the model.
            //NumberOfElements = simulationParameters.LumpedCells.NumberOfLumpedElements;
            bendingStiffness = Vector<double>.Build.Dense(NumberOfElements+1);
            preStressNormalForce = Vector<double>.Build.Dense(NumberOfElements);
            preStressBinormalForce = Vector<double>.Build.Dense(NumberOfElements);
      
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
                momentX = simulationParameters.Drillstring.ElementYoungModuli[i]
                         * simulationParameters.Drillstring.ElementInertia[i]
                         * d2Xdz2; // Bending moment x-component
                momentY = simulationParameters.Drillstring.ElementYoungModuli[i]
                         * simulationParameters.Drillstring.ElementInertia[i]
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
                isFirst = i == 0;
                int offPhaseIndex = isFirst ? 0 : i - 1;
                // Calculate the axial elastic force: E * A 
                double ZiMinus1 = (i == 0) ? 0.0 : state.ZDisplacement[i - 1];
                double ZiPlus1 = (i == state.ZDisplacement.Count - 1) ? 0.0 : state.ZDisplacement[i + 1];            
                axialForce = CalculateAxialElasticForce(i, state) ;
               
                // Get the current property and repeat the fist as a boundary condition
                differentialTrajectoryPhi = parameters.Trajectory.DiffPhiInterpolated[offPhaseIndex];
                differentialTrajectoryTheta = parameters.Trajectory.DiffThetaInterpolated[offPhaseIndex];    
                trajectoryTheta = parameters.Trajectory.InterpolatedTheta[offPhaseIndex];
                innerArea = parameters.Drillstring.ElementInnerArea[offPhaseIndex];
                outerArea = parameters.Drillstring.ElementOuterArea[offPhaseIndex];
                localBendingStiffness = isFirst ? lateralStiffnessLeft[0] : lateralStiffnessMid[i-1];
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
                bendingStiffness[i] = - Math.Max(localBendingStiffness - Math.Pow(Math.PI, 2) * Tension / (parameters.Drillstring.ElementLength[i]), 0.0);            
                torque[i] = CalculateTorsionalElasticForce(i, state);
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
            for (int i = 0; i < NumberOfElements; i++)
            {     
                isFirst = i == 0;
                // Normal force components in Frenet-Serret coordinate system
                differentialTorque = isFirst ?  0 : (torque[i+1] - torque[i]) / parameters.Drillstring.ElementLength[i];
                InertiaTimesYoungModulus = parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementInertia[i];
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
                preStressNormalForce[i] = 0.5 * (oldNormalForce + normalForce) * parameters.Drillstring.ElementLength[i];
                preStressBinormalForce[i] = 0.5 * (oldBinormalForce + binormalForce) * parameters.Drillstring.ElementLength[i];
                //Update normal and binormal values from the 
                oldNormalForce = normalForce;
                oldBinormalForce = binormalForce;     
            }                         
        }
        private double CalculateAxialElasticForce(int i, State state)
        {
            double ZiMinus1 = (i == 0) ? state.TopOfStringRelativeAxialPosition : state.ZDisplacement[i - 1];
            double ZiPlus1 = (i == state.ZDisplacement.Count - 1) ? 0.0 : state.ZDisplacement[i + 1];            
            return - ( axialStiffnessLeft[i] * ZiMinus1  + axialStiffnessMid[i] * state.ZDisplacement[i] + axialStiffnessRight[i] * ZiPlus1 );
        }
        private double CalculateTorsionalElasticForce(int i, State state)
        {
            double PhiMinus1 = (i == 0) ? 0.0 : state.AngularDisplacement[i - 1];
            double PhiPlus1 = (i == state.AngularDisplacement.Count - 1) ? 0.0 : state.AngularDisplacement[i + 1];                                                
            return - ( torsionalStiffnessLeft[i] * PhiMinus1  + torsionalStiffnessMid[i] * state.AngularDisplacement[i] + torsionalStiffnessRight[i] * PhiPlus1 );
        }

        public void CalculateAccelerations(State state, in SimulationParameters parameters)
        {

            /* For the addition - The axial and shear strains must be calculated for the bit-rock interaction */
            #region Bit - rock interaction
            #endregion

            #region Mud motors properties            
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
            #endregion
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
                #region Elastic Forces Calculation
                //  Calculated the torque and force difference in each element (TorqueElement and TorqueNextElement) using the strains 
                //obtained from the axial and the torsional models. As they heve different indexes, they need to be re-aligned with the
                //lateral elements.
                //If it is the last element, use the torque on bit
                // ------------------ Axial Elastic Forces Calculation --------------------       
                elasticForceZ = CalculateAxialElasticForce(i, state);                
                // ------------------ Torsional Elastic Forces Calculation --------------------
                elasticForcePhi = CalculateTorsionalElasticForce(i, state);
                // ------------------ Lateral Elastic Forces Calculation ------------------
                // If it is the first element, get the pinned boundary condition
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Same with the stiffness
                elasticForceX = - ( lateralStiffnessLeft[i] * XiMinus1  + lateralStiffnessMid[i] * state.XDisplacement[i] + lateralStiffnessRight[i] * XiPlus1 );
                elasticForceY = - ( lateralStiffnessLeft[i] * YiMinus1  + lateralStiffnessMid[i] * state.YDisplacement[i] + lateralStiffnessRight[i] * YiPlus1 );
                #endregion
                #region Pre-Stress Force Calculation
                PreStressForceX = preStressNormalForce[i] * Math.Sin(toolFaceAngle[i]) + 
                                preStressBinormalForce[i] * Math.Cos(toolFaceAngle[i]) ;
                PreStressForceY = preStressNormalForce[i] * Math.Cos(toolFaceAngle[i]) - 
                                preStressBinormalForce[i] * Math.Sin(toolFaceAngle[i]) ;                                    
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
                fluidDampingCoefficient = parameters.Flow.FluidDampingCoefficient / parameters.Drillstring.ElementFluidAddedMass[i];                    
                fluidForceX = - parameters.Drillstring.ElementFluidAddedMass[i] * 
                                    (
                                        fluidDampingCoefficient * state.XVelocity[i]
                                        - 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                        + rotationSpeed * state.YVelocity[i]
                                        + 0.5 * fluidDampingCoefficient * rotationSpeed * state.YDisplacement[i]
                                    );
                fluidForceY = - parameters.Drillstring.ElementFluidAddedMass[i] * 
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
                unbalanceForceX = parameters.Drillstring.ElementEccentricMass[i] * parameters.Drillstring.ElementEccentricity[i]*
                    (
                        rotationSpeedSquared * Math.Cos(rotationAngle)
                        + state.AngularAcceleration[i] * Math.Sin(rotationAngle)
                    );
                unbalanceForceY = parameters.Drillstring.ElementEccentricMass[i] * parameters.Drillstring.ElementEccentricity[i]* 
                    (
                        rotationSpeedSquared * Math.Sin(rotationAngle)
                        - rotationAcceleration * Math.Cos(rotationAngle)
                    );                                                            
                #endregion                                        
                #region Coulomb Friction
                // Axial velocity
                axialVelocity = state.ZVelocity[i];
                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.ElementOuterRadius[i];
                //double innerRadius = hasSleeve ? parameters.Drillstring.SleeveInnerRadius : parameters.Drillstring.OuterRadius[i];  
                inertia = hasSleeve ? parameters.Drillstring.SleeveMassMomentOfInertia : parameters.Drillstring.ElementInertia[i];
                // The total normal force is the sum of elastic, pre-stress, fluid, unbalance forces, damping and colission forces                        
                sumForcesX = elasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - parameters.Drillstring.CalculateLateralDamping * state.XVelocity[i] - normalCollisionForce * cosWhirlAngle;
                sumForcesY = elasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - parameters.Drillstring.CalculateLateralDamping * state.YVelocity[i] - normalCollisionForce * sinWhirlAngle;
                sumForcesZ = elasticForceZ - parameters.Drillstring.CalculatedAxialDamping * state.ZVelocity[i]; 
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
                        double xDotDot = sumForcesX / lateralLumpedMass[i];
                        double yDotDot = sumForcesY / lateralLumpedMass[i];
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
                #region Bit-Rock Interaction
                if (i == NumberOfElements)
                {
                    // Populate the bit internal forces accordingly
                    bitInternalForces.ElasticAxialForce = elasticForceZ;
                    bitInternalForces.ElasticTorque = elasticForcePhi;                               
                    //  Calculate interaction forces on bit based on selected bit-rock model 
                    // and update the state accordingly                
                    bitRockModel.CalculateInteractionForce(state, in parameters, in bitInternalForces);
                    bitRockModel.ManageStickingOnBottom(state, in parameters, in bitInternalForces);           
                }
                #endregion
                #region Accelerations                              
                if (hasSleeve)
                {                    
                    // Why is there a TimeStep in here?                    
                    state.SleeveAngularAcceleration[sleeveIndex] = parameters.InnerLoopTimeStep * (sleeveBrakeForce * parameters.Drillstring.SleeveInnerRadius - parameters.Drillstring.SleeveOuterRadius * state.SleeveForces[i]) / parameters.Drillstring.SleeveMassMomentOfInertia;;
                }
                // The torque on bit only apply on the last element
                torqueOnBit = (i == NumberOfElements) ? (   parameters.UseMudMotor ? state.MudTorque : state.TorqueOnBit   )    :   0.0   ;
                // Topdrive Torque only applies to the first element                
                topDriveTorque = (i == 0) ? state.TopDrive.TopDriveMotorTorque : 0.0;
                sumTorque = elasticForcePhi  + topDriveTorque + torqueOnBit - parameters.Drillstring.CalculatedTorsionalDamping * rotationSpeed
                             - frictionTorque;
                // Variables are generated locally to facilitate debugging only.
                angularAcceleration = sumTorque / torsionalLumpedInertia[i];      
                xAcceleration = sumForcesX / (lateralLumpedMass[i] + parameters.Drillstring.ElementFluidAddedMass[i]);
                yAcceleration = sumForcesY / (lateralLumpedMass[i] + parameters.Drillstring.ElementFluidAddedMass[i]);
                zAcceleration = sumForcesZ / axialLumpedMass[i];                
                //Update state
                state.NormalCollisionForce[i] = normalCollisionForce;
                state.RadialDisplacement[i] = radialDisplacement;
                state.RadialVelocity[i] = radialVelocity;
                state.WhirlAngle[i] = whirlAngle;
                state.WhirlVelocity[i] = whirlVelocity;
                state.ZAcceleration[i] = zAcceleration;
                state.XAcceleration[i] = xAcceleration;
                state.YAcceleration[i] = yAcceleration;   
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
                    if (double.IsNaN(xAcceleration) || double.IsNaN(yAcceleration) || double.IsNaN(zAcceleration) || double.IsNaN(state.AngularAcceleration[i]))
                    {
                        Console.WriteLine("NaN detected in lateral calculations at element " + i.ToString() +
                            " XAcc: " + xAcceleration.ToString() +
                            " YAcc: " + yAcceleration.ToString() +
                            " ZAcc: " + zAcceleration.ToString() +
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