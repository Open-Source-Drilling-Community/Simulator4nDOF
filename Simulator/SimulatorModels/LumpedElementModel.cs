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
        private int numberOfNodes; // Number of elements is one less than the number of nodes
        private Vector<double> tension;
        private Vector<double> torque;        
        // Structural properties
      
        private double[] massProportionalDampingAxial; // Rayleigh proportional damping with mass-only coefficient
        private double[] massProportionalDampingLateral; // Rayleigh proportional damping with mass-only coefficient
        private double[] inertiaProportionalDampingTorsional; // Rayleigh proportional damping with mass-only coefficient
        
        private double[] lateralStiffnessLeft; // Stiffness matrix is quasi-diagonal. This is the term left to the diagonal
        private double[] lateralStiffnessMid; // Stiffness matrix is quasi-diagonal. This is the term on the diagonal
        private double[] lateralStiffnessRight; // Stiffness matrix is quasi-diagonal. This is the term right to the diagonal

        private double[] lateralTensionInducedStiffnessLeft; // Stiffness matrix is quasi-diagonal. This is the tension-induced geometric stiffness term left to the diagonal
        private double[] lateralTensionInducedStiffnessMid; // Stiffness matrix is quasi-diagonal. This is the tension-induced geometric stiffness term on the diagonal
        private double[] lateralTensionInducedStiffnessRight; // Stiffness matrix is quasi-diagonal. This is the tension-induced geometric stiffness term right to the diagonal

        private double[] lateralPressureInducedStiffnessMid; // Stiffness matrix is quasi-diagonal. This is the pressure-induced geometric stiffness term on the diagonal
        private double[] lateralPressureInducedStiffnessLeft; // Stiffness matrix is quasi-diagonal. This is the pressure-induced geometric stiffness term left to the diagonal
        private double[] lateralPressureInducedStiffnessRight; // Stiffness matrix is quasi-diagonal. This is the pressure-induced geometric stiffness term right to the diagonal
        private double[] axialForceDistribution;


        private double[] axialStiffnessLeft; // Stiffness matrix is quasi-diagonal. This is the term left to the diagonal
        private double[] axialStiffnessMid; // Stiffness matrix is quasi-diagonal. This is the term on the diagonal
        private double[] axialStiffnessRight; // Stiffness matrix is quasi-diagonal. This is the term right to the diagonal
        private double[] torsionalStiffnessLeft; // Stiffness matrix is quasi-diagonal. This is the term left to the diagonal
        private double[] torsionalStiffnessMid; // Stiffness matrix is quasi-diagonal. This is the term on the diagonal
        private double[] torsionalStiffnessRight; // Stiffness matrix is quasi-diagonal. This is the term right to the diagonal
        private double[] lateralLumpedMass; // Lumped mass is diagonal. So can be represented as a vector
        private double[] axialLumpedMass; // Lumped mass is diagonal. So can be represented as a vector
        private double[] nodeEccentricity; // Lumped mass is diagonal. So can be represented as a vector
        private double[] addedLateralFluidMass; // Lumped mass is diagonal. So can be represented as a vector        
        private double[] torsionalLumpedInertia; // Lumped mass is diagonal. So can be represented as a vector
        private double[] secondMomentOfArea; // Lumped mass is diagonal. So can be represented as a vector
        private double[] quasiStaticPressureDifference; // Quasi-static pressure induced-force due to inner/outer pressure difference 
        
        private IBitRock bitRockModel;  // Bit-rock interaction model      
        private BitInternalForces bitInternalForces; // Class to input bit internal forces
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
        private double elasticForceX;
        private double elasticForceY;
        private double elasticForceZ;
        private double elasticForcePhi;
        

        private double PreStressForceX;
        private double PreStressForceY;        
        // Fluid force-related variables
        private double fluidForceX;
        private double fluidForceY;
        // Unbalance force-related variables
        private double unbalanceForceX;
        private double unbalanceForceY;   
        // Friction related variables
        private double coulombFrictionX;
        private double coulombFrictionY;                    
        private double coulombFrictionZ;    
        
        /// <summary>
        /// Initializes a new instance of <see cref="LumpedElementModel"/> and assembles the
        /// lumped-parameter mass, damping, and stiffness arrays for all degrees of freedom
        /// (axial, lateral, and torsional) from the provided simulation parameters.
        /// </summary>
        /// <param name="parameters">
        /// Simulation parameters containing drill-string geometry, material properties,
        /// flow data, and solver settings.
        /// </param>
        /// <param name="bitRockModel">
        /// The bit-rock interaction model used to compute WOB and TOB at the drill bit node.
        /// </param>
        public LumpedElementModel(in SimulationParameters parameters,
            in IBitRock bitRockModel)
        {
            // Initialize the bit-rock model
            this.bitRockModel = bitRockModel;
            bitInternalForces = new BitInternalForces();
            // Create a simplifiefd drill-string form the input
            numberOfNodes = parameters.NumberOfNodes;
            #region Mass and stiffness matrices
            // In here, the global and element mass and stiffness matrices are created
            // After I populate the matrices, I want to split into 3 vector:
            // left of diagonal, diagonal and right of diagonal for all DoF.
            lateralStiffnessLeft = new double[parameters.NumberOfElements + 1];
            lateralStiffnessMid = new double[parameters.NumberOfElements + 1];
            lateralStiffnessRight = new double[parameters.NumberOfElements + 1];          
            
            lateralTensionInducedStiffnessMid = new double[parameters.NumberOfElements + 1];
            lateralTensionInducedStiffnessLeft = new double[parameters.NumberOfElements + 1];
            lateralTensionInducedStiffnessRight = new double[parameters.NumberOfElements + 1];
            
            lateralPressureInducedStiffnessMid = new double[parameters.NumberOfElements + 1];
            lateralPressureInducedStiffnessLeft = new double[parameters.NumberOfElements + 1];
            lateralPressureInducedStiffnessRight = new double[parameters.NumberOfElements + 1];

            lateralStiffnessLeft[0] = 0.0;
            lateralStiffnessRight[parameters.NumberOfElements] = 0.0;

            lateralTensionInducedStiffnessLeft[0] = 0.0;
            lateralTensionInducedStiffnessLeft[parameters.NumberOfElements] = 0.0;

            axialStiffnessLeft = new double[parameters.NumberOfElements + 1];
            axialStiffnessMid = new double[parameters.NumberOfElements + 1];
            axialStiffnessRight = new double[parameters.NumberOfElements + 1];
            
            torsionalStiffnessLeft = new double[parameters.NumberOfElements + 1];
            torsionalStiffnessMid = new double[parameters.NumberOfElements + 1];
            torsionalStiffnessRight = new double[parameters.NumberOfElements + 1];

            axialLumpedMass = new double[parameters.NumberOfElements + 1];
            lateralLumpedMass = new double[parameters.NumberOfElements + 1];
            torsionalLumpedInertia = new double[parameters.NumberOfElements + 1];
            addedLateralFluidMass =  new double[parameters.NumberOfElements + 1];

            axialForceDistribution = new double[parameters.NumberOfElements + 1];
            nodeEccentricity = new double[parameters.NumberOfElements + 1];
            secondMomentOfArea = new double[parameters.NumberOfElements + 1];


            massProportionalDampingAxial = new double[parameters.NumberOfElements + 1];
            massProportionalDampingLateral = new double[parameters.NumberOfElements + 1];
            inertiaProportionalDampingTorsional = new double[parameters.NumberOfElements + 1];

            // CHANGE TO SPARSE MATRICES??            
            //lateralStiffnessMatrix = Matrix<double>.Build.Dense(parameters.NumberOfElements + 1, parameters.NumberOfElements + 1);
            //axialStiffnessMatrix = Matrix<double>.Build.Dense(parameters.NumberOfElements+ 1, parameters.NumberOfElements + 1);
            //torsionalStiffnessMatrix = Matrix<double>.Build.Dense(parameters.NumberOfElements + 1, parameters.NumberOfElements + 1);
            
            for (int i = 0; i < parameters.NumberOfElements; i++)
            {
                //  The equivalent mass for each DoF is dependent on the assumed mode shape. Axial and Torsional modes use a linear element, which is
                //compatible to a rigid body motion. The calculation can be foundEach element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx. 
                // The lateral modes use a half-sine shape, which is compatible with a simply supported beam with C1 continuity. When the mass element is lumped, it falls
                // back to the conventional 0.5 * rho * A * L for each node.
                //  It needs to use element moment of area, so it preserves the different  
                //                      ___________________________
                // --------------------|                           |--------------------
                //       1             |             2             |          3
                // --------------------|___________________________|--------------------
                //
                // Inertia1(Radius1)   |      Inertia2(Radius2)    |   Inertia3(Radius3)    
                //   Mass1(Radius1)    |        Mass2(Radius2)     |     Mass3(Radius3)
                //

                // ============================================= Inertia / Mass terms =========================================
                //Pre-calculations
                double axialMass = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementArea[i] * parameters.Drillstring.ElementLength[i] / 2.0;
                double torsionalInertia = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementPolarInertia[i] * parameters.Drillstring.ElementLength[i] / 2.0;
                double lateralMass = parameters.Drillstring.ElementDensity[i] * parameters.Drillstring.ElementArea[i] * parameters.Drillstring.ElementLength[i] / 2.0;                
                double lateralAxialInducedStiffness = Math.PI * Math.PI / (2.0 * parameters.Drillstring.ElementLength[i]);
                double poissonRatio = parameters.Drillstring.ElementYoungModuli[i] / (2.0 * parameters.Drillstring.ElementShearModuli[i]) - 1.0;
                double lateralExternalPressureStiffness = (4.0 - 8.0 * poissonRatio) 
                                                    * parameters.Drillstring.ElementOuterArea[i] 
                                                    * (parameters.Flow.AnnulusPressure[i] - parameters.Flow.HydrostaticAnnulusPressure[i]);
                double lateralInternalPressureStiffness =  (4.0 - 8.0 * poissonRatio) 
                                                    * parameters.Drillstring.ElementInnerArea[i] 
                                                    * (parameters.Flow.StringPressure[i] - parameters.Flow.HydrostaticStringPressure[i]);
                double addedFluidMass = 0.5 * ((i == 0) ? parameters.Drillstring.ElementFluidAddedMass[0] : parameters.Drillstring.ElementFluidAddedMass[i - 1]); 
                double eccentricity = 0.5 * ((i == 0) ? parameters.Drillstring.ElementEccentricity[0] * parameters.Drillstring.ElementEccentricMass[0] : 
                                                 parameters.Drillstring.ElementEccentricity[i - 1] * parameters.Drillstring.ElementEccentricMass[i - 1]);
                //Node i
                axialLumpedMass[i] += axialMass; 
                lateralLumpedMass[i] += lateralMass;
                torsionalLumpedInertia[i] += torsionalInertia;
                addedLateralFluidMass[i] += addedFluidMass;
                nodeEccentricity[i] += eccentricity;
                secondMomentOfArea[i] += 0.5 * parameters.Drillstring.ElementPolarInertia[i];
                //Node i+1
                axialLumpedMass[i + 1] += axialMass; 
                lateralLumpedMass[i + 1] += lateralMass;
                torsionalLumpedInertia[i + 1] += torsionalInertia;
                addedLateralFluidMass[i + 1] += addedFluidMass;                
                nodeEccentricity[i + 1] += eccentricity;
                secondMomentOfArea[i + 1] += 0.5 * parameters.Drillstring.ElementPolarInertia[i];                 
                // ========================================================== Damping Terms ==============================================================
                massProportionalDampingAxial[i] += parameters.Drillstring.AxialDampingFactor * axialMass;
                massProportionalDampingAxial[i + 1] += parameters.Drillstring.AxialDampingFactor * axialMass;
                inertiaProportionalDampingTorsional[i] += parameters.Drillstring.TorsionalDampingFactor * torsionalInertia;
                inertiaProportionalDampingTorsional[i + 1] += parameters.Drillstring.TorsionalDampingFactor * torsionalInertia;
                massProportionalDampingLateral[i] += parameters.Drillstring.LateralDampingFactor * lateralMass;
                massProportionalDampingLateral[i + 1] += parameters.Drillstring.LateralDampingFactor * lateralMass;                                
                // ========================================================= Stiffness Terms =============================================================
                //Pre-calculations
                double axialStiffness = parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementArea[i] / parameters.Drillstring.ElementLength[i];              
                double lateralStiffness = Math.PI * Math.PI * Math.PI * parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementSecondMomentOfArea[i] 
                            / parameters.Drillstring.ElementLength[i];
                // The torsional stiffness is calculated as G * J / L, where G is the shear modulus,                 
                // where J = 2I is the area moment of inertia. Each element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx 
                double torsionalStiffness = parameters.Drillstring.ElementShearModuli[i] * parameters.Drillstring.ElementPolarInertia[i] / parameters.Drillstring.ElementLength[i];;         
                //   Add the local elements to the global 
                // element matrix.
                // ---------- Lateral Stiffness -----------  
                lateralTensionInducedStiffnessMid[i] += lateralAxialInducedStiffness;
                lateralTensionInducedStiffnessMid[i + 1] += lateralAxialInducedStiffness;
                lateralTensionInducedStiffnessLeft[i]  = - lateralAxialInducedStiffness;
                lateralTensionInducedStiffnessRight[i] = - lateralAxialInducedStiffness;
                // Flow pressure differential terms
                lateralPressureInducedStiffnessMid[i] += (lateralExternalPressureStiffness - lateralInternalPressureStiffness) * lateralAxialInducedStiffness;
                lateralPressureInducedStiffnessMid[i + 1] += (lateralExternalPressureStiffness - lateralInternalPressureStiffness) * lateralAxialInducedStiffness;
                lateralPressureInducedStiffnessLeft[i]  = - (lateralExternalPressureStiffness - lateralInternalPressureStiffness) * lateralAxialInducedStiffness;
                lateralPressureInducedStiffnessRight[i] = - (lateralExternalPressureStiffness - lateralInternalPressureStiffness) * lateralAxialInducedStiffness;
                //Allocate vectors
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
            }
            //  Keeping the matricial notation of the mass and stiffness is not required.
            // However, it is very convenient to have them as it can be used for modal analysis,
            // to calculate natural frequencies, mode shapes, frequency domain responses and so forth.                      
            #endregion
            // Allocate main variables for the model.
            //NumberOfElements = simulationParameters.NumberOfElements;
            preStressNormalForce = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
            preStressBinormalForce = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
      
            toolFaceAngle = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
            tensionIntegral = Vector<double>.Build.Dense(parameters.NumberOfElements);
            tension = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
            torque = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
            
        }
        /// <summary>
        /// Computes the bending moments at each interior node using the half-sine mode-shape
        /// hypothesis: <c>M = E·I·(d²u/dz²)</c>, where the curvature is approximated as
        /// <c>d²u/dz² = -u·π²/L²</c>.
        /// </summary>
        /// <param name="state">Current simulation state; <see cref="State.BendingMomentX"/> and
        /// <see cref="State.BendingMomentY"/> are updated in-place.</param>
        /// <param name="simulationParameters">Simulation parameters providing element lengths,
        /// Young's moduli, and second moments of area.</param>
        public void UpdateBendingMoments(State state, SimulationParameters simulationParameters)
        {
            //double XiMinus1, YiMinus1, XiPlus1, YiPlus1;
            double invElementLengthSquared;
            double momentX, momentY;
            double d2Xdz2, d2Ydz2; 
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                invElementLengthSquared = 1.0 / (simulationParameters.Drillstring.ElementLength[i] * simulationParameters.Drillstring.ElementLength[i]);

                //XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                //YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                //XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                //YiPlus1 = (i == state.YDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Calculate the derivatives using the half-sine first mode hypothesis
                d2Xdz2 = - state.XDisplacement[i] * Math.PI * Math.PI * invElementLengthSquared; 
                d2Ydz2 = - state.YDisplacement[i] * Math.PI * Math.PI * invElementLengthSquared; 
                //Calcualte the bending moments using the central difference scheme
                momentX = simulationParameters.Drillstring.ElementYoungModuli[i]
                         * simulationParameters.Drillstring.ElementPolarInertia[i]
                         * d2Xdz2; // Bending moment x-component
                momentY = simulationParameters.Drillstring.ElementYoungModuli[i]
                         * simulationParameters.Drillstring.ElementPolarInertia[i]
                         * d2Ydz2; //
                
                state.BendingMomentX[i] =  momentX;
                state.BendingMomentY[i] =  momentY;
            }
        }
        /// <summary>
        /// Pre-computes quantities that must be updated once per time step before the main
        /// integration loop, including:
        /// <list type="bullet">
        ///   <item>Effective axial tension at each node (accounting for elastic forces, buoyancy,
        ///         pressure differentials, and Poisson effects).</item>
        ///   <item>Elastic torque distribution.</item>
        ///   <item>Pre-stress normal and binormal contact forces derived from the Frenet-Serret
        ///         frame along the wellbore trajectory.</item>
        ///   <item>Tool-face angles at each node.</item>
        /// </list>
        /// </summary>
        /// <param name="state">Current simulation state (read-only for displacements; updated for
        /// pre-stress and tool-face data).</param>
        /// <param name="parameters">Simulation parameters providing trajectory, flow, and
        /// drill-string properties.</param>
        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
        
            double axialForce;

            double Tension;
            double tensionIntegralTemp = 0;
            double innerArea;
            double outerArea;
            bool isFirst;
            int revIdx;
            for (int i = 0; i < parameters.NumberOfElements; i++)
            {
                // Index for reverse loop
                revIdx = parameters.NumberOfElements - i;
                tensionIntegralTemp += 0.5*(parameters.Flow.dSigmaDx[revIdx] + parameters.Flow.dSigmaDx[revIdx - 1])/ parameters.Drillstring.ElementLength[i];
                tensionIntegral[i] = tensionIntegralTemp;
            }
            // Loop to compute the tensions and the stiffness of the model
            // NOT BEING TOTALLY USED RIGHT NOW
            for (int i = 0; i < parameters.NumberOfElements + 1; i++)
            {
                isFirst = i == 0;
                int offPhaseIndex = isFirst ? 0 : i - 1;
                // Calculate the axial elastic force: E * A 
                axialForce = CalculateAxialElasticForce(i, state) ;               
                // Get the current property and repeat the fist as a boundary condition
                innerArea = parameters.Drillstring.ElementInnerArea[offPhaseIndex];
                outerArea = parameters.Drillstring.ElementOuterArea[offPhaseIndex];
                //localBendingStiffness = isFirst ? lateralStiffnessLeft[0] : lateralStiffnessMid[i-1];
                // Index for reverse loop
                revIdx = i == parameters.NumberOfElements ? 0 : parameters.NumberOfElements - i - 1;                
                Tension = tensionIntegral[revIdx] + parameters.Flow.AxialBuoyancyForceChangeOfDiameters[i] - axialForce;
                Tension += (1 - 2 * parameters.Drillstring.PoissonRatio) * 
                            (
                                outerArea * (parameters.Flow.AnnulusPressure[i] - parameters.Flow.HydrostaticAnnulusPressure[i])
                              - innerArea * (parameters.Flow.StringPressure[i] - parameters.Flow.HydrostaticStringPressure[i])
                            );
                // Update the model
                tension[i] = Tension;
                //This step checks if there is a negative element and sets to zero
                //bendingStiffness[i] = - Math.Max(localBendingStiffness - Math.Pow(Math.PI, 2) * Tension / (parameters.Drillstring.ElementLength[i]), 0.0);            
                torque[i] = CalculateTorsionalElasticForce(i, state);
                //Roll over for integration
                /*
                oldDifferencialNormalForceSoftString = differentialNormalForceSoftString;*/
            }
            //bendingStiffness[0] = bendingStiffness[1];
            //torque[0] = torque[1];
            //tension[0] = tension[1];
            //Allocate variables for the loop. For synchronous computing
            double normalForce;
            double binormalForce;
            double oldNormalForce = 0;
            double oldBinormalForce = 0;
            double elasticTorqueDifferential;
            double InertiaTimesYoungModulus;
            double hVectorProductNormalDorProductTangent;
            double signToolFace;
            double dotProduct;  
            // Loop to compute the pre-stresses
            for (int i = 0; i < parameters.NumberOfElements + 1; i++)
            {     
                isFirst = i == 0;
                // Normal force components in Frenet-Serret coordinate system
                elasticTorqueDifferential = isFirst ?  0 : (torque[i] - torque[i-1]) / parameters.Drillstring.ElementLength[i - 1];
                InertiaTimesYoungModulus = isFirst ? 
                                parameters.Drillstring.ElementYoungModuli[0] * parameters.Drillstring.ElementPolarInertia[0] : 
                                parameters.Drillstring.ElementYoungModuli[i - 1] * parameters.Drillstring.ElementPolarInertia[i - 1];
                binormalForce = parameters.Flow.BuoyantWeightPerLength[i] * parameters.Trajectory.bz[i] 
                                + parameters.Trajectory.Curvature[i] * elasticTorqueDifferential 
                                + parameters.Trajectory.CurvatureDerivative[i] * torque[i] 
                                - 2 * InertiaTimesYoungModulus * parameters.Trajectory.CurvatureDerivative[i] * parameters.Trajectory.Torsion[i] 
                                - InertiaTimesYoungModulus * parameters.Trajectory.Curvature[i] * parameters.Trajectory.TorsionDerivative[i];
                normalForce = parameters.Trajectory.Curvature[i] * 
                                (
                                    tension[i] 
                                    + parameters.Flow.NormalBuoyancyForceChangeOfDiameters[i] 
                                    - parameters.Trajectory.Torsion[i] * torque[i]
                                ) 
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
                preStressNormalForce[i] = isFirst ? 0.0 : 0.5 * (oldNormalForce + normalForce) * parameters.Drillstring.ElementLength[i - 1];
                preStressBinormalForce[i] = isFirst ? 0.0 : 0.5 * (oldBinormalForce + binormalForce) * parameters.Drillstring.ElementLength[i - 1];
                //Update normal and binormal values from the 
                oldNormalForce = normalForce;
                oldBinormalForce = binormalForce;     
            }                         
        }
        /// <summary>
        /// Returns the net axial elastic force at node <paramref name="i"/> using the
        /// tri-diagonal stiffness representation:
        /// <c>F_z = -(K_left·z_{i-1} + K_mid·z_i + K_right·z_{i+1})</c>.
        /// Boundary conditions: the top-drive relative axial position is used at node 0,
        /// and zero displacement is imposed at the last node (bit on-bottom constraint).
        /// </summary>
        /// <param name="i">Node index (0 … numberOfNodes−1).</param>
        /// <param name="state">Current simulation state providing axial displacements and
        /// top-drive position.</param>
        /// <returns>Net axial elastic force [N] at node <paramref name="i"/>.</returns>
        private double CalculateAxialElasticForce(int i, State state)
        {
            double ZiMinus1 = (i == 0) ? state.TopDrive.RelativeAxialPosition : state.ZDisplacement[i - 1];
            double ZiPlus1 = (i == numberOfNodes - 1) ? 0.0 : state.ZDisplacement[i + 1];            
            return - ( axialStiffnessLeft[i] * ZiMinus1  + axialStiffnessMid[i] * state.ZDisplacement[i] + axialStiffnessRight[i] * ZiPlus1 );
        }
        /// <summary>
        /// Returns the net torsional elastic force (restoring torque) at node <paramref name="i"/>
        /// using the tri-diagonal stiffness representation:
        /// <c>T = -(K_left·φ_{i-1} + K_mid·φ_i + K_right·φ_{i+1})</c>.
        /// Zero angular displacement boundary conditions are applied at the first and last nodes.
        /// </summary>
        /// <param name="i">Node index (0 … numberOfNodes−1).</param>
        /// <param name="state">Current simulation state providing angular displacements.</param>
        /// <returns>Net torsional elastic torque [N·m] at node <paramref name="i"/>.</returns>
        private double CalculateTorsionalElasticForce(int i, State state)
        {
            //There is no element at i = -1. StiffnessLeft should aready be at 0.
            double PhiMinus1 = (i == 0) ? 0.0 : state.AngularDisplacement[i - 1];
            //There is no element at i = numberOfNodes. StiffnessRight should aready be at 0.
            double PhiPlus1 = (i == numberOfNodes - 1) ? 0.0 : state.AngularDisplacement[i + 1];                                                
            return - ( torsionalStiffnessLeft[i] * PhiMinus1  + torsionalStiffnessMid[i] * state.AngularDisplacement[i] + torsionalStiffnessRight[i] * PhiPlus1 );
        }
        /// <summary>
        /// Populates <see cref="axialForceDistribution"/> with the axial elastic force at
        /// every node. This snapshot is consumed by the lateral elastic force calculation
        /// to apply tension-induced (geometric) stiffness corrections.
        /// </summary>
        /// <param name="state">Current simulation state providing axial displacements.</param>
        private void CalculateAxialForceAxialDistribution(State state)
        {
            for (int i = 0; i < numberOfNodes; i++)
            {
                axialForceDistribution[i] = CalculateAxialElasticForce(i, state);
            }            
        }
        /// <summary>
        /// Clamps a lateral stiffness coefficient to prevent negative (buckling-induced) values
        /// from coupling adjacent nodes. When the drill-string buckles, the off-diagonal coupling
        /// terms are zeroed (string model behaviour).
        /// </summary>
        /// <param name="stiffness">Raw stiffness coefficient before clamping.</param>
        /// <param name="centerElement">
        /// <see langword="true"/> for the diagonal (self) term — clamped to ≥ 0;
        /// <see langword="false"/> for the off-diagonal (coupling) terms — clamped to ≤ 0.
        /// </param>
        /// <returns>Clamped stiffness coefficient.</returns>
        private double NegativeStiffnessTreatment(double stiffness, bool centerElement)
        {
            if (centerElement)
            {
                //bendingStiffness[i] = - Math.Max(localBendingStiffness - Math.Pow(Math.PI, 2) * Tension / (parameters.Drillstring.ElementLength[i]), 0.0);   
                return Math.Max(stiffness, 0);
            }
            else
            {
                return Math.Min(stiffness, 0);
            }
        }
        /// <summary>
        /// Main per-step force and acceleration computation. For every node in the drill-string
        /// the method:
        /// <list type="number">
        ///   <item>Converts lateral displacements to polar coordinates (radial displacement,
        ///         whirl angle, radial and whirl velocities).</item>
        ///   <item>Evaluates wellbore-wall collision (normal contact force).</item>
        ///   <item>Computes axial and torsional elastic forces.</item>
        ///   <item>Calls the bit-rock model at the bottommost node to obtain WOB and TOB.</item>
        ///   <item>Computes tension-corrected lateral elastic forces.</item>
        ///   <item>Resolves pre-stress forces into the Cartesian frame using the tool-face
        ///         angle.</item>
        ///   <item>Computes added-fluid inertia and Coriolis/centripetal fluid forces.</item>
        ///   <item>Computes mass-unbalance (eccentricity) forces.</item>
        ///   <item>Applies Coulomb / static friction at the contact point (stick-slip
        ///         detection).</item>
        ///   <item>Assembles net forces and torques and divides by the respective lumped
        ///         masses/inertias to produce nodal accelerations.</item>
        ///   <item>Writes all computed quantities back to <paramref name="state"/>.</item>
        /// </list>
        /// </summary>
        /// <param name="state">
        /// Current simulation state. Velocities and displacements are read; acceleration arrays
        /// and diagnostic fields are written.
        /// </param>
        /// <param name="parameters">
        /// Simulation parameters (drill-string, flow, wellbore, friction, mud-motor, etc.).
        /// </param>
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
                    state.MudTorque = - parameters.MudMotor.P0_motor * parameters.MudMotor.V_motor * (speedRatio - 1);                
            }
            #endregion
            //Update axial elastic forces beforehand - Needed for the equivalent stiffness
            CalculateAxialForceAxialDistribution(state);
            // Topdrive Torque only applies to the first element
            topDriveTorque = (
                                state.TopDrive.TopDriveMotorTorque 
                                - torsionalStiffnessMid[0] * (state.TopDrive.AngularDisplacement - state.AngularDisplacement[0]) 
                            );
            state.TopDrive.AngularAcceleration = topDriveTorque / parameters.TopDriveDrawwork.TopDriveInertia;
            // Apply boundary conditions from the top-drive
            //state.AngularVelocity[0] = state.TopDrive.AngularVelocity;
            //state.AngularDisplacement[0] = state.TopDrive.AngularDisplacement;            
            //state.ZDisplacement[0] = state.TopDrive.AxialPosition;
            //state.ZVelocity[0] = state.TopDrive.AxialVelocity;                
            // =============================== Main loop for calculating forces and accelerations in the lateral model ===============================
            for (int i = 0; i < numberOfNodes; i++)
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
                #region Axial-Torsional Elastic Forces Calculation
                //  Calculated the torque and force difference in each element (TorqueElement and TorqueNextElement) using the strains 
                //obtained from the axial and the torsional models. As they heve different indexes, they need to be re-aligned with the
                //lateral elements.
                //If it is the last element, use the torque on bit
                // ------------------ Axial Elastic Forces Calculation --------------------       
                elasticForceZ = axialForceDistribution[i];          
                // ------------------ Torsional Elastic Forces Calculation --------------------
                elasticForcePhi = CalculateTorsionalElasticForce(i, state);
                #endregion 
                #region Bit-Rock Interaction
                if (i == parameters.NumberOfNodes - 1)
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
                #region Elastic Forces Calculation                
                // ------------------ Lateral Elastic Forces Calculation ------------------
                // If it is the first element, get the pinned boundary condition
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                // Same with the stiffness
                //Calculate equivalent stiffnessess 
                double FaMinus1 = (i == 0) ? 0.0 : axialForceDistribution[i - 1];
                // Does the weight on bit goes here: double FaPlus1 = (i == state.XDisplacement.Count - 1) ? state.WeightOnBit : axialForceDistribution[i + 1];
                double FaPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : axialForceDistribution[i + 1];
                //  Corrects the lateral stiffness with information from the pressure gradient and from the axial force distribution
                //The NegativeStiffnessTreatment function breakes the connection between elements in case of buckling, as in a string model
                double equivalentLeftStiffness = NegativeStiffnessTreatment(lateralStiffnessLeft[i] + FaMinus1 * lateralTensionInducedStiffnessLeft[i] + lateralPressureInducedStiffnessLeft[i], centerElement: false);
                double equivalentMidStiffness = NegativeStiffnessTreatment(lateralStiffnessMid[i] + axialForceDistribution[i] * lateralTensionInducedStiffnessMid[i] + lateralPressureInducedStiffnessMid[i], centerElement: true);                
                double equivalentRightStiffness = NegativeStiffnessTreatment(lateralStiffnessRight[i] + FaPlus1 * lateralTensionInducedStiffnessRight[i] + lateralPressureInducedStiffnessRight[i], centerElement: false);
                elasticForceX = - ( equivalentLeftStiffness * XiMinus1  + equivalentMidStiffness * state.XDisplacement[i] + equivalentRightStiffness * XiPlus1 );
                elasticForceY = - ( equivalentLeftStiffness * YiMinus1  + equivalentMidStiffness * state.YDisplacement[i] + equivalentRightStiffness * YiPlus1 );
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
                fluidForceX =  addedLateralFluidMass[i] * 
                                (
                                    + 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                    - rotationSpeed * state.YVelocity[i]
                                )
                                - parameters.Flow.FluidDampingCoefficient * ( state.XVelocity[i] + 0.5 * rotationSpeed * state.YDisplacement[i] )
                                    
                                ;
                fluidForceY =  addedLateralFluidMass[i] * 
                                (
                                    + 0.25 * rotationSpeedSquared * state.YDisplacement[i]
                                    + rotationSpeed * state.XVelocity[i]
                                )
                                - parameters.Flow.FluidDampingCoefficient * (state.YVelocity[i] - 0.5 * rotationSpeed * state.XDisplacement[i])
                                ;
                #endregion
                // Sleeve braking force
                sleeveBrakeForce = hasSleeve ? parameters.Drillstring.SleeveTorsionalDamping[sleeveIndex] * (rotationSpeed - state.SleeveAngularVelocity[sleeveIndex]) : 0;
                #region Unbalance Forces
                // lateral forces due to mass imbalance
                // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center, 
                // which causes a lateral force and a torque as the pipe is displaced                
                rotationAngle = hasSleeve ? state.SleeveAngularDisplacement[sleeveIndex] : state.AngularDisplacement[i];                                        
                rotationAcceleration = hasSleeve ? state.SleeveAngularAcceleration[sleeveIndex] : state.AngularAcceleration[i]; 
                // The unbalance uses the pipe properties, not the sleeve. This is because the pipe is still rotating withing the sleeve.                                       
                unbalanceForceX = nodeEccentricity[i] *
                    (
                        state.AngularVelocity[i] * state.AngularVelocity[i] * Math.Cos(state.AngularDisplacement[i])
                        + state.AngularAcceleration[i] * Math.Sin(state.AngularDisplacement[i])
                    );
                unbalanceForceY = nodeEccentricity[i] * 
                    (
                        state.AngularVelocity[i] * state.AngularVelocity[i] * Math.Sin(state.AngularDisplacement[i])
                        - state.AngularAcceleration[i] * Math.Cos(state.AngularDisplacement[i])
                    );                                                            
                #endregion                                        
                #region Coulomb Friction
                // Axial velocity
                axialVelocity = state.ZVelocity[i];
                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.NodeOuterRadius[i];
                // The total normal force is the sum of elastic, pre-stress, fluid, unbalance forces, damping and colission forces                        
                sumForcesX = elasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - massProportionalDampingLateral[i] * state.XVelocity[i] - normalCollisionForce * cosWhirlAngle;
                sumForcesY = elasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - massProportionalDampingLateral[i] * state.YVelocity[i] - normalCollisionForce * sinWhirlAngle;
                sumForcesZ = elasticForceZ - massProportionalDampingAxial[i] * state.ZVelocity[i]; 
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
          
                    //Update relevant forces and torque with the calculated coulomb friction force
                    frictionTorque = hasSleeve ? sleeveBrakeForce * outerRadius : (outerRadius * (coulombFrictionX * sinWhirlAngle - coulombFrictionY * cosWhirlAngle));
                    if (hasSleeve)
                    {
                        // It needs to be zeroed before, as there might have changes in the sleeve position when the number of lumped parameters change 
                        state.SleeveForces[i] = 0;                
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
                    if (hasSleeve)
                    {
                        state.SleeveForces[i] = 0;
                    }
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
                // The torque on bit only apply on the last element
                torqueOnBit = (i == parameters.NumberOfElements) ? (   parameters.UseMudMotor ? state.MudTorque : state.TorqueOnBit   )    :   0.0   ;
                double boundaryTorque = (i == 0) ? 
                    + torsionalStiffnessMid[0] * (state.TopDrive.AngularDisplacement - state.AngularDisplacement[0]) : 0;
                // Sets a boundary-condition like fot the displacement                
                sumTorque =  elasticForcePhi + boundaryTorque + torqueOnBit - frictionTorque - inertiaProportionalDampingTorsional[i] * rotationSpeed;
                // Variables are generated locally to facilitate debugging only.
                angularAcceleration = sumTorque / torsionalLumpedInertia[i];      
                xAcceleration = sumForcesX / (lateralLumpedMass[i] + addedLateralFluidMass[i]);
                yAcceleration = sumForcesY / (lateralLumpedMass[i] + addedLateralFluidMass[i]);
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