using MathNet.Numerics.LinearAlgebra;
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
    ///
    ///     The axial and torsional degrees of freedom use first order finite elements on the
    /// axial-torsional grid, and the lateral degrees of freedom use the lateral grid. Every
    /// axial-torsional node is also a lateral node. Axial-torsional values are linearly interpolated
    /// to the lateral nodes, and the loads computed at the lateral nodes are returned to the
    /// axial-torsional nodes as consistent nodal loads.
    /// </summary>
    public class LumpedElementModel : IModel<LumpedElementModel>
    {
        /// <summary>
        /// Drill-string holding the two grids and the interpolation between them
        /// </summary>
        private SimulatorDrillString drillString;
        /// <summary>
        /// Number of lateral nodes
        /// </summary>
        private int numberOfLateralNodes;
        /// <summary>
        /// Number of axial-torsional nodes
        /// </summary>
        private int numberOfAxialNodes;

        #region Lateral grid properties
        /// <summary>
        /// Total tension at each lateral node, used for the pre-stress forces and the outputs.
        /// Computed in PrepareModel
        /// </summary>
        private Vector<double> tension;
        /// <summary>
        /// Torque at each lateral node, used for the pre-stress forces and the outputs.
        /// Computed in PrepareModel
        /// </summary>
        private Vector<double> torque;
        /// <summary>
        /// Derivative of the torque along the string at each lateral node.
        /// It is the slope of the linear torque field inside each axial-torsional element
        /// </summary>
        private Vector<double> torqueDerivative;
        /// <summary>
        /// Static part of the tension (buoyant weight, buoyancy and pressure terms) at each lateral node
        /// </summary>
        private Vector<double> staticTension;
        /// <summary>
        /// Total tension at each lateral node updated at every inner step, used for the
        /// tension-induced geometric stiffness
        /// </summary>
        private Vector<double> tensionAtLateralNodes;
        /// <summary>
        /// Torque at each lateral node updated at every inner step
        /// </summary>
        private Vector<double> torqueAtLateralNodes;
        /// <summary>
        /// Rayleigh proportional damping with mass-only coefficient
        /// for the lateral direction
        /// </summary>
        private double[] massProportionalDampingLateral;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the lateral stiffness term left to the diagonal
        /// </summary>
        private double[] lateralStiffnessLeft;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the lateral stiffness term on the diagonal
        /// </summary>
        private double[] lateralStiffnessMid;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the lateral stiffness term right to the diagonal
        /// </summary>
        private double[] lateralStiffnessRight;
        /// <summary>
        /// Tension-induced geometric stiffness coefficient of each lateral element, π²/(2L).
        /// Multiplied by the element tension it gives the element geometric stiffness
        /// </summary>
        private double[] lateralGeometricStiffnessCoefficient;
        /// <summary>
        /// Pressure-induced geometric stiffness of each lateral element
        /// </summary>
        private double[] lateralPressureInducedStiffness;
        /// <summary>
        /// Lumped lateral mass; diagonal matrix represented as a vector.
        /// It is also the axial lumped mass on the lateral grid
        /// </summary>
        private double[] lateralLumpedMass;
        /// <summary>
        /// Node eccentricity (mass offset from geometric center);
        /// diagonal matrix represented as a vector
        /// </summary>
        private double[] nodeEccentricity;
        /// <summary>
        /// Added lateral fluid mass from Fritz model;
        /// diagonal matrix represented as a vector
        /// </summary>
        private double[] addedLateralFluidMass;
        /// <summary>
        /// Second moment of area at each node;
        /// diagonal matrix represented as a vector
        /// </summary>
        private double[] secondMomentOfArea;
        /// <summary>
        /// Pre-stress normal force, used for calculating pre-bending and contact forces.
        /// Computed in PrepareModel and consumed in CalculateAccelerations
        /// </summary>
        private Vector<double> preStressNormalForce;
        /// <summary>
        /// Pre-stress binormal force, used for calculating pre-bending and contact forces.
        /// Computed in PrepareModel and consumed in CalculateAccelerations
        /// </summary>
        private Vector<double> preStressBinormalForce;
        /// <summary>
        /// Integral of the tension; auxiliary vector preallocated for the tension
        /// integral computation in PrepareModel
        /// </summary>
        private Vector<double> tensionIntegral;
        /// <summary>
        /// Tool-face angle at each node
        /// </summary>
        private Vector<double> toolFaceAngle;
        /// <summary>
        /// Trial axial acceleration interpolated to the lateral nodes, used for the static friction check
        /// </summary>
        private double[] trialAxialAccelerationAtLateralNodes;
        /// <summary>
        /// Axial friction force at each lateral node
        /// </summary>
        private double[] axialFrictionAtLateralNodes;
        /// <summary>
        /// Friction (or sleeve braking) torque at each lateral node
        /// </summary>
        private double[] frictionTorqueAtLateralNodes;
        #endregion

        #region Axial-torsional grid properties
        /// <summary>
        /// Rayleigh proportional damping with mass-only coefficient
        /// for the axial direction
        /// </summary>
        private double[] massProportionalDampingAxial;
        /// <summary>
        /// Rayleigh proportional damping with inertia-only coefficient
        /// for the torsional direction
        /// </summary>
        private double[] inertiaProportionalDampingTorsional;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the axial stiffness term left to the diagonal
        /// </summary>
        private double[] axialStiffnessLeft;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the axial stiffness term on the diagonal
        /// </summary>
        private double[] axialStiffnessMid;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the axial stiffness term right to the diagonal
        /// </summary>
        private double[] axialStiffnessRight;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the torsional stiffness term left to the diagonal
        /// </summary>
        private double[] torsionalStiffnessLeft;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the torsional stiffness term on the diagonal
        /// </summary>
        private double[] torsionalStiffnessMid;
        /// <summary>
        /// Stiffness matrix is quasi-diagonal.
        /// This is the torsional stiffness term right to the diagonal
        /// </summary>
        private double[] torsionalStiffnessRight;
        /// <summary>
        /// Axial stiffness EA/L of each axial-torsional element
        /// </summary>
        private double[] axialElementStiffness;
        /// <summary>
        /// Torsional stiffness GJ/L of each axial-torsional element
        /// </summary>
        private double[] torsionalElementStiffness;
        /// <summary>
        /// Lumped axial mass; diagonal matrix represented as a vector
        /// </summary>
        private double[] axialLumpedMass;
        /// <summary>
        /// Lumped torsional inertia; diagonal matrix represented as a vector
        /// </summary>
        private double[] torsionalLumpedInertia;
        /// <summary>
        /// Net axial elastic force at each axial-torsional node
        /// </summary>
        private double[] axialElasticForce;
        /// <summary>
        /// Net torsional elastic torque at each axial-torsional node
        /// </summary>
        private double[] torsionalElasticTorque;
        /// <summary>
        /// Elastic tension at each axial-torsional node, averaged from the adjacent elements
        /// </summary>
        private double[] nodalElasticTension;
        /// <summary>
        /// Torque at each axial-torsional node, averaged from the adjacent elements
        /// </summary>
        private double[] nodalTorque;
        /// <summary>
        /// Axial acceleration from the elastic and damping forces only, before friction
        /// </summary>
        private double[] trialAxialAcceleration;
        /// <summary>
        /// Consistent nodal load of the axial friction forces
        /// </summary>
        private double[] axialFrictionNodal;
        /// <summary>
        /// Consistent nodal load of the friction torques
        /// </summary>
        private double[] frictionTorqueNodal;
        #endregion

        /// <summary>
        /// Bit-rock interaction model
        /// </summary>
        private IBitRock bitRockModel;
        /// <summary>
        /// Class used to pass internal bit forces to the bit-rock interaction model
        /// </summary>
        private BitInternalForces bitInternalForces;

        /// <summary>
        /// Initializes a new instance of <see cref="LumpedElementModel"/> and assembles the
        /// mass, damping, and stiffness arrays for the lateral degrees of freedom on the lateral grid
        /// and for the axial and torsional degrees of freedom on the axial-torsional grid.
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
            drillString = parameters.Drillstring;
            numberOfLateralNodes = parameters.NumberOfNodes;
            numberOfAxialNodes = parameters.NumberOfAxialNodes;
            int numberOfLateralElements = parameters.NumberOfElements;
            int numberOfAxialElements = parameters.NumberOfAxialElements;
            #region Lateral mass and stiffness matrices
            // In here, the global and element mass and stiffness matrices are created
            // After I populate the matrices, I want to split into 3 vector:
            // left of diagonal, diagonal and right of diagonal for all DoF.
            lateralStiffnessLeft = new double[numberOfLateralNodes];
            lateralStiffnessMid = new double[numberOfLateralNodes];
            lateralStiffnessRight = new double[numberOfLateralNodes];
            // Terms due to the tension and to the flow pressure differential, per element
            lateralGeometricStiffnessCoefficient = new double[numberOfLateralElements];
            lateralPressureInducedStiffness = new double[numberOfLateralElements];
            // Lateral mass
            lateralLumpedMass = new double[numberOfLateralNodes];
            addedLateralFluidMass =  new double[numberOfLateralNodes];
            // Unbalance
            nodeEccentricity = new double[numberOfLateralNodes];
            secondMomentOfArea = new double[numberOfLateralNodes];
            // Damping
            massProportionalDampingLateral = new double[numberOfLateralNodes];
            for (int i = 0; i < numberOfLateralElements; i++)
            {
                // The lateral modes use a half-sine shape, which is compatible with a simply supported beam with C1 continuity. When the mass element is lumped, it falls
                // back to the conventional 0.5 * rho * A * L for each node.
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
                // ---------- Lateral Mass -----------
                lateralLumpedMass[i] += lateralMass;
                lateralLumpedMass[i + 1] += lateralMass;
                addedLateralFluidMass[i] += addedFluidMass;
                addedLateralFluidMass[i + 1] += addedFluidMass;
                nodeEccentricity[i] += eccentricity;
                nodeEccentricity[i + 1] += eccentricity;
                secondMomentOfArea[i] += 0.5 * parameters.Drillstring.ElementPolarInertia[i];
                secondMomentOfArea[i + 1] += 0.5 * parameters.Drillstring.ElementPolarInertia[i];
                // ---------- Damping -----------
                massProportionalDampingLateral[i] += parameters.Drillstring.LateralDampingFactor * lateralMass;
                massProportionalDampingLateral[i + 1] += parameters.Drillstring.LateralDampingFactor * lateralMass;
                // ---------- Lateral Stiffness -----------
                double lateralStiffness = Math.PI * Math.PI * Math.PI * parameters.Drillstring.ElementYoungModuli[i] * parameters.Drillstring.ElementSecondMomentOfArea[i]
                            / parameters.Drillstring.ElementLength[i];
                lateralGeometricStiffnessCoefficient[i] = lateralAxialInducedStiffness;
                lateralPressureInducedStiffness[i] = (lateralExternalPressureStiffness - lateralInternalPressureStiffness) * lateralAxialInducedStiffness;
                lateralStiffnessLeft[i + 1] = - lateralStiffness;
                lateralStiffnessMid[i] += lateralStiffness;
                lateralStiffnessMid[i + 1] += lateralStiffness;
                lateralStiffnessRight[i] = - lateralStiffness;
            }
            #endregion
            #region Axial and torsional mass and stiffness matrices
            //  The equivalent mass for each DoF is dependent on the assumed mode shape. Axial and Torsional modes use a linear element, which is
            //compatible to a rigid body motion. The calculation can be foundEach element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx.
            axialStiffnessLeft = new double[numberOfAxialNodes];
            axialStiffnessMid = new double[numberOfAxialNodes];
            axialStiffnessRight = new double[numberOfAxialNodes];
            torsionalStiffnessLeft = new double[numberOfAxialNodes];
            torsionalStiffnessMid = new double[numberOfAxialNodes];
            torsionalStiffnessRight = new double[numberOfAxialNodes];
            axialElementStiffness = new double[numberOfAxialElements];
            torsionalElementStiffness = new double[numberOfAxialElements];
            axialLumpedMass = new double[numberOfAxialNodes];
            torsionalLumpedInertia = new double[numberOfAxialNodes];
            massProportionalDampingAxial = new double[numberOfAxialNodes];
            inertiaProportionalDampingTorsional = new double[numberOfAxialNodes];
            for (int e = 0; e < numberOfAxialElements; e++)
            {
                double length = parameters.Drillstring.AxialElementLength[e];
                double axialMass = parameters.Drillstring.AxialElementDensity[e] * parameters.Drillstring.AxialElementArea[e] * length / 2.0;
                double torsionalInertia = parameters.Drillstring.AxialElementDensity[e] * parameters.Drillstring.AxialElementPolarInertia[e] * length / 2.0;
                // ---------- Axial Mass -----------
                axialLumpedMass[e] += axialMass;
                axialLumpedMass[e + 1] += axialMass;
                // ---------- Torsional Mass -----------
                torsionalLumpedInertia[e] += torsionalInertia;
                torsionalLumpedInertia[e + 1] += torsionalInertia;
                // ---------- Damping -----------
                massProportionalDampingAxial[e] += parameters.Drillstring.AxialDampingFactor * axialMass;
                massProportionalDampingAxial[e + 1] += parameters.Drillstring.AxialDampingFactor * axialMass;
                inertiaProportionalDampingTorsional[e] += parameters.Drillstring.TorsionalDampingFactor * torsionalInertia;
                inertiaProportionalDampingTorsional[e + 1] += parameters.Drillstring.TorsionalDampingFactor * torsionalInertia;
                // ---------- Stiffness -----------
                double axialStiffness = parameters.Drillstring.AxialElementYoungModuli[e] * parameters.Drillstring.AxialElementArea[e] / length;
                // The torsional stiffness is calculated as G * J / L, where G is the shear modulus,
                // where J = 2I is the area moment of inertia. Each element matrix can be found in /AxuiliarDevFiles/LumpedParameterMatrices.wxmx
                double torsionalStiffness = parameters.Drillstring.AxialElementShearModuli[e] * parameters.Drillstring.AxialElementPolarInertia[e] / length;
                axialElementStiffness[e] = axialStiffness;
                torsionalElementStiffness[e] = torsionalStiffness;
                axialStiffnessLeft[e + 1] = - axialStiffness;
                axialStiffnessMid[e] += axialStiffness;
                axialStiffnessMid[e + 1] += axialStiffness;
                axialStiffnessRight[e] = - axialStiffness;
                torsionalStiffnessLeft[e + 1] = - torsionalStiffness;
                torsionalStiffnessMid[e] += torsionalStiffness;
                torsionalStiffnessMid[e + 1] += torsionalStiffness;
                torsionalStiffnessRight[e] = - torsionalStiffness;
            }
            #endregion
            // Allocate main variables for the model.
            preStressNormalForce = Vector<double>.Build.Dense(numberOfLateralNodes);
            preStressBinormalForce = Vector<double>.Build.Dense(numberOfLateralNodes);
            toolFaceAngle = Vector<double>.Build.Dense(numberOfLateralNodes);
            tensionIntegral = Vector<double>.Build.Dense(numberOfLateralElements);
            tension = Vector<double>.Build.Dense(numberOfLateralNodes);
            torque = Vector<double>.Build.Dense(numberOfLateralNodes);
            torqueDerivative = Vector<double>.Build.Dense(numberOfLateralNodes);
            staticTension = Vector<double>.Build.Dense(numberOfLateralNodes);
            tensionAtLateralNodes = Vector<double>.Build.Dense(numberOfLateralNodes);
            torqueAtLateralNodes = Vector<double>.Build.Dense(numberOfLateralNodes);
            trialAxialAccelerationAtLateralNodes = new double[numberOfLateralNodes];
            axialFrictionAtLateralNodes = new double[numberOfLateralNodes];
            frictionTorqueAtLateralNodes = new double[numberOfLateralNodes];
            axialElasticForce = new double[numberOfAxialNodes];
            torsionalElasticTorque = new double[numberOfAxialNodes];
            nodalElasticTension = new double[numberOfAxialNodes];
            nodalTorque = new double[numberOfAxialNodes];
            trialAxialAcceleration = new double[numberOfAxialNodes];
            axialFrictionNodal = new double[numberOfAxialNodes];
            frictionTorqueNodal = new double[numberOfAxialNodes];
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
            double invElementLengthSquared;
            double momentX, momentY;
            double d2Xdz2, d2Ydz2;
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                invElementLengthSquared = 1.0 / (simulationParameters.Drillstring.ElementLength[i] * simulationParameters.Drillstring.ElementLength[i]);
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
        ///   <item>Static axial tension at each lateral node (buoyancy, pressure differentials and Poisson effects).</item>
        ///   <item>Elastic tension and torque, recovered at the axial-torsional nodes and linearly
        ///         interpolated to the lateral nodes.</item>
        ///   <item>Pre-stress normal and binormal contact forces derived from the Frenet-Serret
        ///         frame along the wellbore trajectory.</item>
        ///   <item>Tool-face angles at each node.</item>
        /// </list>
        /// </summary>
        /// <param name="state">Current simulation state.</param>
        /// <param name="parameters">Simulation parameters providing trajectory, flow, and
        /// drill-string properties.</param>
        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
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
            // Static part of the tension at the lateral nodes
            for (int i = 0; i < numberOfLateralNodes; i++)
            {
                isFirst = i == 0;
                int offPhaseIndex = isFirst ? 0 : i - 1;
                // Get the current property and repeat the fist as a boundary condition
                innerArea = parameters.Drillstring.ElementInnerArea[offPhaseIndex];
                outerArea = parameters.Drillstring.ElementOuterArea[offPhaseIndex];
                // Index for reverse loop
                revIdx = i == parameters.NumberOfElements ? 0 : parameters.NumberOfElements - i - 1;
                staticTension[i] = tensionIntegral[revIdx] + parameters.Flow.AxialBuoyancyForceChangeOfDiameters[i];
                staticTension[i] += (1 - 2 * parameters.Drillstring.PoissonRatio) *
                            (
                                outerArea * (parameters.Flow.AnnulusPressure[i] - parameters.Flow.HydrostaticAnnulusPressure[i])
                              - innerArea * (parameters.Flow.StringPressure[i] - parameters.Flow.HydrostaticStringPressure[i])
                            );
            }
            // Elastic part of the tension and torque from the axial-torsional elements
            RecoverNodalElasticForces(state);
            for (int i = 0; i < numberOfLateralNodes; i++)
            {
                int e = drillString.LateralToAxialElement[i];
                tension[i] = staticTension[i] + drillString.InterpolateToLateral(nodalElasticTension, i);
                torque[i] = drillString.InterpolateToLateral(nodalTorque, i);
                // Slope of the linear torque field inside the axial-torsional element
                torqueDerivative[i] = (nodalTorque[e + 1] - nodalTorque[e]) / drillString.AxialElementLength[e];
            }
            //Allocate variables for the loop. For synchronous computing
            double normalForce;
            double binormalForce;
            double oldNormalForce = 0;
            double oldBinormalForce = 0;
            double InertiaTimesYoungModulus;
            double hVectorProductNormalDorProductTangent;
            double signToolFace;
            double dotProduct;
            // Loop to compute the pre-stresses
            for (int i = 0; i < numberOfLateralNodes; i++)
            {
                isFirst = i == 0;
                // Normal force components in Frenet-Serret coordinate system
                InertiaTimesYoungModulus = isFirst ?
                                parameters.Drillstring.ElementYoungModuli[0] * parameters.Drillstring.ElementPolarInertia[0] :
                                parameters.Drillstring.ElementYoungModuli[i - 1] * parameters.Drillstring.ElementPolarInertia[i - 1];
                binormalForce = parameters.Flow.BuoyantWeightPerLength[i] * parameters.Trajectory.bz[i]
                                + parameters.Trajectory.Curvature[i] * torqueDerivative[i]
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
        /// Recovers the elastic tension and torque at the axial-torsional nodes. With first order
        /// elements, they are constant inside each element: <c>T_e = EA/L·(z_{e+1} − z_e)</c> and
        /// <c>τ_e = GJ/L·(φ_e − φ_{e+1})</c>. Interior nodes take the average of the two adjacent
        /// elements, and the top and bit nodes take the value of their only element.
        /// </summary>
        /// <param name="state">Current simulation state providing axial and angular displacements.</param>
        private void RecoverNodalElasticForces(State state)
        {
            int numberOfAxialElements = numberOfAxialNodes - 1;
            double previousTension = 0;
            double previousTorque = 0;
            for (int e = 0; e < numberOfAxialElements; e++)
            {
                double elementTension = axialElementStiffness[e] * (state.ZDisplacement[e + 1] - state.ZDisplacement[e]);
                double elementTorque = torsionalElementStiffness[e] * (state.AngularDisplacement[e] - state.AngularDisplacement[e + 1]);
                if (e == 0)
                {
                    nodalElasticTension[0] = elementTension;
                    nodalTorque[0] = elementTorque;
                }
                else
                {
                    nodalElasticTension[e] = 0.5 * (previousTension + elementTension);
                    nodalTorque[e] = 0.5 * (previousTorque + elementTorque);
                }
                previousTension = elementTension;
                previousTorque = elementTorque;
            }
            nodalElasticTension[numberOfAxialElements] = previousTension;
            nodalTorque[numberOfAxialElements] = previousTorque;
        }
        /// <summary>
        /// Returns the net axial elastic force at axial-torsional node <paramref name="i"/> using the
        /// tri-diagonal stiffness representation:
        /// <c>F_z = -(K_left·z_{i-1} + K_mid·z_i + K_right·z_{i+1})</c>.
        /// Boundary conditions: the top-drive relative axial position is used at node 0,
        /// and zero displacement is imposed at the last node (bit on-bottom constraint).
        /// </summary>
        /// <param name="i">Axial-torsional node index (0 … numberOfAxialNodes−1).</param>
        /// <param name="state">Current simulation state providing axial displacements and
        /// top-drive position.</param>
        /// <returns>Net axial elastic force [N] at node <paramref name="i"/>.</returns>
        private double CalculateAxialElasticForce(int i, State state)
        {
            double ZiMinus1 = (i == 0) ? state.TopDrive.RelativeAxialPosition : state.ZDisplacement[i - 1];
            double ZiPlus1 = (i == numberOfAxialNodes - 1) ? 0.0 : state.ZDisplacement[i + 1];
            return - ( axialStiffnessLeft[i] * ZiMinus1  + axialStiffnessMid[i] * state.ZDisplacement[i] + axialStiffnessRight[i] * ZiPlus1 );
        }
        /// <summary>
        /// Returns the net torsional elastic force (restoring torque) at axial-torsional node <paramref name="i"/>
        /// using the tri-diagonal stiffness representation:
        /// <c>T = -(K_left·φ_{i-1} + K_mid·φ_i + K_right·φ_{i+1})</c>.
        /// Zero angular displacement boundary conditions are applied at the first and last nodes.
        /// </summary>
        /// <param name="i">Axial-torsional node index (0 … numberOfAxialNodes−1).</param>
        /// <param name="state">Current simulation state providing angular displacements.</param>
        /// <returns>Net torsional elastic torque [N·m] at node <paramref name="i"/>.</returns>
        private double CalculateTorsionalElasticForce(int i, State state)
        {
            //There is no element at i = -1. StiffnessLeft should aready be at 0.
            double PhiMinus1 = (i == 0) ? 0.0 : state.AngularDisplacement[i - 1];
            //There is no element at i = numberOfAxialNodes. StiffnessRight should aready be at 0.
            double PhiPlus1 = (i == numberOfAxialNodes - 1) ? 0.0 : state.AngularDisplacement[i + 1];
            return - ( torsionalStiffnessLeft[i] * PhiMinus1  + torsionalStiffnessMid[i] * state.AngularDisplacement[i] + torsionalStiffnessRight[i] * PhiPlus1 );
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
                return Math.Max(stiffness, 0);
            }
            else
            {
                return Math.Min(stiffness, 0);
            }
        }
        /// <summary>
        /// Main per-step force and acceleration computation, in four passes:
        /// <list type="number">
        ///   <item>Axial-torsional pass: axial and torsional elastic forces, elastic tension and torque
        ///         at the axial-torsional nodes, and the top-drive torque.</item>
        ///   <item>Linear interpolation of the axial-torsional values to the lateral nodes.</item>
        ///   <item>Lateral pass: polar coordinates, wellbore-wall collision, tension-corrected lateral
        ///         elastic forces, pre-stress, fluid and unbalance forces, Coulomb / static friction and
        ///         lateral accelerations.</item>
        ///   <item>The axial friction forces and friction torques of the lateral nodes are converted to
        ///         consistent nodal loads, the bit-rock model is called at the bit node, and the axial and
        ///         torsional accelerations are computed.</item>
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

            #region 1 - Axial-torsional elastic forces
            for (int j = 0; j < numberOfAxialNodes; j++)
            {
                axialElasticForce[j] = CalculateAxialElasticForce(j, state);
                torsionalElasticTorque[j] = CalculateTorsionalElasticForce(j, state);
                // Axial acceleration without friction, used for the static friction check at the lateral nodes
                trialAxialAcceleration[j] = (axialElasticForce[j] - massProportionalDampingAxial[j] * state.ZVelocity[j]) / axialLumpedMass[j];
            }
            RecoverNodalElasticForces(state);
            // Topdrive Torque only applies to the first element
            double topDriveTorque = (
                                state.TopDrive.TopDriveMotorTorque
                                - torsionalStiffnessMid[0] * (state.TopDrive.AngularDisplacement - state.AngularDisplacement[0])
                            );
            state.TopDrive.AngularAcceleration = topDriveTorque / parameters.TopDriveDrawwork.TopDriveInertia;
            #endregion

            #region 2 - Linear interpolation to the lateral nodes
            drillString.InterpolateToLateral(state.ZDisplacement, state.AxialDisplacementAtLateralNodes);
            drillString.InterpolateToLateral(state.ZVelocity, state.AxialVelocityAtLateralNodes);
            drillString.InterpolateToLateral(state.ZAcceleration, state.AxialAccelerationAtLateralNodes);
            drillString.InterpolateToLateral(state.AngularDisplacement, state.AngularDisplacementAtLateralNodes);
            drillString.InterpolateToLateral(state.AngularVelocity, state.AngularVelocityAtLateralNodes);
            drillString.InterpolateToLateral(state.AngularAcceleration, state.AngularAccelerationAtLateralNodes);
            drillString.InterpolateToLateral(trialAxialAcceleration, trialAxialAccelerationAtLateralNodes);
            drillString.InterpolateToLateral(nodalTorque, torqueAtLateralNodes);
            for (int i = 0; i < numberOfLateralNodes; i++)
            {
                tensionAtLateralNodes[i] = staticTension[i] + drillString.InterpolateToLateral(nodalElasticTension, i);
            }
            #endregion

            #region 3 - Lateral pass
            for (int i = 0; i < numberOfLateralNodes; i++)
            {
                #region Polar Coordinates Conversion
                // Get the radial displacement
                double radialDisplacement = Math.Sqrt(state.XDisplacement[i] * state.XDisplacement[i] + state.YDisplacement[i] * state.YDisplacement[i]);
                // Calculate the whirl angle
                double whirlAngle = Math.Atan2(state.YDisplacement[i], state.XDisplacement[i]);
                // Pre-calculate whirl angle sin and cos to avoid multiple calculations
                double cosWhirlAngle = whirlAngle == 0.0 ? 1.0 : state.XDisplacement[i]/radialDisplacement;
                double sinWhirlAngle = whirlAngle == 0.0 ? 0.0 : state.YDisplacement[i]/radialDisplacement;
                // Calculate the radial velocity
                double radialVelocity = state.XVelocity[i] * cosWhirlAngle + state.YVelocity[i] * sinWhirlAngle;
                // Calculate whirl velocity
                double whirlVelocity = radialDisplacement == 0.0 ? 1E6 : (state.YVelocity[i] * state.XDisplacement[i] - state.XVelocity[i] * state.YDisplacement[i])/(radialDisplacement * radialDisplacement);
                #endregion

                #region Collision calculation
                // Check if there is collision or not and store the Heaveside Step Function
                double heavesideStep = radialDisplacement >= parameters.Wellbore.DrillStringClearance[i] ? 1.0 : 0.0;
                // Calculate normal force
                double normalCollisionForce = heavesideStep *
                    (
                        parameters.Wellbore.WallStiffness * (radialDisplacement - parameters.Wellbore.DrillStringClearance[i])
                       + parameters.Wellbore.WallDamping * radialVelocity
                    );

                if (state.NormalCollisionForce.Count > 1 && i == state.NormalCollisionForce.Count - 2)
                    normalCollisionForce += parameters.Input.ForceToInduceBitWhirl;
                #endregion

                #region Elastic Forces Calculation
                // ------------------ Lateral Elastic Forces Calculation ------------------
                // If it is the first element, get the pinned boundary condition
                double XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                double YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                // If it is the last element, get the pinned boundary condition
                double XiPlus1 = (i == numberOfLateralNodes - 1) ? 0.0 : state.XDisplacement[i + 1];
                double YiPlus1 = (i == numberOfLateralNodes - 1) ? 0.0 : state.YDisplacement[i + 1];
                //  The geometric stiffness of each lateral element uses the tension at its mid-point,
                // which is the average of the linear tension profile at its nodes
                double leftGeometricStiffness = (i == 0) ? 0.0 :
                        0.5 * (tensionAtLateralNodes[i - 1] + tensionAtLateralNodes[i]) * lateralGeometricStiffnessCoefficient[i - 1] + lateralPressureInducedStiffness[i - 1];
                double rightGeometricStiffness = (i == numberOfLateralNodes - 1) ? 0.0 :
                        0.5 * (tensionAtLateralNodes[i] + tensionAtLateralNodes[i + 1]) * lateralGeometricStiffnessCoefficient[i] + lateralPressureInducedStiffness[i];
                //  Corrects the lateral stiffness with information from the pressure gradient and from the tension distribution
                //The NegativeStiffnessTreatment function breakes the connection between elements in case of buckling, as in a string model
                double equivalentLeftStiffness = NegativeStiffnessTreatment(lateralStiffnessLeft[i] - leftGeometricStiffness, centerElement: false);
                double equivalentMidStiffness = NegativeStiffnessTreatment(lateralStiffnessMid[i] + leftGeometricStiffness + rightGeometricStiffness, centerElement: true);
                double equivalentRightStiffness = NegativeStiffnessTreatment(lateralStiffnessRight[i] - rightGeometricStiffness, centerElement: false);
                double elasticForceX = - ( equivalentLeftStiffness * XiMinus1  + equivalentMidStiffness * state.XDisplacement[i] + equivalentRightStiffness * XiPlus1 );
                double elasticForceY = - ( equivalentLeftStiffness * YiMinus1  + equivalentMidStiffness * state.YDisplacement[i] + equivalentRightStiffness * YiPlus1 );
                #endregion
                #region Pre-Stress Force Calculation
                double PreStressForceX = preStressNormalForce[i] * Math.Sin(toolFaceAngle[i]) +
                                preStressBinormalForce[i] * Math.Cos(toolFaceAngle[i]) ;
                double PreStressForceY = preStressNormalForce[i] * Math.Cos(toolFaceAngle[i]) -
                                preStressBinormalForce[i] * Math.Sin(toolFaceAngle[i]) ;
                #endregion
                #region Fluid Force Calculation
                //Extracts angular speed depending on if it is a sleeve or not
                bool hasSleeve = state.SleeveToLumpedIndex[i] != -1;
                //If it is not used properly, it should crash the code.
                int sleeveIndex = hasSleeve ?  state.SleeveToLumpedIndex[i] : -1;
                //  Mask between sleeve and non-sleeve nodes. The pipe speed comes from the torsional model, interpolated to the lateral node
                double rotationSpeed = hasSleeve ? state.SleeveAngularVelocity[sleeveIndex] : state.AngularVelocityAtLateralNodes[i];
                double rotationSpeedSquared = rotationSpeed * rotationSpeed;
                double fluidForceX =  addedLateralFluidMass[i] *
                                (
                                    + 0.25 * rotationSpeedSquared * state.XDisplacement[i]
                                    - rotationSpeed * state.YVelocity[i]
                                )
                                - parameters.Flow.FluidDampingCoefficient * ( state.XVelocity[i] + 0.5 * rotationSpeed * state.YDisplacement[i] );
                double fluidForceY =  addedLateralFluidMass[i] *
                                (
                                    + 0.25 * rotationSpeedSquared * state.YDisplacement[i]
                                    + rotationSpeed * state.XVelocity[i]
                                )
                                - parameters.Flow.FluidDampingCoefficient * (state.YVelocity[i] - 0.5 * rotationSpeed * state.XDisplacement[i]);
                #endregion
                // Sleeve braking force
                double sleeveBrakeForce = hasSleeve ? parameters.Drillstring.SleeveTorsionalDamping[sleeveIndex] * (rotationSpeed - state.SleeveAngularVelocity[sleeveIndex]) : 0;
                #region Unbalance Forces
                // lateral forces due to mass imbalance
                // The imbalance force comes from the assumption that the pipe element center of mass i slocated at a distance from its geometric center,
                // which causes a lateral force and a torque as the pipe is displaced
                // The unbalance uses the pipe properties, not the sleeve. This is because the pipe is still rotating withing the sleeve.
                double pipeAngularVelocity = state.AngularVelocityAtLateralNodes[i];
                double unbalanceForceX = nodeEccentricity[i] *
                    (
                        pipeAngularVelocity * pipeAngularVelocity * Math.Cos(state.AngularDisplacementAtLateralNodes[i])
                        + state.AngularAccelerationAtLateralNodes[i] * Math.Sin(state.AngularDisplacementAtLateralNodes[i])
                    );
                double unbalanceForceY = nodeEccentricity[i] *
                    (
                        pipeAngularVelocity * pipeAngularVelocity * Math.Sin(state.AngularDisplacementAtLateralNodes[i])
                        - state.AngularAccelerationAtLateralNodes[i] * Math.Cos(state.AngularDisplacementAtLateralNodes[i])
                    );
                #endregion
                #region Coulomb Friction
                // Axial velocity
                double axialVelocity = state.AxialVelocityAtLateralNodes[i];
                // "Masks" sleeve or non-sleeve variables if hasSleeve = true
                double outerRadius = hasSleeve ? parameters.Drillstring.SleeveOuterRadius : parameters.Drillstring.NodeOuterRadius[i];
                // The total normal force is the sum of elastic, pre-stress, fluid, unbalance forces, damping and colission forces
                double sumForcesX = elasticForceX + PreStressForceX + fluidForceX + unbalanceForceX - massProportionalDampingLateral[i] * state.XVelocity[i] - normalCollisionForce * cosWhirlAngle;
                double sumForcesY = elasticForceY + PreStressForceY + fluidForceY + unbalanceForceY - massProportionalDampingLateral[i] * state.YVelocity[i] - normalCollisionForce * sinWhirlAngle;
                // Axial force acting at the lateral node: lumped mass times the linearly interpolated trial axial acceleration
                double sumForcesZ = lateralLumpedMass[i] * trialAxialAccelerationAtLateralNodes[i];
                double coulombFrictionX = 0.0;
                double coulombFrictionY = 0.0;
                double coulombFrictionZ = 0.0;
                double frictionTorque;
                // Skip the calculation if there is no collision
                if (heavesideStep == 1.0)
                {
                    // With this 4nDoF model, no slip condition is only possible if vz = 0.
                    double noSlipWhirlAcceleration = 0;
                    double noSlipThetaDotDot = 0;
                    // Define tangential velocity
                    double tangentialVelocityX = state.XVelocity[i] - outerRadius * rotationSpeed * sinWhirlAngle;
                    double tangentialVelocityY = state.YVelocity[i] + outerRadius * rotationSpeed * cosWhirlAngle;
                    double tangentialVelocityZ = axialVelocity;
                    double tangentialMagnitude = Math.Sqrt(tangentialVelocityX * tangentialVelocityX + tangentialVelocityY * tangentialVelocityY + tangentialVelocityZ * tangentialVelocityZ) + Constants.RegularizationCoefficient;
                    // Get tangential direction unit vector
                    double tangentialDirectionX = tangentialVelocityX / tangentialMagnitude;
                    double tangentialDirectionY = tangentialVelocityY / tangentialMagnitude;
                    double tangentialDirectionZ = tangentialVelocityZ / tangentialMagnitude;
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
                        noSlipThetaDotDot = (radialVelocity * whirlVelocity + yDotDot * cosWhirlAngle - xDotDot * sinWhirlAngle) / outerRadius;
                    }
                    //  Check for a static condition by evaluating the tangential speed at the contact point.
                    // Note that on a pure rolling condition, must be tangentialMagnitude = 0. This does not imply that there
                    // the pipe is static.
                    if (Math.Abs(tangentialMagnitude) < 1E-6)
                    {
                        state.SlipCondition[i] = 0;
                        double tangentialForceProjection = Math.Abs(sumForcesX * tangentialDirectionX + sumForcesY * tangentialDirectionY + sumForcesZ * tangentialDirectionZ);
                        //if the total of forces applied in the tangential direction is smaller than the maximum static friction force, then it is a no slip condition. Otherwise, it is a slip condition and the friction is equal to the stribeck friction.
                        if (tangentialForceProjection < coulombStaticForceMagnitude)
                        {
                            state.SlipCondition[i] = 0;
                            //Projects into the lateral and axial directions
                            coulombFrictionX = - tangentialForceProjection * tangentialDirectionX;
                            coulombFrictionY = - tangentialForceProjection * tangentialDirectionY;
                            coulombFrictionZ = - tangentialForceProjection * tangentialDirectionZ;
                        }
                        else
                        {
                            state.SlipCondition[i] = 1;
                            //Projects into the lateral and axial directions
                            coulombFrictionX = - kinematicFriction * tangentialDirectionX;
                            coulombFrictionY = - kinematicFriction * tangentialDirectionY;
                            coulombFrictionZ = - kinematicFriction * tangentialDirectionZ;
                        }
                    }
                    else
                    {
                        //Projects into the lateral and axial directions
                        coulombFrictionX = - kinematicFriction * tangentialDirectionX;
                        coulombFrictionY = - kinematicFriction * tangentialDirectionY;
                        coulombFrictionZ = - kinematicFriction * tangentialDirectionZ;
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
                        double axialCoulombFrictionForce = coulombFrictionZ *  (1 - parameters.Drillstring.AxialFrictionReduction);
                        double tangentialSleeveForce = Math.Sqrt(coulombFrictionX * coulombFrictionX + coulombFrictionY * coulombFrictionY - axialCoulombFrictionForce * axialCoulombFrictionForce) * Math.Sign(coulombFrictionLateral);
                        coulombFrictionX = - tangentialSleeveForce * tangentialDirectionX;
                        coulombFrictionY = - tangentialSleeveForce * tangentialDirectionY;
                        coulombFrictionZ = axialCoulombFrictionForce;
                        state.SleeveForces[i] = coulombFrictionLateral;
                    }
                    //Update forces
                    sumForcesX += coulombFrictionX;
                    sumForcesY += coulombFrictionY;
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
                // Loads returned to the axial-torsional nodes
                axialFrictionAtLateralNodes[i] = coulombFrictionZ;
                frictionTorqueAtLateralNodes[i] = frictionTorque;
                #region Lateral accelerations
                if (hasSleeve)
                {
                    // Why is there a TimeStep in here?
                    state.SleeveAngularAcceleration[sleeveIndex] = parameters.InnerLoopTimeStep * (sleeveBrakeForce * parameters.Drillstring.SleeveInnerRadius - parameters.Drillstring.SleeveOuterRadius * state.SleeveForces[i]) / parameters.Drillstring.SleeveMassMomentOfInertia;
                }
                double xAcceleration = sumForcesX / (lateralLumpedMass[i] + addedLateralFluidMass[i]);
                double yAcceleration = sumForcesY / (lateralLumpedMass[i] + addedLateralFluidMass[i]);
                //Update state
                state.NormalCollisionForce[i] = normalCollisionForce;
                state.RadialDisplacement[i] = radialDisplacement;
                state.RadialVelocity[i] = radialVelocity;
                state.WhirlAngle[i] = whirlAngle;
                state.WhirlVelocity[i] = whirlVelocity;
                state.XAcceleration[i] = xAcceleration;
                state.YAcceleration[i] = yAcceleration;
                state.Tension[i+1] = tensionAtLateralNodes[i];
                state.Torque[i+1] = torqueAtLateralNodes[i];
                #endregion

                #region  Debbugging Outputs
                if (double.IsNaN(xAcceleration) || double.IsNaN(yAcceleration))
                {
                    Console.WriteLine("NaN detected in lateral calculations at element " + i.ToString() +
                        " XAcc: " + xAcceleration.ToString() +
                        " YAcc: " + yAcceleration.ToString() +
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
            #endregion

            #region 4 - Consistent nodal loads and axial-torsional accelerations
            drillString.ConsistentNodalLoad(axialFrictionAtLateralNodes, axialFrictionNodal);
            drillString.ConsistentNodalLoad(frictionTorqueAtLateralNodes, frictionTorqueNodal);
            for (int j = 0; j < numberOfAxialNodes; j++)
            {
                double elasticForceZ = axialElasticForce[j];
                double elasticForcePhi = torsionalElasticTorque[j];
                double torqueOnBit = 0.0;
                #region Bit-Rock Interaction
                if (j == numberOfAxialNodes - 1)
                {
                    // Populate the bit internal forces accordingly
                    bitInternalForces.ElasticAxialForce = elasticForceZ;
                    bitInternalForces.ElasticTorque = elasticForcePhi;
                    //  Calculate interaction forces on bit based on selected bit-rock model
                    // and update the state accordingly
                    bitRockModel.CalculateInteractionForce(state, in parameters, in bitInternalForces);
                    bitRockModel.ManageStickingOnBottom(state, in parameters, in bitInternalForces);
                    // The torque on bit only apply on the last element
                    torqueOnBit =  parameters.UseMudMotor ? state.MudTorque : state.TorqueOnBit;
                }
                #endregion
                // Sets a boundary-condition like for the angular displacement
                double boundaryTorque = (j == 0) ?
                    + torsionalStiffnessMid[0] * (state.TopDrive.AngularDisplacement - state.AngularDisplacement[0]) : 0;
                double sumTorque =  elasticForcePhi + boundaryTorque + torqueOnBit - frictionTorqueNodal[j] - inertiaProportionalDampingTorsional[j] * state.AngularVelocity[j];
                double boundaryAxial = (j == 0) ?
                    + axialStiffnessMid[0] * (state.TopDrive.RelativeAxialPosition - state.ZDisplacement[0])
                    : 0;
                double sumForcesZ = elasticForceZ - massProportionalDampingAxial[j] * state.ZVelocity[j] + axialFrictionNodal[j] + boundaryAxial;
                state.AngularAcceleration[j] = sumTorque / torsionalLumpedInertia[j];
                state.ZAcceleration[j] = sumForcesZ / axialLumpedMass[j];
                if (double.IsNaN(state.ZAcceleration[j]) || double.IsNaN(state.AngularAcceleration[j]))
                {
                    Console.WriteLine("NaN detected in axial-torsional calculations at node " + j.ToString() +
                        " ZAcc: " + state.ZAcceleration[j].ToString() +
                        " AngularAcc: " + state.AngularAcceleration[j].ToString()
                        );
                    return;
                }
            }
            #endregion
        }
    }
}
