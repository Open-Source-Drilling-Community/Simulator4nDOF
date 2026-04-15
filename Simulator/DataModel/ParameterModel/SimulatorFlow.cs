using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.ModelShared;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    /// <summary>
    /// Manages drilling-fluid flow state along the drill string and annulus, including
    /// PVT-based pressure/density/temperature profiles and buoyancy forces.
    /// Several calculations are documented in: ../AuxiliarDevFiles/PVT_Pressure_Equation.wxmx
    /// </summary>
    public class SimulatorFlow
    {
        // -------------------------------------------------------------------------
        // Public state vectors — one value per lumped cell node
        // -------------------------------------------------------------------------

        /// <summary>Pressure profile inside the drill string at each lumped-cell node [Pa].</summary>
        public Vector<double> StringPressure;

        /// <summary>Fluid mass density inside the drill string at each lumped-cell node [kg/m³].</summary>
        public Vector<double> StringDensity;

        /// <summary>Fluid temperature inside the drill string at each lumped-cell node [°C].</summary>
        public Vector<double> StringTemperature;

        /// <summary>Hydrostatic (static) pressure contribution of the drill-string fluid at each node [Pa].</summary>
        public Vector<double> HydrostaticStringPressure;

        /// <summary>Hydrostatic (static) pressure contribution of the annulus fluid at each node [Pa].</summary>
        public Vector<double> HydrostaticAnnulusPressure;

        /// <summary>Buoyant weight of the drill string per unit length at each node [N/m].</summary>
        public Vector<double> BuoyantWeightPerLength;

        /// <summary>Axial tension gradient (dσ/dx) along the drill string at each node [N/m].</summary>
        public Vector<double> dSigmaDx;

        /// <summary>
        /// Axial component of the buoyancy force arising from cross-sectional area changes
        /// (e.g., at tool joints) at each lumped-cell node [N].
        /// </summary>
        public Vector<double> AxialBuoyancyForceChangeOfDiameters;

        /// <summary>
        /// Normal (lateral) component of the buoyancy force arising from cross-sectional area changes
        /// at each lumped-cell node [N].
        /// </summary>
        public Vector<double> NormalBuoyancyForceChangeOfDiameters;

        /// <summary>Fluid mass density in the annulus at each lumped-cell node [kg/m³].</summary>
        public Vector<double> AnnulusDensity;

        /// <summary>Pressure in the annulus at each lumped-cell node [Pa].</summary>
        public Vector<double> AnnulusPressure;

        /// <summary>Fluid temperature in the annulus at each lumped-cell node [°C].</summary>
        public Vector<double> AnnulusTemperature;

        /// <summary>Fluid damping coefficient used in the lateral dynamics model [N·s/m].</summary>
        public double FluidDampingCoefficient = 3000.0;

        /// <summary>Reference fluid mass density, taken from the drilling-fluid data sheet [kg/m³].</summary>
        public double FluidDensity;

        // -------------------------------------------------------------------------
        // PVT model polynomial coefficients
        // The fluid density is modelled as:
        //   ρ = A + B·T + (C + D·T)·P + (E + F·T)·P²
        // where T is temperature [°C] and P is pressure [Pa].
        // Calibrated from: Cayeux (2021), SPE/IADC 204084-MS.
        // -------------------------------------------------------------------------

        /// <summary>PVT model coefficient A: constant term of the density polynomial [kg/m³].</summary>
        private double APvtCoeff;

        /// <summary>PVT model coefficient B: temperature-linear term of the density polynomial [kg/(m³·°C)].</summary>
        private double BPvtCoeff;

        /// <summary>PVT model coefficient C: pressure-linear term of the density polynomial [kg/(m³·Pa)].</summary>
        private double CPvtCoeff;

        /// <summary>PVT model coefficient D: cross term (pressure × temperature) of the density polynomial [kg/(m³·Pa·°C)].</summary>
        private double DPvtCoeff;

        /// <summary>PVT model coefficient E: pressure-quadratic term of the density polynomial [kg/(m³·Pa²)].</summary>
        private double EPvtCoeff;

        /// <summary>PVT model coefficient F: cross term (pressure² × temperature) of the density polynomial [kg/(m³·Pa²·°C)].</summary>
        private double FPvtCoeff;

        // -------------------------------------------------------------------------
        // ODE solver state vectors
        // -------------------------------------------------------------------------

        /// <summary>
        /// State vector for the annulus ODE integrator: [density (kg/m³), pressure (Pa), temperature (°C)].
        /// </summary>
        private Vector<double> XVectorAnnulusVariables;

        /// <summary>
        /// State vector for the drill-string ODE integrator: [density (kg/m³), pressure (Pa), temperature (°C)].
        /// </summary>
        private Vector<double> XVectorStringVariables;

        // -------------------------------------------------------------------------
        // Configuration-derived private fields
        // -------------------------------------------------------------------------

        /// <summary>Geothermal profile data sorted by increasing vertical depth; null when not available.</summary>
        private List<GeothermalData>? geothermalDataList;

        /// <summary>Temperature gradient at the current integration depth [°C/m], updated each integration step.</summary>
        private double currentTemparetureGratient;

        /// <summary>Drilling-fluid description (composition, PVT parameters, reference conditions).</summary>
        private DrillingFluidDescription drillingFluidDescription;

        /// <summary>Wellhead (surface) pressure used as the annulus boundary condition [Pa].</summary>
        private readonly double wellheadPressure;

        /// <summary>
        /// When <c>true</c>, buoyancy is computed via the buoyancy-factor approach;
        /// otherwise the explicit fluid-volume approach is used.
        /// </summary>
        private readonly bool useBuoyancyFactor;

        /// <summary>Pump pressure used as the drill-string boundary condition at surface [Pa].</summary>
        private readonly double pumpPressure;

        /// <summary>Geothermal properties (depth–temperature profile); null when not configured.</summary>
        private readonly GeothermalProperties? geothermalProperties;

        // -------------------------------------------------------------------------
        // Constructor
        // -------------------------------------------------------------------------

        /// <summary>
        /// Initialises the <see cref="SimulatorFlow"/> instance, allocates all state vectors,
        /// resolves PVT coefficients (from composition or from defaults), and integrates
        /// the initial pressure/density/temperature profile along the well.
        /// </summary>
        /// <param name="configuration">Simulator configuration (fluid, wellhead pressure, buoyancy flag, …).</param>
        /// <param name="lumpedCells">Lumped-cell discretisation (number of elements, cumulative lengths).</param>
        /// <param name="trajectory">Well trajectory (inclination, vertical-depth profile).</param>
        /// <param name="drillString">Drill-string geometry and material properties.</param>
        public SimulatorFlow(
            in Configuration configuration,            
            in SimulatorTrajectory trajectory,
            in SimulatorDrillString drillString)
        {
            int numberOfElements = drillString.ElementLength.Count;
            // --- Extract scalar configuration values ---
            drillingFluidDescription = configuration.DrillingFluidDescription;
            wellheadPressure = configuration.WellheadPressure;
            useBuoyancyFactor = configuration.UseBuoyancyFactor;
            pumpPressure = configuration.PumpPressure;
            geothermalProperties = configuration.GeothermalProperties;

            // Default temperature gradient [°C/m] used when no geothermal data is available
            currentTemparetureGratient = 0.03;

            // Sort geothermal data by increasing vertical depth so binary-search-style lookups work correctly
            geothermalDataList = geothermalProperties == null
                ? null
                : geothermalProperties.GeothermalDataList
                    .Where(d => d.VerticalDepth != null)
                    .OrderBy(d => d.VerticalDepth)
                    .ToList();

            // --- Reference fluid density ---
            // If no density value is available, fall back to the default calibrated from Fig. 1.b) of:
            // Cayeux, Eric "Automatic Measurement of the Dependence on Pressure and Temperature of the
            // Mass Density of Drilling Fluids." SPE/IADC 204084-MS, Virtual, March 2021.
            FluidDensity = drillingFluidDescription.FluidMassDensity.GaussianValue.Mean ?? 1750;

            // --- Allocate state vectors (one entry per lumped-cell node) ---
            AnnulusDensity    = Vector<double>.Build.Dense(numberOfElements + 1);
            AnnulusPressure   = Vector<double>.Build.Dense(numberOfElements + 1);
            AnnulusTemperature = Vector<double>.Build.Dense(numberOfElements + 1);
            StringDensity     = Vector<double>.Build.Dense(numberOfElements + 1);
            StringPressure    = Vector<double>.Build.Dense(numberOfElements + 1);
            StringTemperature = Vector<double>.Build.Dense(numberOfElements + 1);

            HydrostaticStringPressure  = Vector<double>.Build.Dense(numberOfElements + 1);
            HydrostaticAnnulusPressure = Vector<double>.Build.Dense(numberOfElements + 1);

            // ODE state vectors: [density, pressure, temperature]
            XVectorAnnulusVariables = Vector<double>.Build.Dense(3);
            XVectorStringVariables  = Vector<double>.Build.Dense(3);

            BuoyantWeightPerLength                = Vector<double>.Build.Dense(numberOfElements + 1);
            AxialBuoyancyForceChangeOfDiameters   = Vector<double>.Build.Dense(numberOfElements + 1);
            NormalBuoyancyForceChangeOfDiameters  = Vector<double>.Build.Dense(numberOfElements + 1);
            dSigmaDx = Vector<double>.Build.Dense(numberOfElements + 1);

            // --- Resolve PVT coefficients ---
            FluidPVTParameters? fluidPVTParameters;
            if (drillingFluidDescription.FluidPVTParameters == null)
            {
                // Attempt to derive combined PVT coefficients from the fluid composition (brine + base oil)
                fluidPVTParameters = CalculateFluidPVTParameters(
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.PVTParameters,
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.PVTParameters,
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.MassFraction.GaussianValue.Mean,
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.MassDensity.GaussianValue.Mean,
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.MassFraction.GaussianValue.Mean,
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.MassDensity.GaussianValue.Mean
                );

                // If composition-based derivation also failed, fall back to standard values
                // calibrated from Fig. 1.b) of Cayeux (2021), SPE/IADC 204084-MS
                if (fluidPVTParameters == null)
                {
                    fluidPVTParameters = new FluidPVTParameters
                    {
                        A0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =  1981.9832345443956}},
                        B0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =    -0.8671702042673033}},
                        C0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =    -1.8110549001629749e-6}},
                        D0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =     7.341326730109053e-9}},
                        E0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =     2.3352470230170143e-14}},
                        F0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean =    -6.361566331825902e-17}}
                    };
                }
            }
            else
            {
                // Use the PVT parameters already supplied in the fluid description
                fluidPVTParameters = drillingFluidDescription.FluidPVTParameters;
            }

            // --- Integrate pressure/density/temperature profiles along the well ---
            IntegratePressureProfile(
                fluidPVTParameters,
                FluidDensity,
                drillingFluidDescription.ReferenceTemperature.GaussianValue.Mean,
                wellheadPressure,
                pumpPressure,
                drillString.RelativeNodeDepth);

            // --- Compute initial buoyancy distribution ---
            UpdateBuoyancy(trajectory, drillString, useBuoyancyFactor);
        }

        // -------------------------------------------------------------------------
        // Private methods
        // -------------------------------------------------------------------------

        /// <summary>
        /// Integrates the PVT ordinary differential equation (ODE) along the cumulative
        /// depth profile to populate the annulus and drill-string pressure, density, and
        /// temperature vectors.
        /// </summary>
        /// <param name="fluidPVTParameters">Six-coefficient polynomial PVT model (A–F).</param>
        /// <param name="densityNullable">Reference fluid density at surface conditions [kg/m³].</param>
        /// <param name="temperatureNullable">Reference fluid temperature at surface conditions [°C].</param>
        /// <param name="wellheadPressure">Annulus boundary condition at surface [Pa].</param>
        /// <param name="pumpPressure">Drill-string boundary condition at surface (pump pressure) [Pa].</param>
        /// <param name="depthIntegrationProfile">Cumulative along-hole depth at each lumped-cell node [m].</param>
        private void IntegratePressureProfile(
            FluidPVTParameters fluidPVTParameters,
            double? densityNullable,
            double? temperatureNullable,
            double wellheadPressure,
            double pumpPressure,
            List<double> depthIntegrationProfile)
        {
            // Guard: all PVT coefficients and reference conditions must be provided
            if (
                fluidPVTParameters.A0.GaussianValue.Mean == null ||
                fluidPVTParameters.B0.GaussianValue.Mean == null ||
                fluidPVTParameters.C0.GaussianValue.Mean == null ||
                fluidPVTParameters.D0.GaussianValue.Mean == null ||
                fluidPVTParameters.E0.GaussianValue.Mean == null ||
                fluidPVTParameters.F0.GaussianValue.Mean == null ||
                densityNullable == null ||
                temperatureNullable == null
            )
            {
                throw new ArgumentException("Required fluid PVT parameters or density/temperature values are null.");
            }
            else
            {
                double currentDepth = 0;

                // Runge-Kutta-Fehlberg 4(5) solver for the pressure/density/temperature ODE
                RKF45 odeSolver = new RKF45(ΔxRef: 10);

                // Cache PVT coefficients as class fields so PVTOrdinaryDifferentialEquation can access them
                APvtCoeff = (double) fluidPVTParameters.A0.GaussianValue.Mean;
                BPvtCoeff = (double) fluidPVTParameters.B0.GaussianValue.Mean;
                CPvtCoeff = (double) fluidPVTParameters.C0.GaussianValue.Mean;
                DPvtCoeff = (double) fluidPVTParameters.D0.GaussianValue.Mean;
                EPvtCoeff = (double) fluidPVTParameters.E0.GaussianValue.Mean;
                FPvtCoeff = (double) fluidPVTParameters.F0.GaussianValue.Mean;

                // --- Compute surface (node 0) reference conditions ---
                double referenceTemperature = (double) temperatureNullable;
                double referencePressure    = wellheadPressure;

                // Evaluate density at surface using the PVT polynomial:  ρ = A + B·T + (C + D·T)·P + (E + F·T)·P²
                double referenceDensity = (
                    APvtCoeff + BPvtCoeff * referenceTemperature
                    + (CPvtCoeff + DPvtCoeff * referenceTemperature) * referencePressure
                    + (EPvtCoeff + FPvtCoeff * referenceTemperature) * referencePressure * referencePressure
                );

                // --- Set annulus initial conditions at node 0 ---
                double annulusDensity;
                double annulusTemperature;
                double annulusPressure;

                (GeothermalData, GeothermalData)? bounds = GetGeothermalDataBounds(depthIntegrationProfile[0]);

                if (bounds != null)
                {
                    // Derive the initial annulus temperature from the geothermal profile via linear interpolation
                    var (lowerGeothermalData, upperGeothermalData) = bounds.Value;
                    double lowerDepth       = lowerGeothermalData.VerticalDepth ?? currentDepth;
                    double upperDepth       = upperGeothermalData.VerticalDepth ?? currentDepth;
                    double lowerTemperature = lowerGeothermalData.Temperature   ?? referenceTemperature;
                    double upperTemperature = upperGeothermalData.Temperature   ?? referenceTemperature;
                    double localGradient    = (upperDepth != lowerDepth)
                        ? (upperTemperature - lowerTemperature) / (upperDepth - lowerDepth)
                        : (lowerGeothermalData.TemperatureGradient ?? 0.03);
                    double tempAtZero       = lowerTemperature - lowerDepth * localGradient;

                    annulusTemperature = localGradient * currentDepth + tempAtZero;
                    annulusPressure    = referencePressure;
                    annulusDensity     = APvtCoeff + BPvtCoeff * annulusTemperature
                                        + (CPvtCoeff + DPvtCoeff * annulusTemperature) * annulusPressure
                                        + (EPvtCoeff + FPvtCoeff * annulusTemperature) * annulusPressure * annulusPressure;
                }
                else
                {
                    // No geothermal data: use fluid datasheet reference conditions directly
                    annulusDensity     = referenceDensity;
                    annulusPressure    = referencePressure;
                    annulusTemperature = referenceTemperature;
                }

                // --- Infer the drill-string initial temperature from the pump pressure and annulus density ---
                // Solving the PVT polynomial for T given ρ (annulusDensity) and P (pumpPressure):
                //   T = (ρ - E·P² - C·P - A) / (F·P² + D·P + B)
                double drillStringTemperature = (
                    annulusDensity - EPvtCoeff * pumpPressure * pumpPressure
                                   - CPvtCoeff * pumpPressure
                                   - APvtCoeff)
                    / (FPvtCoeff * pumpPressure * pumpPressure + DPvtCoeff * pumpPressure + BPvtCoeff);

                // --- Pack initial conditions into ODE state vectors ---
                XVectorAnnulusVariables[0] = annulusDensity;
                XVectorAnnulusVariables[1] = annulusPressure;
                XVectorAnnulusVariables[2] = annulusTemperature;

                XVectorStringVariables[0] = annulusDensity;     // same density at surface
                XVectorStringVariables[1] = pumpPressure;
                XVectorStringVariables[2] = drillStringTemperature;

                // Store node-0 values in the public state vectors
                AnnulusDensity[0]     = annulusDensity;
                AnnulusTemperature[0] = annulusTemperature;
                AnnulusPressure[0]    = annulusPressure;

                StringDensity[0]     = annulusDensity;
                StringTemperature[0] = drillStringTemperature;
                StringPressure[0]    = pumpPressure;

                // --- Integrate from node 1 to the last node using the RKF45 solver ---
                double dX;
                for (int i = 1; i < depthIntegrationProfile.Count; i++)
                {
                    // Step size equals the length of the previous element
                    dX = i > 1
                        ? depthIntegrationProfile[i - 1] - depthIntegrationProfile[i - 2]
                        : depthIntegrationProfile[0];
                    currentDepth = depthIntegrationProfile[i - 1];

                    // Update the temperature gradient from the geothermal profile at the current depth
                    currentTemparetureGratient = EstimateTemperatureGradient(currentDepth);

                    // Advance the annulus state one step
                    XVectorAnnulusVariables = odeSolver.Solve(XVectorAnnulusVariables, PVTOrdinaryDifferentialEquation, currentDepth, dX);
                    AnnulusDensity[i]     = XVectorAnnulusVariables[0];
                    AnnulusPressure[i]    = XVectorAnnulusVariables[1];
                    AnnulusTemperature[i] = XVectorAnnulusVariables[2];

                    // Advance the drill-string state one step
                    XVectorStringVariables = odeSolver.Solve(XVectorStringVariables, PVTOrdinaryDifferentialEquation, currentDepth, dX);
                    StringDensity[i]     = XVectorStringVariables[0];
                    StringPressure[i]    = XVectorStringVariables[1];
                    StringTemperature[i] = XVectorStringVariables[2];
                }
            }
        }

        /// <summary>
        /// Estimates the local temperature gradient [°C/m] at a given vertical depth by
        /// linearly interpolating between the two nearest geothermal data points.
        /// Returns the default gradient of 0.03 °C/m when no geothermal data is available.
        /// </summary>
        /// <param name="currentDepth">Vertical depth at which the gradient is evaluated [m].</param>
        /// <returns>Temperature gradient [°C/m].</returns>
        private double EstimateTemperatureGradient(double currentDepth)
        {
            if (geothermalDataList != null)
            {
                (GeothermalData, GeothermalData)? bounds = GetGeothermalDataBounds(currentDepth);
                if (bounds.HasValue)
                {
                    // Linearly interpolate the gradient between the bracketing geothermal points
                    var (lower, upper) = bounds.Value;
                    double lowerDepth    = lower.VerticalDepth    ?? currentDepth;
                    double upperDepth    = upper.VerticalDepth    ?? currentDepth;
                    double lowerGradient = lower.TemperatureGradient ?? currentTemparetureGratient;
                    double upperGradient = upper.TemperatureGradient ?? currentTemparetureGratient;
                    return (upperDepth != lowerDepth)
                        ? lowerGradient + (upperGradient - lowerGradient) * (currentDepth - lowerDepth) / (upperDepth - lowerDepth)
                        : lowerGradient;
                }
                else
                {
                    // Depth is outside the range of available geothermal data
                    return 0.03;
                }
            }
            else
            {
                // No geothermal data configured; use global default
                return 0.03;
            }
        }

        /// <summary>
        /// Finds the two geothermal data points that bracket <paramref name="depth"/>,
        /// returning them as (lower, upper). If <paramref name="depth"/> is outside the
        /// available range, both elements of the pair are set to the nearest boundary point.
        /// Returns <c>null</c> when no geothermal data is available.
        /// </summary>
        /// <param name="depth">Vertical depth to look up [m].</param>
        /// <returns>
        /// A tuple (Lower, Upper) of the bracketing <see cref="GeothermalData"/> records,
        /// or <c>null</c> if the geothermal list is empty or null.
        /// </returns>
        private (GeothermalData Lower, GeothermalData Upper)? GetGeothermalDataBounds(double depth)
        {
            if (geothermalDataList == null || geothermalDataList.Count == 0)
            {
                return null;
            }

            // Depth is shallower than the shallowest available point — clamp to the first entry
            if (depth <= geothermalDataList[0].VerticalDepth)
            {
                return (geothermalDataList[0], geothermalDataList[0]);
            }

            var lastIndex = geothermalDataList.Count - 1;

            // Depth is deeper than the deepest available point — clamp to the last entry
            if (depth >= geothermalDataList[lastIndex].VerticalDepth)
            {
                return (geothermalDataList[lastIndex], geothermalDataList[lastIndex]);
            }

            // Find the two consecutive points that bracket the desired depth
            for (int i = 1; i < geothermalDataList.Count; i++)
            {
                var upper = geothermalDataList[i];
                var lower = geothermalDataList[i - 1];
                if (upper.VerticalDepth >= depth)
                {
                    return (lower, upper);
                }
            }

            // Fallback: should not be reached given the checks above
            return (geothermalDataList[lastIndex], geothermalDataList[lastIndex]);
        }

        /// <summary>
        /// Derives a combined set of fluid PVT polynomial coefficients (A–F) for a
        /// brine–base-oil mixture by volume-fraction weighting of the individual
        /// component coefficients.
        /// </summary>
        /// <param name="brinePVTParameters">PVT coefficients for the brine phase.</param>
        /// <param name="baseOilPVTParameters">PVT coefficients for the base-oil phase.</param>
        /// <param name="brineMassFraction">Mass fraction of the brine component [–].</param>
        /// <param name="brineMass">Mass of the brine sample used to determine its volume [kg].</param>
        /// <param name="baseOilMassFraction">Mass fraction of the base-oil component [–].</param>
        /// <param name="baseOilMass">Mass of the base-oil sample used to determine its volume [kg].</param>
        /// <returns>
        /// A <see cref="FluidPVTParameters"/> containing the volume-weighted mixture coefficients,
        /// or <c>null</c> if any required input is missing.
        /// </returns>
        private FluidPVTParameters? CalculateFluidPVTParameters(
            BrinePVTParameters brinePVTParameters,
            BaseOilPVTParameters baseOilPVTParameters,
            double? brineMassFraction,
            double? brineMass,
            double? baseOilMassFraction,
            double? baseOilMass
        )
        {
            // Guard: all component parameters must be available to compute a mixture model
            if (
                baseOilPVTParameters.A0.GaussianValue.Mean == null ||
                baseOilPVTParameters.B0.GaussianValue.Mean == null ||
                baseOilPVTParameters.C0.GaussianValue.Mean == null ||
                baseOilPVTParameters.D0.GaussianValue.Mean == null ||
                baseOilPVTParameters.E0.GaussianValue.Mean == null ||
                baseOilPVTParameters.F0.GaussianValue.Mean == null ||
                brinePVTParameters.S0.GaussianValue.Mean == null ||
                brinePVTParameters.S1.GaussianValue.Mean == null ||
                brinePVTParameters.S2.GaussianValue.Mean == null ||
                brinePVTParameters.S3.GaussianValue.Mean == null ||
                brinePVTParameters.Bw.GaussianValue.Mean == null ||
                brinePVTParameters.Cw.GaussianValue.Mean == null ||
                brinePVTParameters.Dw.GaussianValue.Mean == null ||
                brinePVTParameters.Ew.GaussianValue.Mean == null ||
                brinePVTParameters.Fw.GaussianValue.Mean == null ||
                brineMassFraction == null ||
                brineMass == null ||
                baseOilMassFraction == null ||
                baseOilMass == null
            )
            {
                return null;
            }
            else
            {
                // --- Compute component volumes from mass and mass fraction ---
                // Volume = mass / mass-fraction  (since mass-fraction = mass / total_volume · density ≡ mass / volume here)
                double baseOilVolume = (double) baseOilMass / (double) baseOilMassFraction;
                double brineVolume   = (double) brineMass   / (double) brineMassFraction;

                // --- Extract base-oil PVT coefficients ---
                double ABaseOil = (double) baseOilPVTParameters.A0.GaussianValue.Mean;
                double BBaseOil = (double) baseOilPVTParameters.B0.GaussianValue.Mean;
                double CBaseOil = (double) baseOilPVTParameters.C0.GaussianValue.Mean;
                double DBaseOil = (double) baseOilPVTParameters.D0.GaussianValue.Mean;
                double EBaseOil = (double) baseOilPVTParameters.E0.GaussianValue.Mean;
                double FBaseOil = (double) baseOilPVTParameters.F0.GaussianValue.Mean;

                // --- Evaluate brine PVT coefficients ---
                // A_brine is a salinity-dependent polynomial: S0 + S1·χ + S2·χ² + S3·χ³ (χ = mass fraction)
                double ABrine = (double) (
                    brinePVTParameters.S0.GaussianValue.Mean
                    + brinePVTParameters.S1.GaussianValue.Mean * brineMassFraction
                    + brinePVTParameters.S2.GaussianValue.Mean * brineMassFraction * brineMassFraction
                    + brinePVTParameters.S3.GaussianValue.Mean * brineMassFraction * brineMassFraction * brineMassFraction
                );
                double BBrine = (double) brinePVTParameters.Bw.GaussianValue.Mean;
                double CBrine = (double) brinePVTParameters.Cw.GaussianValue.Mean;
                double DBrine = (double) brinePVTParameters.Dw.GaussianValue.Mean;
                double EBrine = (double) brinePVTParameters.Ew.GaussianValue.Mean;
                double FBrine = (double) brinePVTParameters.Fw.GaussianValue.Mean;

                // --- Volume fractions for mixture weighting ---
                double ratioOil   = baseOilVolume / (baseOilVolume + brineVolume);
                double ratioBrine = 1.0 - ratioOil;

                // --- Return volume-fraction-weighted mixture PVT coefficients ---
                return new FluidPVTParameters
                {
                    A0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * ABaseOil + ratioBrine * ABrine} },
                    B0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * BBaseOil + ratioBrine * BBrine} },
                    C0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * CBaseOil + ratioBrine * CBrine} },
                    D0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * DBaseOil + ratioBrine * DBrine} },
                    E0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * EBaseOil + ratioBrine * EBrine} },
                    F0 = new GaussianDrillingProperty{ GaussianValue = new GaussianDistribution { Mean = ratioOil * FBaseOil + ratioBrine * FBrine} }
                };
            }
        }

        /// <summary>
        /// Defines the right-hand side of the PVT ODE system used to propagate
        /// density, pressure, and temperature with depth.
        /// <para>
        /// The state vector is <c>X = [ρ, P, T]</c>. The system is derived from:
        /// <list type="bullet">
        ///   <item>The hydrostatic pressure equation: dP/dz = ρ·g</item>
        ///   <item>The geothermal temperature gradient: dT/dz = γ</item>
        ///   <item>The PVT polynomial: ρ = A + B·T + (C + D·T)·P + (E + F·T)·P²</item>
        /// </list>
        /// Differentiating the PVT polynomial and substituting yields a 3×3 linear system
        /// whose coefficient matrix always has determinant −1 and is therefore always invertible.
        /// </para>
        /// </summary>
        /// <param name="xStep">Current depth coordinate passed by the ODE solver [m] (not used explicitly here).</param>
        /// <param name="Xinputs">State vector [ρ (kg/m³), P (Pa), T (°C)].</param>
        /// <returns>Derivatives vector [dρ/dz, dP/dz, dT/dz].</returns>
        private Vector<double> PVTOrdinaryDifferentialEquation(double xStep, Vector<double> Xinputs)
        {
            // State:  Xinputs = [density, pressure, temperature]

            // --- Right-hand side of the coupled ODE ---
            // d(ρ)/dz = 0  (density changes only through pressure/temperature coupling, handled by the matrix)
            // d(P)/dz = ρ·g
            // d(T)/dz = γ  (current geothermal gradient)
            Vector<double> RightSideOfTheODE = Vector<double>.Build.Dense(3);
            RightSideOfTheODE[0] = 0.0;
            RightSideOfTheODE[1] = Constants.GravitationalAcceleration * Xinputs[0];
            RightSideOfTheODE[2] = currentTemparetureGratient;

            // --- Coefficient matrix from differentiating the PVT polynomial ---
            // Obtained by differentiating  ρ = A + B·T + (C+D·T)·P + (E+F·T)·P²  with respect to depth
            // and coupling with the hydrostatic and geothermal equations.
            // The determinant of this matrix is always −1, so it is always invertible.
            Matrix<double> CoefficientMatrix = Matrix<double>.Build.Dense(3, 3);

            // Column 0: coefficient of dρ/dz
            CoefficientMatrix[0, 0] = -1;
            CoefficientMatrix[1, 0] =  0;
            CoefficientMatrix[2, 0] =  0;

            // Column 1: coefficient of dP/dz  →  ∂ρ/∂P = (C + D·T) + 2·(E + F·T)·P
            CoefficientMatrix[0, 1] = 2 * Xinputs[1] * (FPvtCoeff * Xinputs[2] + EPvtCoeff) + DPvtCoeff * Xinputs[2] + CPvtCoeff;
            CoefficientMatrix[1, 1] =  1;
            CoefficientMatrix[2, 1] =  0;

            // Column 2: coefficient of dT/dz  →  ∂ρ/∂T = B + D·P + F·P²
            CoefficientMatrix[0, 2] = FPvtCoeff * Xinputs[1] * Xinputs[1] + DPvtCoeff * Xinputs[1] + BPvtCoeff;
            CoefficientMatrix[1, 2] =  0;
            CoefficientMatrix[2, 2] =  1;

            // Solve the linear system:  CoefficientMatrix · [dρ/dz, dP/dz, dT/dz]ᵀ = RightSideOfTheODE
            Matrix<double> InvCoefficientMatrix = CoefficientMatrix.Inverse();
            return InvCoefficientMatrix * RightSideOfTheODE;
        }

        // -------------------------------------------------------------------------
        // Public methods
        // -------------------------------------------------------------------------

        /// <summary>
        /// Recomputes the buoyancy-related quantities for all lumped-cell nodes:
        /// hydrostatic pressures, buoyant weight per length (<see cref="BuoyantWeightPerLength"/>),
        /// axial tension gradient (<see cref="dSigmaDx"/>), and buoyancy forces due to
        /// cross-sectional area changes (<see cref="AxialBuoyancyForceChangeOfDiameters"/>,
        /// <see cref="NormalBuoyancyForceChangeOfDiameters"/>).
        /// </summary>
        /// <param name="lumpedCell">Lumped-cell discretisation data.</param>
        /// <param name="trajectory">Well trajectory (inclination angles, vertical depths).</param>
        /// <param name="drillString">Drill-string geometry and density properties.</param>
        /// <param name="useBuoyancyFactor">
        /// When <c>true</c>, uses the buoyancy-factor method; otherwise uses the explicit
        /// fluid-volume method that accounts for tool-joint geometry.
        /// </param>
        public void UpdateBuoyancy(in SimulatorTrajectory trajectory, in SimulatorDrillString drillString, bool useBuoyancyFactor)
        {
            double tempStringPressure  = 0;
            double tempAnnulusPressure = 0;
            double buoyancyFactor;
            double elementInnerArea;
            double elementOuterArea;
            double pipeElementArea;
            double toolJointElementOuterArea;
            double toolJointElementInnerArea;
            double depth;
            double MassPerLength;
            double interpolatedTheta;

            // Prepend a zero depth so that the trapezoidal integral starts from the surface
            Vector<double> extendedDepth = ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth);
            int numberOfElements = drillString.ElementLength.Count;
            // --- Pass 1: integrate hydrostatic pressures using the trapezoidal rule ---
            for (int i = 1; i < numberOfElements; i++)
            {
                depth = i == 0 ? 0 : trajectory.InterpolatedVerticalDepth[i - 1];

                // Trapezoidal increment: ΔP = 0.5 · g · (ρ_i + ρ_{i-1}) · Δz
                tempStringPressure  += 0.5 * Constants.GravitationalAcceleration * (StringDensity[i]  + StringDensity[i - 1])  * (extendedDepth[i] - extendedDepth[i - 1]);
                tempAnnulusPressure += 0.5 * Constants.GravitationalAcceleration * (AnnulusDensity[i] + AnnulusDensity[i - 1]) * (extendedDepth[i] - extendedDepth[i - 1]);

                HydrostaticStringPressure[i]  = tempStringPressure;
                HydrostaticAnnulusPressure[i] = tempAnnulusPressure;
            }

            // --- Pass 2: compute buoyant weight and area-change buoyancy forces ---
            for (int i = 0; i < numberOfElements; i++)
            {
                // For the first node, replicate the properties of the first element
                elementInnerArea          = i == 0 ? drillString.ElementInnerArea[0]          : drillString.ElementInnerArea[i - 1];
                elementOuterArea          = i == 0 ? drillString.ElementOuterArea[0]          : drillString.ElementOuterArea[i - 1];
                pipeElementArea           = i == 0 ? drillString.ElementArea[0]               : drillString.ElementArea[i - 1];
                toolJointElementInnerArea = i == 0 ? drillString.ElementToolJointInnerArea[0] : drillString.ElementToolJointInnerArea[i - 1];
                toolJointElementOuterArea = i == 0 ? drillString.ElementToolJointOuterArea[0] : drillString.ElementToolJointOuterArea[i - 1];

                // Inclination angle at the current node (zero at surface)
                interpolatedTheta = (i == 0) ? 0 : trajectory.InterpolatedTheta[i - 1];

                // Axial tension gradient: dσ/dx = w_b · cos(θ)
                dSigmaDx[i] = BuoyantWeightPerLength[i] * Math.Cos(interpolatedTheta);

                if (useBuoyancyFactor)
                {
                    // --- Buoyancy-factor method ---
                    // The buoyancy factor accounts for differential annulus/string pressures
                    // acting on the pipe cross-section, normalised to the pipe area.
                    buoyancyFactor = (elementOuterArea * (1 - AnnulusDensity[i] / drillString.SteelDensity) -
                                      elementInnerArea * (1 - StringDensity[i]  / drillString.SteelDensity)) /
                                     (elementOuterArea - elementInnerArea);

                    BuoyantWeightPerLength[i] = Constants.GravitationalAcceleration
                                                * buoyancyFactor
                                                * drillString.SteelDensity
                                                * pipeElementArea
                                                * drillString.ElementWeightCorrectionFactor[i];

                    // Hydrostatic pressures at the current and previous nodes
                    double hydrostaticAnnularCurrent = HydrostaticAnnulusPressure[i];
                    double hydrostaticStringCurrent  = HydrostaticStringPressure[i];
                    double hydrostaticAnnularPrev    = i == 0 ? 0 : HydrostaticAnnulusPressure[i - 1];
                    double hydrostaticStringPrev     = i == 0 ? 0 : HydrostaticStringPressure[i - 1];

                    // Tool-joint areas at the previous node (out-of-phase with the current element)
                    double AtjoPrev = i == 0 ? 0 : drillString.ElementToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : drillString.ElementToolJointInnerArea[i - 1];

                    // Axial buoyancy force from area change:
                    // ΔF_axial = A_o·P_ann - A_o_prev·P_ann_prev - A_i·P_str + A_i_prev·P_str_prev
                    AxialBuoyancyForceChangeOfDiameters[i] = elementOuterArea          * hydrostaticAnnularCurrent
                                                           - AtjoPrev                  * hydrostaticAnnularPrev
                                                           - elementInnerArea           * hydrostaticStringCurrent
                                                           + AtjiPrev                  * hydrostaticStringPrev;

                    // Normal (lateral) buoyancy force from area change at the tool-joint shoulder
                    NormalBuoyancyForceChangeOfDiameters[i] = AtjoPrev * hydrostaticAnnularPrev - AtjiPrev * hydrostaticStringPrev;
                }
                else
                {
                    // --- Explicit fluid-volume method ---
                    // Buoyant weight accounts for the displaced annulus fluid and the interior string fluid
                    // over the element length, split between tool-joint and plain-pipe sections.
                    MassPerLength = drillString.SteelDensity * pipeElementArea * drillString.ElementWeightCorrectionFactor[i];

                    BuoyantWeightPerLength[i] = (MassPerLength +
                        (  toolJointElementInnerArea * drillString.ToolJointLength                                    * StringDensity[i]
                         + elementInnerArea           * (drillString.ElementLength[i] - drillString.ToolJointLength) * StringDensity[i]
                         - toolJointElementOuterArea  * drillString.ToolJointLength                                    * AnnulusDensity[i]
                         - elementOuterArea            * (drillString.ElementLength[i] - drillString.ToolJointLength) * AnnulusDensity[i])
                        / drillString.ElementLength[i]) * Constants.GravitationalAcceleration;

                    // No area-change buoyancy forces in this method
                    AxialBuoyancyForceChangeOfDiameters[i]  = 0;
                    NormalBuoyancyForceChangeOfDiameters[i] = 0;
                }
            }
        }
    }
}
