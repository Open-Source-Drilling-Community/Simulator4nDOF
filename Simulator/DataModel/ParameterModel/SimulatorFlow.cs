using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.ModelShared;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulatorFlow
    {
        // Several of the calculation in here can be found in: ../AuxiliarDevFiles/PVT_Pressure_Equation.wxmx
        public Vector<double> StringPressure;
        public Vector<double> StringDensity;
        public Vector<double> StringTemperature;
        public Vector<double> HydrostaticStringPressure;
        public Vector<double> HydrostaticAnnulusPressure;
        public Vector<double> BuoyantWeightPerLength;                                   // [N/m] Buoyant drill string weight per length
        public Vector<double> dSigmaDx;                            // [N/m] Tension per length
        public Vector<double> AxialBuoyancyForceChangeOfDiameters;
        public Vector<double> NormalBuoyancyForceChangeOfDiameters;
        public Vector<double> AnnulusDensity;
        public Vector<double> AnnulusPressure;
        public Vector<double> AnnulusTemperature;
        public double FluidDampingCoefficient = 3000.0; // [N.s/m] Fluid damping coefficient for lateral dynamics
        
        public double FluidDensity;

        private double APvtCoeff;
        private double BPvtCoeff;
        private double CPvtCoeff;
        private double DPvtCoeff;
        private double EPvtCoeff;
        private double FPvtCoeff;
        private Vector<double> XVectorAnnulusVariables;
        private Vector<double> XVectorStringVariables;
        private List<GeothermalData>? geothermalDataList;
        private double currentTemparetureGratient;
        // Auxiliar variables from configurations
        private DrillingFluidDescription drillingFluidDescription;
        private readonly double wellheadPressure;
        private readonly bool useBuoyancyFactor;
        private readonly double pumpPressure; 
        private readonly GeothermalProperties? geothermalProperties;

        public SimulatorFlow(
            in Configuration configuration,
            in LumpedCells lumpedCells, 
            in SimulatorTrajectory trajectory, 
            in SimulatorDrillString drillString)
        {
            drillingFluidDescription = configuration.DrillingFluidDescription;
            wellheadPressure = configuration.WellheadPressure;
            useBuoyancyFactor = configuration.UseBuoyancyFactor;
            pumpPressure = configuration.PumpPressure;
            geothermalProperties = configuration.GeothermalProperties;
            //Default value for temperature gradient 
            currentTemparetureGratient = 0.03;
            // reorder geothermal properties by depth
            geothermalDataList = geothermalProperties == null
                ? null
                : geothermalProperties.GeothermalDataList
                    .Where(d => d.VerticalDepth != null)
                    .OrderBy(d => d.VerticalDepth)
                    .ToList();
            //   IfThere is no density available, use the value calibrated from the figure 1.b):
            // Cayeux, Eric "Automatic Measurement of the Dependence on Pressure and Temperature of the Mass Density of Drilling Fluids." 
            // Paper presented at the SPE/IADC International Drilling Conference and Exhibition,
            // Virtual, March 2021. doi: https://doi.org/10.2118/204084-MS      
            FluidDensity = drillingFluidDescription.FluidMassDensity.GaussianValue.Mean ?? 1750;
            AnnulusDensity = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            AnnulusPressure = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            AnnulusTemperature = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            StringDensity = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            StringPressure = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            StringTemperature = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);

            HydrostaticStringPressure = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            HydrostaticAnnulusPressure = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);

            XVectorAnnulusVariables = Vector<double>.Build.Dense(3);
            XVectorStringVariables = Vector<double>.Build.Dense(3);
            
            BuoyantWeightPerLength = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);  
            AxialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            NormalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);

            dSigmaDx = Vector<double>.Build.Dense(lumpedCells.NumberOfLumpedElements + 1);
            FluidPVTParameters? fluidPVTParameters;
            if (drillingFluidDescription.FluidPVTParameters == null)
            {   
                fluidPVTParameters = CalculateFluidPVTParameters(
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.PVTParameters, 
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.PVTParameters, 
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.MassFraction.GaussianValue.Mean, 
                    drillingFluidDescription.DrillingFluidComposition.BrineProperies.MassDensity.GaussianValue.Mean, 
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.MassFraction.GaussianValue.Mean,
                    drillingFluidDescription.DrillingFluidComposition.BaseOilProperies.MassDensity.GaussianValue.Mean
                );
                //      If it is still null, use standard values calibrated from Fig. 1.b)
                // Cayeux, Eric "Automatic Measurement of the Dependence on Pressure and Temperature of the Mass Density of Drilling Fluids." 
                // Paper presented at the SPE/IADC International Drilling Conference and Exhibition,
                // Virtual, March 2021. doi: https://doi.org/10.2118/204084-MS                
                if (fluidPVTParameters == null)
                {
                    fluidPVTParameters = new FluidPVTParameters
                    {
                        A0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = 1981.9832345443956}},
                        B0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = -0.8671702042673033}},
                        C0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = -1.8110549001629749e-6}},
                        D0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = 7.341326730109053e-9}},
                        E0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = 2.3352470230170143e-14}},
                        F0 = new GaussianDrillingProperty {GaussianValue = new GaussianDistribution {Mean = -6.361566331825902e-17}}                    
                    };
                }
            }
            else
            {
                fluidPVTParameters = drillingFluidDescription.FluidPVTParameters;
            }
            IntegratePressureProfile(
                fluidPVTParameters, 
                FluidDensity, 
                drillingFluidDescription.ReferenceTemperature.GaussianValue.Mean,
                wellheadPressure,
                pumpPressure,
                lumpedCells.CumulativeElementLength);   
            UpdateBuoyancy(lumpedCells, trajectory, drillString, useBuoyancyFactor);
        }
        private void IntegratePressureProfile(
            FluidPVTParameters fluidPVTParameters, 
            double? densityNullable, 
            double? temperatureNullable,
            double wellheadPressure, 
            double pumpPressure,
            Vector<double> depthIntegrationProfile)
        {
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
                // Check what this is
                double currentDepth = 0;
                RKF45 odeSolver = new RKF45(ΔxRef: 10);
                APvtCoeff = (double) fluidPVTParameters.A0.GaussianValue.Mean;
                BPvtCoeff = (double) fluidPVTParameters.B0.GaussianValue.Mean;
                CPvtCoeff = (double) fluidPVTParameters.C0.GaussianValue.Mean;
                DPvtCoeff = (double) fluidPVTParameters.D0.GaussianValue.Mean;
                EPvtCoeff = (double) fluidPVTParameters.E0.GaussianValue.Mean;
                FPvtCoeff = (double) fluidPVTParameters.F0.GaussianValue.Mean;
                double annulusDensity;
                double annulusTemperature;
                double annulusPressure;

                double referenceTemperature = (double) temperatureNullable;                
                 //  If there is no available geothermal data, use the fluid description datasheet to
                // calculate the fluid's properties
                double referencePressure = wellheadPressure;
                double referenceDensity = (
                            APvtCoeff + BPvtCoeff * referenceTemperature
                            + (CPvtCoeff + DPvtCoeff * referenceTemperature) * referencePressure
                            + (EPvtCoeff + FPvtCoeff * referenceTemperature) * referencePressure * referencePressure                        
                        );                             
                        
                (GeothermalData, GeothermalData)? bounds = GetGeothermalDataBounds(depthIntegrationProfile[0]);                

                if (bounds != null)
                {   
                    //Get the initial temperature through the geothermal data
                    var (lowerGeothermalData, upperGeothermalData) = bounds.Value;
                    double lowerDepth = lowerGeothermalData.VerticalDepth ?? currentDepth;
                    double upperDepth = upperGeothermalData.VerticalDepth ?? currentDepth;
                    double lowerTemperature = lowerGeothermalData.Temperature ?? referenceTemperature;
                    double upperTemperature = upperGeothermalData.Temperature ?? referenceTemperature;
                    double localGradient = (upperDepth != lowerDepth) ?  (upperTemperature - lowerTemperature)/(upperDepth - lowerDepth) : (lowerGeothermalData.TemperatureGradient ?? 0.03);
                    double tempAtZero = lowerTemperature - lowerDepth * localGradient;
                    annulusTemperature = localGradient * currentDepth + tempAtZero;                
                    annulusPressure = referencePressure;
                    annulusDensity = APvtCoeff + BPvtCoeff * annulusTemperature
                                    + (CPvtCoeff + DPvtCoeff * annulusTemperature) * annulusPressure
                                    + (EPvtCoeff + FPvtCoeff * annulusTemperature) * annulusPressure * annulusPressure;
                }
                else 
                {
                   annulusDensity = referenceDensity;
                   annulusPressure = referencePressure;
                   annulusTemperature = referenceTemperature;                   
                }
                
                //The Drill-string initial temperature is calculated as:
                double drillStringTemperature = (
                    annulusDensity - EPvtCoeff * pumpPressure * pumpPressure - CPvtCoeff * pumpPressure - APvtCoeff)/
                    (FPvtCoeff * pumpPressure * pumpPressure + DPvtCoeff * pumpPressure + BPvtCoeff);

                //XVector is the solver-ready format of the variables
                XVectorAnnulusVariables[0] = annulusDensity;
                XVectorAnnulusVariables[1] = annulusPressure;
                XVectorAnnulusVariables[2] = annulusTemperature;
                
                XVectorStringVariables[0] = annulusDensity;
                XVectorStringVariables[1] = pumpPressure;
                XVectorStringVariables[2] = drillStringTemperature;
                
                AnnulusDensity[0] = annulusDensity;
                AnnulusTemperature[0] = annulusTemperature;
                AnnulusPressure[0] = annulusPressure;

                StringDensity[0] = annulusDensity;
                StringTemperature[0] = drillStringTemperature;
                StringPressure[0] = pumpPressure;

                double dX;                
                for (int i = 1; i < depthIntegrationProfile.Count; i++)
                {
                    dX = i > 1 ? depthIntegrationProfile[i - 1] - depthIntegrationProfile[i - 2] : depthIntegrationProfile[0]; 
                    currentDepth = depthIntegrationProfile[i - 1];
                    currentTemparetureGratient = EstimateTemperatureGradient(currentDepth);
           
                    //Get the next step
                    XVectorAnnulusVariables = odeSolver.Solve(XVectorAnnulusVariables, PVTOrdinaryDifferentialEquation, currentDepth, dX);
                    AnnulusDensity[i] = XVectorAnnulusVariables[0];
                    AnnulusPressure[i] = XVectorAnnulusVariables[1];                
                    AnnulusTemperature[i] = XVectorAnnulusVariables[2];
                    //Get the next step
                    XVectorStringVariables = odeSolver.Solve(XVectorStringVariables, PVTOrdinaryDifferentialEquation, currentDepth, dX);
                    StringDensity[i] = XVectorStringVariables[0];
                    StringPressure[i] = XVectorStringVariables[1];                
                    StringTemperature[i] = XVectorStringVariables[2];
                }      
            }
        }
        private double EstimateTemperatureGradient(double currentDepth)
        {
            if (geothermalDataList != null)
            {
                (GeothermalData, GeothermalData)? bounds = GetGeothermalDataBounds(currentDepth);
                if (bounds.HasValue)
                {
                    var (lower, upper) = bounds.Value;
                    double lowerDepth = lower.VerticalDepth ?? currentDepth;
                    double upperDepth = upper.VerticalDepth ?? currentDepth;
                    double lowerGradient = lower.TemperatureGradient ?? currentTemparetureGratient;
                    double upperGradient = upper.TemperatureGradient ?? currentTemparetureGratient;
                    return (upperDepth != lowerDepth)
                        ? lowerGradient + (upperGradient - lowerGradient) * (currentDepth - lowerDepth) / (upperDepth - lowerDepth)
                        : lowerGradient;
                }
                else 
                {   
                    return 0.03;
                }
            }
            else
            {
                return 0.03;             
            }                
        }    
        private (GeothermalData Lower, GeothermalData Upper)? GetGeothermalDataBounds(double depth)
        {
            if (geothermalDataList == null || geothermalDataList.Count == 0)
            {
                return null;
            }

            // If the requested depth is outside the range of available geothermal points, use the closest available point.
            if (depth <= geothermalDataList[0].VerticalDepth)
            {
                return (geothermalDataList[0], geothermalDataList[0]);
            }

            var lastIndex = geothermalDataList.Count - 1;
            if (depth >= geothermalDataList[lastIndex].VerticalDepth)
            {
                return (geothermalDataList[lastIndex], geothermalDataList[lastIndex]);
            }

            // Find the two points that bracket the desired depth.
            for (int i = 1; i < geothermalDataList.Count; i++)
            {
                var upper = geothermalDataList[i];
                var lower = geothermalDataList[i - 1];
                if (upper.VerticalDepth >= depth)
                {
                    return (lower, upper);
                }
            }

            return (geothermalDataList[lastIndex], geothermalDataList[lastIndex]);
        }

        private FluidPVTParameters? CalculateFluidPVTParameters(
            BrinePVTParameters brinePVTParameters, 
                BaseOilPVTParameters baseOilPVTParameters,             
                double? brineMassFraction,
                double? brineMass,
                double? baseOilMassFraction,
                double? baseOilMass
            )
        {
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
                double baseOilVolume = (double) baseOilMass / (double) baseOilMassFraction;

                double ABaseOil = (double) baseOilPVTParameters.A0.GaussianValue.Mean;
                double BBaseOil = (double) baseOilPVTParameters.B0.GaussianValue.Mean;
                double CBaseOil = (double) baseOilPVTParameters.C0.GaussianValue.Mean;
                double DBaseOil = (double) baseOilPVTParameters.D0.GaussianValue.Mean;
                double EBaseOil = (double) baseOilPVTParameters.E0.GaussianValue.Mean;
                double FBaseOil = (double) baseOilPVTParameters.F0.GaussianValue.Mean;

                double brineVolume = (double) brineMass / (double) brineMassFraction;
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
                double ratioOil = baseOilVolume / ( baseOilVolume + brineVolume );
                double ratioBrine = 1 - ratioOil;
            
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
        private Vector<double> PVTOrdinaryDifferentialEquation(double xStep, Vector<double> Xinputs)
        {
            // Xinputs = [density; pressure; temperature]
            Vector<double> RightSideOfTheODE = Vector<double>.Build.Dense(3);
            // RightSideOfTheODE = |    0    |
            //                     | density |
            //                     |  0.03   | degree Celsius/m
            //
            RightSideOfTheODE[0] = 0.0;
            RightSideOfTheODE[1] = Constants.GravitationalAcceleration * Xinputs[0];
            RightSideOfTheODE[2] = currentTemparetureGratient;            
            Matrix<double> CoefficientMatrix = Matrix<double>.Build.Dense(3 , 3);
            // 1st column
            CoefficientMatrix[0, 0] = -1;
            CoefficientMatrix[1, 0] =  0;
            CoefficientMatrix[2, 0] =  0;
            // 2nd Column
            CoefficientMatrix[0, 1] = 2*Xinputs[1] * (FPvtCoeff * Xinputs[2] + EPvtCoeff) + DPvtCoeff * Xinputs[2] + CPvtCoeff;
            CoefficientMatrix[1, 1] =  1;
            CoefficientMatrix[2, 1] =  0;
            // 3rd Column
            CoefficientMatrix[0, 2] = FPvtCoeff * Xinputs[1] * Xinputs[1] + DPvtCoeff * Xinputs[1] + BPvtCoeff;
            CoefficientMatrix[1, 2] =  0;
            CoefficientMatrix[2, 2] =  1;
            // The determinant of CoefficientMatrix should always be -1, and it should always be inversible!
            Matrix<double> InvCoefficientMatrix = CoefficientMatrix.Inverse();
            return InvCoefficientMatrix * RightSideOfTheODE;
        }
        public void UpdateBuoyancy(in LumpedCells lumpedCell, in SimulatorTrajectory trajectory, in SimulatorDrillString drillString, bool useBuoyancyFactor)
        {
         
            // Compute hydrostatic pressures
            //HydrostaticStringPressure = CumulativeTrapezoidal(ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth), Constants.GravitationalAcceleration * StringDensity);
            //HydrostaticAnnulusPressure = CumulativeTrapezoidal(ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth), Constants.GravitationalAcceleration * AnnulusDensity);
            double tempStringPressure = 0;
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
            Vector<double> extendedDepth = ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth);

            for (int i = 1; i < lumpedCell.CumulativeElementLength.Count; i++)
            {
                depth = i == 0 ? 0 : trajectory.InterpolatedVerticalDepth[i - 1];
                //Integrate the density to get the current pressure 
                tempStringPressure += 0.5 * Constants.GravitationalAcceleration * (StringDensity[i] + StringDensity[i - 1]) * (extendedDepth[i] - extendedDepth[i-1]);
                tempAnnulusPressure += 0.5 * Constants.GravitationalAcceleration * (AnnulusDensity[i] + AnnulusDensity[i - 1]) * (extendedDepth[i] - extendedDepth[i-1]);
                HydrostaticStringPressure[i] = tempStringPressure;
                HydrostaticAnnulusPressure[i] = tempAnnulusPressure;    
            }

            for (int i = 0; i < lumpedCell.CumulativeElementLength.Count; i++)
            {   
                //Extract variables for computations assuming the first element used the same as the first one available 
                elementInnerArea = i == 0 ? drillString.ElementInnerArea[0] : drillString.ElementInnerArea[i - 1];
                elementOuterArea = i == 0 ? drillString.ElementOuterArea[0] : drillString.ElementOuterArea[i - 1];
                pipeElementArea = i == 0 ? drillString.ElementArea[0] : drillString.ElementArea[i - 1];
                toolJointElementInnerArea = i == 0 ? drillString.ElementToolJointInnerArea[0] : drillString.ElementToolJointInnerArea[i - 1];
                toolJointElementOuterArea = i == 0 ? drillString.ElementToolJointOuterArea[0] : drillString.ElementToolJointOuterArea[i - 1];                    
                // Stress variation
                interpolatedTheta = (i == 0) ? 0 : trajectory.InterpolatedTheta[i - 1];
                dSigmaDx[i] = BuoyantWeightPerLength[i] * Math.Cos(interpolatedTheta);            
                if (useBuoyancyFactor)
                {
                    buoyancyFactor = (elementOuterArea * (1 - AnnulusDensity[i] / drillString.SteelDensity) -
                                         elementInnerArea * (1 - StringDensity[i] / drillString.SteelDensity)) /
                                         (elementOuterArea - elementInnerArea);
                    BuoyantWeightPerLength[i] = Constants.GravitationalAcceleration * buoyancyFactor * drillString.SteelDensity * pipeElementArea * drillString.WeightCorrectionFactor[i];            

                    //Calculate the variation of pressure due to change in diameters
                    double hydrostaticAnnularCurrent = HydrostaticAnnulusPressure[i];
                    double hydrostaticStringCurrent = HydrostaticStringPressure[i];
                    double hydrostaticAnnularPrev = i == 0 ? 0 : HydrostaticAnnulusPressure[i - 1];
                    double hydrostaticStringPrev = i == 0 ? 0 : HydrostaticStringPressure[i - 1];
                    // out-of-phase tool-joint areas
                    double AtjoPrev = i == 0 ? 0 : drillString.ElementToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : drillString.ElementToolJointInnerArea[i - 1];
                    //populate matrices
                    AxialBuoyancyForceChangeOfDiameters[i] = elementOuterArea * hydrostaticAnnularCurrent
                                                             - AtjoPrev * hydrostaticAnnularPrev
                                                             - elementInnerArea * hydrostaticStringCurrent
                                                             + AtjiPrev * hydrostaticStringPrev;

                    NormalBuoyancyForceChangeOfDiameters[i] = AtjoPrev * hydrostaticAnnularPrev - AtjiPrev * hydrostaticStringPrev;

                }
                else
                {
                    MassPerLength = drillString.SteelDensity * pipeElementArea * drillString.WeightCorrectionFactor[i];
                    BuoyantWeightPerLength[i] = (MassPerLength +
                             (toolJointElementInnerArea * drillString.ToolJointLength * StringDensity[i] +
                               elementInnerArea * (lumpedCell.DistanceBetweenElements - drillString.ToolJointLength) * StringDensity[i] -
                               toolJointElementOuterArea * drillString.ToolJointLength * AnnulusDensity[i] -
                               elementOuterArea * (lumpedCell.DistanceBetweenElements - drillString.ToolJointLength) * AnnulusDensity[i]) /
                               lumpedCell.DistanceBetweenElements) * Constants.GravitationalAcceleration;
                    AxialBuoyancyForceChangeOfDiameters[i] = 0;
                    NormalBuoyancyForceChangeOfDiameters[i] = 0;
                }
            }

        }
    }
}

