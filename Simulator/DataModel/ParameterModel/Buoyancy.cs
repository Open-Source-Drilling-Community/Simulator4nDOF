using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.ModelShared;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Buoyancy
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
        private double APvtCoeff;
        private double BPvtCoeff;
        private double CPvtCoeff;
        private double DPvtCoeff;
        private double EPvtCoeff;
        private double FPvtCoeff;
        private Vector<double> XVectorAnnulusVariables;
        private Vector<double> XVectorStringVariables;
                
        public Buoyancy(in LumpedCells lumpedCells, in SimulatorTrajectory trajectory, in SimulatorDrillString drillString,
            DrillingFluidDescription drillingFluidDescription, double fluidDensity, bool useBuoyancyFactor, double surfacePressure)
        {
            AnnulusDensity = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            AnnulusPressure = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            AnnulusTemperature = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            StringDensity = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            StringPressure = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            StringTemperature = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);

            HydrostaticStringPressure = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            HydrostaticAnnulusPressure = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);

            XVectorAnnulusVariables = Vector<double>.Build.Dense(3);
            XVectorStringVariables = Vector<double>.Build.Dense(3);
            
            BuoyantWeightPerLength = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);  
            AxialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
            NormalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);

            dSigmaDx = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count);
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
                fluidDensity, 
                drillingFluidDescription.ReferenceTemperature.GaussianValue.Mean,
                surfacePressure,
                lumpedCells.ElementLength);   
            UpdateBuoyancy(lumpedCells, trajectory, drillString, useBuoyancyFactor);
        }
        private void IntegratePressureProfile(
            FluidPVTParameters fluidPVTParameters, 
            double? densityNullable, 
            double? temperatureNullable, 
            double surfacePressure,
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
                double density = (double)densityNullable;
                double annulusTemperature = (double)temperatureNullable;
                //  Initial pressure in the annulus can be calculated with Bhaskara. 
                // One answer will always be negative as b > 0 and Delta > 0. 
                // We calculate only the one that will be positive
                double a = EPvtCoeff + FPvtCoeff * annulusTemperature;
                double b = DPvtCoeff * annulusTemperature + CPvtCoeff;
                double c = APvtCoeff + BPvtCoeff * annulusTemperature - density;
                double Delta = b * b - 4 * a * c;
                if (Delta < 0)
                {
                    throw new ArgumentException("Initial pressure could not be computed!");
                }
                double annulusPressure = (-b + Math.Sqrt(Delta)) / (2.0 * a); 
                //The Drill-string initial temperature is calculated as:
                double drillStringTemperature = (
                    density - EPvtCoeff * surfacePressure * surfacePressure - CPvtCoeff * surfacePressure - APvtCoeff)/
                    (FPvtCoeff * surfacePressure * surfacePressure + DPvtCoeff * surfacePressure + BPvtCoeff);

                //XVector is the solver-ready format of the variables
                XVectorAnnulusVariables[0] = density;
                XVectorAnnulusVariables[1] = annulusPressure;
                XVectorAnnulusVariables[2] = annulusTemperature;
                
                XVectorStringVariables[0] = density;
                XVectorStringVariables[1] = surfacePressure;
                XVectorStringVariables[2] = drillStringTemperature;
                
                AnnulusDensity[0] = density;
                AnnulusTemperature[0] = annulusTemperature;
                AnnulusPressure[0] = annulusPressure;

                StringDensity[0] = density;
                StringTemperature[0] = drillStringTemperature;
                StringPressure[0] = surfacePressure;

                double dX;                
                for (int i = 1; i < depthIntegrationProfile.Count; i++)
                {
                    dX = i > 1 ? depthIntegrationProfile[i - 1] - depthIntegrationProfile[i - 2] : depthIntegrationProfile[0]; 
                    currentDepth = depthIntegrationProfile[i - 1];
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
            RightSideOfTheODE[2] = 0.03;            
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

            for (int i = 1; i < lumpedCell.ElementLength.Count; i++)
            {
                depth = i == 0 ? 0 : trajectory.InterpolatedVerticalDepth[i - 1];
                //Integrate the density to get the current pressure 
                tempStringPressure += 0.5 * Constants.GravitationalAcceleration * (StringDensity[i] + StringDensity[i - 1]) * (extendedDepth[i] - extendedDepth[i-1]);
                tempAnnulusPressure += 0.5 * Constants.GravitationalAcceleration * (AnnulusDensity[i] + AnnulusDensity[i - 1]) * (extendedDepth[i] - extendedDepth[i-1]);
                HydrostaticStringPressure[i] = tempStringPressure;
                HydrostaticAnnulusPressure[i] = tempAnnulusPressure;    
            }

            for (int i = 0; i < lumpedCell.ElementLength.Count; i++)
            {   
                //Extract variables for computations assuming the first element used the same as the first one available 
                elementInnerArea = i == 0 ? drillString.InnerArea[0] : drillString.InnerArea[i - 1];
                elementOuterArea = i == 0 ? drillString.OuterArea[0] : drillString.OuterArea[i - 1];
                pipeElementArea = i == 0 ? drillString.PipeArea[0] : drillString.PipeArea[i - 1];
                toolJointElementInnerArea = i == 0 ? drillString.ToolJointInnerArea[0] : drillString.ToolJointInnerArea[i - 1];
                toolJointElementOuterArea = i == 0 ? drillString.ToolJointOuterArea[0] : drillString.ToolJointOuterArea[i - 1];                    
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
                    double AtjoPrev = i == 0 ? 0 : drillString.ToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : drillString.ToolJointInnerArea[i - 1];
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

