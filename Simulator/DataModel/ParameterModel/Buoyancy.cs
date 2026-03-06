using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.ModelShared;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Buoyancy
    {
        private Vector<double> StringMDProfile;
        private Vector<double> StringPressureProf;
        private Vector<double> StringDensityProfile;
        private Vector<double> AnnulusMDProfile;
        private Vector<double> AnnularPressureProfile;
        private Vector<double> AnnularDensityProfile;
        public Vector<double> StringPressure;
        public Vector<double> StringDensity;
        public Vector<double> HydrostaticStringPressure;
        public Vector<double> HydrostaticAnnularPressure;
        public Vector<double> BuoyantWeightPerLength;                                   // [N/m] Buoyant drill string weight per length
        public Vector<double> dSigmaDx;                            // [N/m] Tension per length
        public Vector<double> AxialBuoyancyForceChangeOfDiameters;
        public Vector<double> NormalBuoyancyForceChangeOfDiameters;
        public Vector<double> AnnulusDensity;
        public Vector<double> AnnulusPressure;
        
        private Vector<double> temperatureVector;
        private double APvtCoeff;
        private double BPvtCoeff;
        private double CPvtCoeff;
        private double DPvtCoeff;
        private double EPvtCoeff;
        private double FPvtCoeff;
        private Vector<double> XVector;
        
        public Buoyancy(in LumpedCells lumpedCells, in SimulatorTrajectory trajectory, in SimulatorDrillString drillString,
             DrillingFluidDescription drillingFluidDescription, bool useBuoyancyFactor)
        {
            // Read the string and annulus data files
            //var stringData = ReadTextFile(stringPressureFilename);
            //int rowsStringData = stringData.GetLength(0);
            //StringMDProfile = Reverse(GetColumn(stringData, 0));
            //StringPressureProf = Reverse(1E5 * GetColumn(stringData, 1));
            //StringDensityProfile = Reverse(GetColumn(stringData, 2)); ;
            //var annulusData = ReadTextFile(annulusPressureFilename);
            //int rowsAnnulusData = annulusData.GetLength(0);
            //AnnulusMDProfile = GetColumn(annulusData, 0);
            //AnnularPressureProfile = 1E5 * GetColumn(annulusData, 1);
            //AnnularDensityProfile = GetColumn(annulusData, 2);
            AnnulusDensity = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count + 1);
            temperatureVector = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count + 1);
            AnnulusPressure = Vector<double>.Build.Dense(lumpedCells.ElementLength.Count + 1);
            XVector = Vector<double>.Build.Dense(3);
            if (drillingFluidDescription.FluidPVTParameters == null)
            {
                return;
            }
            else
            {
                IntegratePressureProfile(drillingFluidDescription.FluidPVTParameters, 
                    drillingFluidDescription.FluidMassDensity.GaussianValue.Mean, 
                    drillingFluidDescription.ReferenceTemperature.GaussianValue.Mean,
                    lumpedCells);   
            }
            UpdateBuoyancy(lumpedCells, trajectory, drillString, useBuoyancyFactor);
        }
        private void IntegratePressureProfile(FluidPVTParameters fluidPVTParameters, double? densityNullable, double? temperatureNullable, LumpedCells lumpedCells)
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
                double dX = lumpedCells.ElementLength[0];
                double currentDepth = lumpedCells.ElementLength[0];
                RKF45 odeSolver = new RKF45(dX);
                APvtCoeff = (double) fluidPVTParameters.A0.GaussianValue.Mean;
                BPvtCoeff = (double) fluidPVTParameters.B0.GaussianValue.Mean;
                CPvtCoeff = (double) fluidPVTParameters.C0.GaussianValue.Mean;
                DPvtCoeff = (double) fluidPVTParameters.D0.GaussianValue.Mean;
                EPvtCoeff = (double) fluidPVTParameters.E0.GaussianValue.Mean;
                FPvtCoeff = (double) fluidPVTParameters.F0.GaussianValue.Mean;
                double density = (double)densityNullable;
                double temperature = (double)temperatureNullable;
                //  Initial pressure can be calculated with Bhaskara. One answer will always be negative
                // as b > 0 and Delta > 0. We calculate only the one 
                //that will be positive
                double a = EPvtCoeff + FPvtCoeff * temperature;
                double b = DPvtCoeff * temperature + CPvtCoeff;
                double c = APvtCoeff + BPvtCoeff * temperature - density;
                double Delta = b * b - 4 * a * c;
                if (Delta < 0)
                {
                    throw new ArgumentException("Initial pressure could not be computed!");
                }
                double pressure = (-b + Math.Sqrt(Delta)) / (2.0 * a); 
                //XVector is the solver-ready format of the variables
                
                XVector[0] = density;
                XVector[1] = temperature;
                XVector[2] = pressure;

                AnnulusDensity[0] = density;
                temperatureVector[0] = temperature;
                AnnulusPressure[0] = pressure;
                
                for (int i = 1; i < lumpedCells.ElementLength.Count + 1; i++)
                {
                    dX = lumpedCells.ElementLength[i] - lumpedCells.ElementLength[i - 1]; 
                    currentDepth = lumpedCells.ElementLength[i];
                    //Get the next step
                    XVector = odeSolver.Solve(XVector, PVTOrdinaryDifferentialEquation, currentDepth, dX);
                    AnnulusDensity[i] = XVector[0];
                    temperatureVector[i] = XVector[1];
                    AnnulusPressure[i] = XVector[2];
                }                
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
            RightSideOfTheODE[1] = Xinputs[0];
            RightSideOfTheODE[2] = 0.03;            
            Matrix<double> CoefficientMatrix = Matrix<double>.Build.Dense(3 , 3);
            // 1st column
            CoefficientMatrix[0, 0] = -1;
            CoefficientMatrix[1, 0] =  0;
            CoefficientMatrix[2, 0] =  0;
            // 2nd Column
            CoefficientMatrix[0, 0] = 2*Xinputs[1] * (FPvtCoeff * Xinputs[2] + EPvtCoeff) + DPvtCoeff * Xinputs[2] + CPvtCoeff;
            CoefficientMatrix[1, 0] =  1;
            CoefficientMatrix[2, 0] =  0;
            // 3rd Column
            CoefficientMatrix[0, 0] = FPvtCoeff * Xinputs[1] * Xinputs[1] + DPvtCoeff * Xinputs[1] + BPvtCoeff;
            CoefficientMatrix[1, 0] =  0;
            CoefficientMatrix[2, 0] =  1;
            // The determinant of CoefficientMatrix should always be -1, and it should always be inversible!
            Matrix<double> InvCoefficientMatrix = CoefficientMatrix.Inverse();
            return InvCoefficientMatrix * RightSideOfTheODE;
        }
        public void UpdateBuoyancy(in LumpedCells lumpedCell, in SimulatorTrajectory trajectory, in SimulatorDrillString drillString, bool useBuoyancyFactor)
        {
            // Interpolate pressures and densities at positions xL
            StringPressure = LinearInterpolate(StringMDProfile, StringPressureProf, lumpedCell.ElementLength);
            StringDensity = LinearInterpolate(StringMDProfile, StringDensityProfile, lumpedCell.ElementLength);
            //AnnularPressure = LinearInterpolate(AnnulusMDProfile, AnnularPressureProfile, lumpedCell.ElementLength);
            //AnnularDensity = LinearInterpolate(AnnulusMDProfile, AnnularDensityProfile, lumpedCell.ElementLength);

            // Compute hydrostatic pressures
            HydrostaticStringPressure = CummulativeTrapezoidal(ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth), Constants.GravitationalAcceleration * StringDensity);
            HydrostaticAnnularPressure = CummulativeTrapezoidal(ExtendVectorStart(0, trajectory.InterpolatedVerticalDepth), Constants.GravitationalAcceleration * AnnulusDensity);

            // Calculate buoyant weight using the appropriate method
            double[] elementInnerArea = new double[drillString.InnerArea.Count() + 1];
            elementInnerArea[0] = drillString.InnerArea[0];
            Array.Copy(drillString.InnerArea.ToArray(), 0, elementInnerArea, 1, drillString.InnerArea.Count());

            double[] elementOuterArea = new double[drillString.OuterArea.Count() + 1];
            elementOuterArea[0] = drillString.OuterArea[0];
            Array.Copy(drillString.OuterArea.ToArray(), 0, elementOuterArea, 1, drillString.OuterArea.Count());

            double[] toolJointElementInnerArea = new double[drillString.ToolJointInnerArea.Count() + 1];
            toolJointElementInnerArea[0] = drillString.ToolJointInnerArea[0];
            Array.Copy(drillString.ToolJointInnerArea.ToArray(), 0, toolJointElementInnerArea, 1, drillString.ToolJointInnerArea.Count());

            double[] toolJointElementOuterArea = new double[drillString.ToolJointOuterArea.Count() + 1];
            toolJointElementOuterArea[0] = drillString.ToolJointOuterArea[0];
            Array.Copy(drillString.ToolJointOuterArea.ToArray(), 0, toolJointElementOuterArea, 1, drillString.ToolJointOuterArea.Count());

            double[] pipeElementArea = new double[drillString.PipeArea.Count() + 1];
            pipeElementArea[0] = drillString.PipeArea[0];
            Array.Copy(drillString.PipeArea.ToArray(), 0, pipeElementArea, 1, drillString.PipeArea.Count());

            if (useBuoyancyFactor)
            {
                double[] buoyancyFactor = new double[elementOuterArea.Length];
                for (int i = 0; i < elementOuterArea.Length; i++)
                {
                    buoyancyFactor[i] = (elementOuterArea[i] * (1 - AnnulusDensity[i] / drillString.SteelDensity) -
                                         elementInnerArea[i] * (1 - StringDensity[i] / drillString.SteelDensity)) /
                                         (elementOuterArea[i] - elementInnerArea[i]);
                }

                BuoyantWeightPerLength = Vector<double>.Build.Dense(elementInnerArea.Length);
                for (int i = 0; i < elementInnerArea.Length; i++)
                {
                    BuoyantWeightPerLength[i] = Constants.GravitationalAcceleration * buoyancyFactor[i] * drillString.SteelDensity * pipeElementArea[i] * drillString.WeightCorrectionFactor[i];
                }
            }
            else
            {
                double[] mass_per_length = new double[pipeElementArea.Length];
                for (int i = 0; i < pipeElementArea.Length; i++)
                {
                    mass_per_length[i] = drillString.SteelDensity * pipeElementArea[i] * drillString.WeightCorrectionFactor[i];
                }

                BuoyantWeightPerLength = Vector<double>.Build.Dense(mass_per_length.Length);
                for (int i = 0; i < mass_per_length.Length; i++)
                {
                    BuoyantWeightPerLength[i] = (mass_per_length[i] +
                             (toolJointElementInnerArea[i] * drillString.ToolJointLength * StringDensity[i] +
                               elementInnerArea[i] * (lumpedCell.DistanceBetweenElements - drillString.ToolJointLength) * StringDensity[i] -
                               toolJointElementOuterArea[i] * drillString.ToolJointLength * AnnulusDensity[i] -
                               elementOuterArea[i] * (lumpedCell.DistanceBetweenElements - drillString.ToolJointLength) * AnnulusDensity[i]) /
                               lumpedCell.DistanceBetweenElements) * Constants.GravitationalAcceleration;
                }
            }

            double[] thetaVece = new double[trajectory.InterpolatedTheta.Count() + 1];
            thetaVece[0] = 0;
            Array.Copy(trajectory.InterpolatedTheta.ToArray(), 0, thetaVece, 1, trajectory.InterpolatedTheta.Count());

            // Compute the tension per length
            dSigmaDx = Vector<double>.Build.Dense(BuoyantWeightPerLength.Count);
            for (int i = 0; i < BuoyantWeightPerLength.Count; i++)
            {
                dSigmaDx[i] = BuoyantWeightPerLength[i] * Math.Cos(thetaVece[i]);
            }

            // Axial and normal buoyancy force changes if buoyancy factor is not used
            if (!useBuoyancyFactor)
            {
                Vector<double> hydrostaticAnnularPressureShifted = ExtendVectorStart(0, HydrostaticAnnularPressure);
                Vector<double> hydrostaticStringPressureShifted = ExtendVectorStart(0, HydrostaticStringPressure);

                // Compute axial buoyancy force change of diameters
                AxialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(HydrostaticAnnularPressure.Count);
                NormalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(HydrostaticAnnularPressure.Count);

                for (int i = 0; i < HydrostaticAnnularPressure.Count; i++)
                {
                    double AoCurrent = i == 0 ? drillString.OuterArea[0] : drillString.OuterArea[i - 1];
                    double AiCurrent = i == 0 ? drillString.InnerArea[0] : drillString.InnerArea[i - 1];
                    double hydrostaticAnnularCurrent = HydrostaticAnnularPressure[i];
                    double hydrostaticStringCurrent = HydrostaticStringPressure[i];

                    double AtjoPrev = i == 0 ? 0 : drillString.ToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : drillString.ToolJointInnerArea[i - 1];
                    double hydrostaticAnnularPrev = i == 0 ? 0 : HydrostaticAnnularPressure[i - 1];
                    double hydrostaticStringPrev = i == 0 ? 0 : HydrostaticStringPressure[i - 1];

                    AxialBuoyancyForceChangeOfDiameters[i] = AoCurrent * hydrostaticAnnularCurrent
                                                             - AtjoPrev * hydrostaticAnnularPrev
                                                             - AiCurrent * hydrostaticStringCurrent
                                                             + AtjiPrev * hydrostaticStringPrev;

                    NormalBuoyancyForceChangeOfDiameters[i] = AtjoPrev * hydrostaticAnnularPrev - AtjiPrev * hydrostaticStringPrev;
                }
            }
            else
            {
                // If buoyancy factor is used, axial and normal components are set to zero
                AxialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(HydrostaticAnnularPressure.Count);
                NormalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(HydrostaticAnnularPressure.Count);
            }
        }
    }
}

