using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator;

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
        public Vector<double> AnnularPressure;
        public Vector<double> AnnularDensity;
        public Vector<double> HydrostaticStringPressure;
        public Vector<double> HydrostaticAnnularPressure;
        public Vector<double> BuoyantWeightPerLength;                                   // [N/m] Buoyant drill string weight per length
        public Vector<double> dSigmaDx;                            // [N/m] Tension per length
        public Vector<double> AxialBuoyancyForceChangeOfDiameters;
        public Vector<double> NormalBuoyancyForceChangeOfDiameters;

        public Buoyancy(in LumpedCells lc, in Trajectory t, in Drillstring ds, string stringPressureFilename, string annulusPressureFilename, bool useBuoyancyFactor)
        {
            // Read the string and annulus data files
            var stringData = ReadTextFile(stringPressureFilename);

            int rowsStringData = stringData.GetLength(0);
            StringMDProfile = Reverse(GetColumn(stringData, 0));
            StringPressureProf = Reverse(1E5 * GetColumn(stringData, 1));
            StringDensityProfile = Reverse(GetColumn(stringData, 2)); ;

            var annulusData = ReadTextFile(annulusPressureFilename);

            int rowsAnnulusData = annulusData.GetLength(0);
            AnnulusMDProfile = GetColumn(annulusData, 0);
            AnnularPressureProfile = 1E5 * GetColumn(annulusData, 1);
            AnnularDensityProfile = GetColumn(annulusData, 2);

            UpdateBuoyancy(lc, t, ds, useBuoyancyFactor);

            // Calculate soft string normal force
            Vector<double> drag = Vector<double>.Build.Dense(lc.NumberOfLumpedElements + 1);
            Vector<double> tension = Vector<double>.Build.Dense(lc.NumberOfLumpedElements + 1);

            var cumTrapz = CummulativeTrapezoidal(lc.ElementLength, Reverse(dSigmaDx));

            tension = Reverse(cumTrapz) + AxialBuoyancyForceChangeOfDiameters - drag;

            Vector<double> phiVec_dote = ExtendVectorStart(0, t.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, t.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, t.thetaVec);
        }

        public void UpdateBuoyancy(in LumpedCells lc, in Trajectory t, in Drillstring ds, bool useBuoyancyFactor)
        {
            // Interpolate pressures and densities at positions xL
            StringPressure = LinearInterpolate(StringMDProfile, StringPressureProf, lc.ElementLength);
            StringDensity = LinearInterpolate(StringMDProfile, StringDensityProfile, lc.ElementLength);
            AnnularPressure = LinearInterpolate(AnnulusMDProfile, AnnularPressureProfile, lc.ElementLength);
            AnnularDensity = LinearInterpolate(AnnulusMDProfile, AnnularDensityProfile, lc.ElementLength);

            // Compute hydrostatic pressures
            HydrostaticStringPressure = CummulativeTrapezoidal(ExtendVectorStart(0, t.TVDVec), Constants.GravitationalAcceleration * StringDensity);
            HydrostaticAnnularPressure = CummulativeTrapezoidal(ExtendVectorStart(0, t.TVDVec), Constants.GravitationalAcceleration * AnnularDensity);

            // Calculate buoyant weight using the appropriate method
            double[] Aie = new double[ds.InnerArea.Count() + 1];
            Aie[0] = ds.InnerArea[0];
            Array.Copy(ds.InnerArea.ToArray(), 0, Aie, 1, ds.InnerArea.Count());

            double[] Aoe = new double[ds.OuterArea.Count() + 1];
            Aoe[0] = ds.OuterArea[0];
            Array.Copy(ds.OuterArea.ToArray(), 0, Aoe, 1, ds.OuterArea.Count());

            double[] Atjie = new double[ds.ToolJointInnerArea.Count() + 1];
            Atjie[0] = ds.ToolJointInnerArea[0];
            Array.Copy(ds.ToolJointInnerArea.ToArray(), 0, Atjie, 1, ds.ToolJointInnerArea.Count());

            double[] Atjoe = new double[ds.ToolJointOuterArea.Count() + 1];
            Atjoe[0] = ds.ToolJointOuterArea[0];
            Array.Copy(ds.ToolJointOuterArea.ToArray(), 0, Atjoe, 1, ds.ToolJointOuterArea.Count());

            double[] Ae = new double[ds.PipeArea.Count() + 1];
            Ae[0] = ds.PipeArea[0];
            Array.Copy(ds.PipeArea.ToArray(), 0, Ae, 1, ds.PipeArea.Count());

            if (useBuoyancyFactor)
            {
                double[] buoyancyFactor = new double[Aoe.Length];
                for (int i = 0; i < Aoe.Length; i++)
                {
                    buoyancyFactor[i] = (Aoe[i] * (1 - AnnularDensity[i] / ds.SteelDensity) -
                                         Aie[i] * (1 - StringDensity[i] / ds.SteelDensity)) /
                                         (Aoe[i] - Aie[i]);
                }

                BuoyantWeightPerLength = Vector<double>.Build.Dense(Aie.Length);
                for (int i = 0; i < Aie.Length; i++)
                {
                    BuoyantWeightPerLength[i] = Constants.GravitationalAcceleration * buoyancyFactor[i] * ds.SteelDensity * Ae[i] * ds.WeightCorrectionFactor[i];
                }
            }
            else
            {
                double[] mass_per_length = new double[Ae.Length];
                for (int i = 0; i < Ae.Length; i++)
                {
                    mass_per_length[i] = ds.SteelDensity * Ae[i] * ds.WeightCorrectionFactor[i];
                }

                BuoyantWeightPerLength = Vector<double>.Build.Dense(mass_per_length.Length);
                for (int i = 0; i < mass_per_length.Length; i++)
                {
                    BuoyantWeightPerLength[i] = (mass_per_length[i] +
                             (Atjie[i] * ds.ToolJointLength * StringDensity[i] +
                               Aie[i] * (lc.DistanceBetweenElements - ds.ToolJointLength) * StringDensity[i] -
                               Atjoe[i] * ds.ToolJointLength * AnnularDensity[i] -
                               Aoe[i] * (lc.DistanceBetweenElements - ds.ToolJointLength) * AnnularDensity[i]) /
                               lc.DistanceBetweenElements) * Constants.GravitationalAcceleration;
                }
            }

            double[] thetaVece = new double[t.thetaVec.Count() + 1];
            thetaVece[0] = 0;
            Array.Copy(t.thetaVec.ToArray(), 0, thetaVece, 1, t.thetaVec.Count());

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
                    double AoCurrent = i == 0 ? ds.OuterArea[0] : ds.OuterArea[i - 1];
                    double AiCurrent = i == 0 ? ds.InnerArea[0] : ds.InnerArea[i - 1];
                    double hydrostaticAnnularCurrent = HydrostaticAnnularPressure[i];
                    double hydrostaticStringCurrent = HydrostaticStringPressure[i];

                    double AtjoPrev = i == 0 ? 0 : ds.ToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : ds.ToolJointInnerArea[i - 1];
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

