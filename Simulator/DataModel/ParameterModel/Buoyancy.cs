using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Buoyancy
    {
        private Vector<double> stringMD_prof;
        private Vector<double> stringPressure_prof;
        private Vector<double> stringDensity_prof;
        private Vector<double> annulusMD_prof;
        private Vector<double> annularPressure_prof;
        private Vector<double> annularDensity_prof;
        public Vector<double> stringPressure;
        public Vector<double> stringDensity;
        public Vector<double> annularPressure;
        public Vector<double> annularDensity;
        public Vector<double> hydrostaticStringPressure;
        public Vector<double> hydrostaticAnnularPressure;
        public Vector<double> Wb;                                   // [N/m] Buoyant drill string weight per length
        public Vector<double> dsigma_dx;                            // [N/m] Tension per length
        public Vector<double> axialBuoyancyForceChangeOfDiameters;
        public Vector<double> normalBuoyancyForceChangeOfDiameters;

        public Buoyancy(in LumpedCells lc, in Trajectory t, in Drillstring ds, string stringPressureFilename, string annulusPressureFilename, bool useBuoyancyFactor)
        {
            // Read the string and annulus data files
            var stringData = ReadTextFile(stringPressureFilename);

            int rowsStringData = stringData.GetLength(0);
            stringMD_prof = Reverse(GetColumn(stringData, 0));
            stringPressure_prof = Reverse(1E5 * GetColumn(stringData, 1));
            stringDensity_prof = Reverse(GetColumn(stringData, 2)); ;

            var annulusData = ReadTextFile(annulusPressureFilename);

            int rowsAnnulusData = annulusData.GetLength(0);
            annulusMD_prof = GetColumn(annulusData, 0);
            annularPressure_prof = 1E5 * GetColumn(annulusData, 1);
            annularDensity_prof = GetColumn(annulusData, 2);

            UpdateBuoyancy(lc, t, ds, useBuoyancyFactor);

            // Calculate soft string normal force
            Vector<double> drag = Vector<double>.Build.Dense(lc.NL + 1);
            Vector<double> tension = Vector<double>.Build.Dense(lc.NL + 1);

            var cumTrapz = CumTrapz(lc.xL, Reverse(dsigma_dx));

            tension = Reverse(cumTrapz) + axialBuoyancyForceChangeOfDiameters - drag;

            Vector<double> phiVec_dote = ExtendVectorStart(0, t.phiVec_dot);
            Vector<double> thetaVec_dote = ExtendVectorStart(0, t.thetaVec_dot);
            Vector<double> thetaVece = ExtendVectorStart(0, t.thetaVec);
        }

        public void UpdateBuoyancy(in LumpedCells lc, in Trajectory t, in Drillstring ds, bool useBuoyancyFactor)
        {
            // Interpolate pressures and densities at positions xL
            stringPressure = LinearInterpolate(stringMD_prof, stringPressure_prof, lc.xL);
            stringDensity = LinearInterpolate(stringMD_prof, stringDensity_prof, lc.xL);
            annularPressure = LinearInterpolate(annulusMD_prof, annularPressure_prof, lc.xL);
            annularDensity = LinearInterpolate(annulusMD_prof, annularDensity_prof, lc.xL);

            // Compute hydrostatic pressures
            hydrostaticStringPressure = CumTrapz(ExtendVectorStart(0, t.TVDVec), Constants.g * stringDensity);
            hydrostaticAnnularPressure = CumTrapz(ExtendVectorStart(0, t.TVDVec), Constants.g * annularDensity);

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
                    buoyancyFactor[i] = (Aoe[i] * (1 - annularDensity[i] / ds.SteelDensity) -
                                         Aie[i] * (1 - stringDensity[i] / ds.SteelDensity)) /
                                         (Aoe[i] - Aie[i]);
                }

                Wb = Vector<double>.Build.Dense(Aie.Length);
                for (int i = 0; i < Aie.Length; i++)
                {
                    Wb[i] = Constants.g * buoyancyFactor[i] * ds.SteelDensity * Ae[i] * ds.WeightCorrectionFactor[i];
                }
            }
            else
            {
                double[] mass_per_length = new double[Ae.Length];
                for (int i = 0; i < Ae.Length; i++)
                {
                    mass_per_length[i] = ds.SteelDensity * Ae[i] * ds.WeightCorrectionFactor[i];
                }

                Wb = Vector<double>.Build.Dense(mass_per_length.Length);
                for (int i = 0; i < mass_per_length.Length; i++)
                {
                    Wb[i] = (mass_per_length[i] +
                             (Atjie[i] * ds.ToolJointLength * stringDensity[i] +
                               Aie[i] * (lc.dxL - ds.ToolJointLength) * stringDensity[i] -
                               Atjoe[i] * ds.ToolJointLength * annularDensity[i] -
                               Aoe[i] * (lc.dxL - ds.ToolJointLength) * annularDensity[i]) /
                               lc.dxL) * Constants.g;
                }
            }

            double[] thetaVece = new double[t.thetaVec.Count() + 1];
            thetaVece[0] = 0;
            Array.Copy(t.thetaVec.ToArray(), 0, thetaVece, 1, t.thetaVec.Count());

            // Compute the tension per length
            dsigma_dx = Vector<double>.Build.Dense(Wb.Count);
            for (int i = 0; i < Wb.Count; i++)
            {
                dsigma_dx[i] = Wb[i] * Math.Cos(thetaVece[i]);
            }

            // Axial and normal buoyancy force changes if buoyancy factor is not used
            if (!useBuoyancyFactor)
            {
                Vector<double> hydrostaticAnnularPressureShifted = ExtendVectorStart(0, hydrostaticAnnularPressure);
                Vector<double> hydrostaticStringPressureShifted = ExtendVectorStart(0, hydrostaticStringPressure);

                // Compute axial buoyancy force change of diameters
                axialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(hydrostaticAnnularPressure.Count);
                normalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(hydrostaticAnnularPressure.Count);

                for (int i = 0; i < hydrostaticAnnularPressure.Count; i++)
                {
                    double AoCurrent = i == 0 ? ds.OuterArea[0] : ds.OuterArea[i - 1];
                    double AiCurrent = i == 0 ? ds.InnerArea[0] : ds.InnerArea[i - 1];
                    double hydrostaticAnnularCurrent = hydrostaticAnnularPressure[i];
                    double hydrostaticStringCurrent = hydrostaticStringPressure[i];

                    double AtjoPrev = i == 0 ? 0 : ds.ToolJointOuterArea[i - 1];
                    double AtjiPrev = i == 0 ? 0 : ds.ToolJointInnerArea[i - 1];
                    double hydrostaticAnnularPrev = i == 0 ? 0 : hydrostaticAnnularPressure[i - 1];
                    double hydrostaticStringPrev = i == 0 ? 0 : hydrostaticStringPressure[i - 1];

                    axialBuoyancyForceChangeOfDiameters[i] = AoCurrent * hydrostaticAnnularCurrent
                                                             - AtjoPrev * hydrostaticAnnularPrev
                                                             - AiCurrent * hydrostaticStringCurrent
                                                             + AtjiPrev * hydrostaticStringPrev;

                    normalBuoyancyForceChangeOfDiameters[i] = AtjoPrev * hydrostaticAnnularPrev - AtjiPrev * hydrostaticStringPrev;
                }
            }
            else
            {
                // If buoyancy factor is used, axial and normal components are set to zero
                axialBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(hydrostaticAnnularPressure.Count);
                normalBuoyancyForceChangeOfDiameters = Vector<double>.Build.Dense(hydrostaticAnnularPressure.Count);
            }
        }
    }
}

