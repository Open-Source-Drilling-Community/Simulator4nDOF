using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Wellbore
    {
        // Move topdrive
        public readonly double TopDriveInertia = 2900;                     // [kg.m^2] Top drive mass moment of inertia
        public readonly double FluidDampingCoefficient = 3000.0;                     // [N.s/m] Fluid damping coefficient for lateral dynamics
        public readonly double WallStiffness = 5.0e7;                      // [N.s/m] coefficient of damping at wellbore wall
        public readonly double WallDamping = 5.0e4;                      // percent of mass imbalance compared to total mass

        // Geometry to be configured
        private List<BoreHoleSize> boreHoleSizes =
            new List<BoreHoleSize>() {
                new BoreHoleSize(){Depth = 120, ID = 21 * Constants.InchToMeterConversion },
                new BoreHoleSize(){Depth = 2181, ID = 9.0*Constants.InchToMeterConversion } 
            };
        public Vector<double> BoreholeRadius;                   // [m] Wellbore radius calculation
        public Vector<double> DrillStringClearance;            // [m] drillstring radial clearance to the borehole wall

        public Wellbore(in Drillstring drillString,
                        in LumpedCells lumpedCells,
                        List<BoreHoleSize> boreHoleSizes,
                        double topDriveMomentOfInertia,
                        double fluidDamping)
        {
            TopDriveInertia = topDriveMomentOfInertia;
            FluidDampingCoefficient = fluidDamping;

            this.boreHoleSizes = boreHoleSizes; 
            BoreholeRadius = Vector<double>.Build.Dense(drillString.OuterRadius.Count);
            DrillStringClearance = Vector<double>.Build.Dense(drillString.OuterRadius.Count);

            UpdateWellbore(drillString, lumpedCells);
        }

        public void UpdateWellbore(in Drillstring drillString, in LumpedCells lumpedElement)
        {
            // Wellbore radius calculation
            BoreholeRadius = Vector<double>.Build.Dense(drillString.OuterRadius.Count);
            int idx = 0;
            for (int i = 1; i < lumpedElement.ElementLength.Count(); i++)
            {
                if (idx < boreHoleSizes.Count)
                {
                    if (lumpedElement.ElementLength[i] <= boreHoleSizes[idx].Depth)
                    {
                        BoreholeRadius[i - 1] = boreHoleSizes[idx].ID / 2;
                    }
                    else
                    {
                        while (idx < boreHoleSizes.Count)
                        {
                            idx++;
                            if (idx < boreHoleSizes.Count && lumpedElement.ElementLength[i] <= boreHoleSizes[idx].Depth)
                            {
                                BoreholeRadius[i - 1] = boreHoleSizes[idx].ID / 2;
                                break;
                            }
                        }
                    }
                }
                if (idx >= boreHoleSizes.Count)
                {
                    BoreholeRadius[i - 1] = drillString.BitRadius;
                }
            }

            DrillStringClearance = BoreholeRadius - drillString.OuterRadius;
            foreach (int i in drillString.SleeveIndexPosition)
                DrillStringClearance[i] = BoreholeRadius[i] - drillString.SleeveOuterRadius;
        }
    }
}
