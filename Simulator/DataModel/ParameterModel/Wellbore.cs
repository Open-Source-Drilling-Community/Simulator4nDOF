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
        private List<SimulatorBoreHole> boreHoleSizes {get; set;}
        public Vector<double> BoreholeRadius;                   // [m] Wellbore radius calculation
        public Vector<double> DrillStringClearance;            // [m] drillstring radial clearance to the borehole wall

        public Wellbore(in SimulatorDrillString drillString,
                        in LumpedCells lumpedCells,
                        ModelShared.CasingSection casingSection,
                        double topDriveMomentOfInertia,
                        double fluidDamping)
        {
            TopDriveInertia = topDriveMomentOfInertia;
            FluidDampingCoefficient = fluidDamping;
            List<SimulatorBoreHole> boreHoleSizes = new();
            double depth = 0;
            if(casingSection.TopDepth.GaussianValue.Mean != null)
            {
                depth += (double) casingSection.TopDepth.GaussianValue.Mean;
            }
            foreach (ModelShared.BoreHoleSize bhSize in casingSection.CasingSectionSizeTable)
            {
  

                boreHoleSizes.Add(new SimulatorBoreHole
                    {
                        Depth = (double ) casingSection.TopDepth.GaussianValue.Mean!,
                        Diameter = (double) bhSize.HoleSize.GaussianValue.Mean!,
                        Length  = (double) bhSize.Length.GaussianValue.Mean!  
                    }
                );
                if (bhSize.Length.GaussianValue.Mean != null)
                    depth += (double) bhSize.Length.GaussianValue.Mean;                
            }

            this.boreHoleSizes = boreHoleSizes; 
            BoreholeRadius = Vector<double>.Build.Dense(drillString.OuterRadius.Count);
            DrillStringClearance = Vector<double>.Build.Dense(drillString.OuterRadius.Count);

            UpdateWellbore(drillString, lumpedCells);
        }

        public void UpdateWellbore(in SimulatorDrillString drillString, in LumpedCells lumpedElement)
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
                        BoreholeRadius[i - 1] = boreHoleSizes[idx].Diameter / 2;
                    }
                    else
                    {
                        while (idx < boreHoleSizes.Count)
                        {
                            idx++;
                            if (idx < boreHoleSizes.Count && lumpedElement.ElementLength[i] <= boreHoleSizes[idx].Depth)
                            {
                                BoreholeRadius[i - 1] = boreHoleSizes[idx].Diameter / 2;
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
