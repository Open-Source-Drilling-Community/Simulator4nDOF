using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulatorWellbore
    {
        // Move topdrive
        public readonly double WallStiffness = 5.0e7;                 // [N.s/m] coefficient of damping at wellbore wall
        public readonly double WallDamping = 5.0e4;                   // percent of mass imbalance compared to total mass

        // Geometry to be configured
        private List<SimulatorBoreHole> boreHoleSizes {get; set;}
        public Vector<double> DrillStringClearance;            // [m] drillstring radial clearance to the borehole wall

        private Vector<double> boreholeRadius;                   // [m] Wellbore radius calculation

        public SimulatorWellbore(in SimulatorDrillString drillString,
                        in LumpedCells lumpedCells,
                        ModelShared.CasingSection casingSection
                        )
        {
            
            List<SimulatorBoreHole> boreHoleSizes = new();
            double depth = casingSection.TopDepth.GaussianValue.Mean ?? 0;
            foreach (BoreHoleSize boreHoleSize in casingSection.CasingSectionSizeTable)
            {
                boreHoleSizes.Add(new SimulatorBoreHole{
                        Depth = (double ) depth,
                        Diameter = (double) boreHoleSize.HoleSize.GaussianValue.Mean!,
                        Length  = (double) boreHoleSize.Length.GaussianValue.Mean!  
                    }
                );
                depth += boreHoleSize.Length.GaussianValue.Mean ?? 0.0;


            }




            depth = 0;
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
            boreholeRadius = Vector<double>.Build.Dense(drillString.OuterRadius.Count);
            DrillStringClearance = Vector<double>.Build.Dense(drillString.OuterRadius.Count);

            UpdateWellbore(drillString, lumpedCells);
        }

        public void UpdateWellbore(in SimulatorDrillString drillString, in LumpedCells lumpedElement)
        {
            // Wellbore radius calculation
            boreholeRadius = Vector<double>.Build.Dense(drillString.OuterRadius.Count);
            int idx = 0;
            for (int i = 1; i < lumpedElement.ElementLength.Count(); i++)
            {
                if (idx < boreHoleSizes.Count)
                {
                    if (lumpedElement.ElementLength[i] <= boreHoleSizes[idx].Depth)
                    {
                        boreholeRadius[i - 1] = 0.5 * boreHoleSizes[idx].Diameter;
                    }
                    else
                    {
                        while (idx < boreHoleSizes.Count)
                        {
                            idx++;
                            if (idx < boreHoleSizes.Count && lumpedElement.ElementLength[i] <= boreHoleSizes[idx].Depth)
                            {
                                boreholeRadius[i - 1] = boreHoleSizes[idx].Diameter / 2;
                                break;
                            }
                        }
                    }
                }
                if (idx >= boreHoleSizes.Count)
                {
                    boreholeRadius[i - 1] = drillString.BitRadius;
                }
            }

            DrillStringClearance = boreholeRadius - drillString.OuterRadius;
            foreach (int i in drillString.SleeveIndexPosition)
                DrillStringClearance[i] = boreholeRadius[i] - drillString.SleeveOuterRadius;
        }
    }
}
 