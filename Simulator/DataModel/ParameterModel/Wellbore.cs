using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Wellbore
    {
        // Move topdrive
        public readonly double I_TD = 2900;                     // [kg.m^2] Top drive mass moment of inertia
        public readonly double Df = 3000.0;                     // [N.s/m] Fluid damping coefficient for lateral dynamics
        public readonly double kw = 5.0e7;                      // [N.s/m] coefficient of damping at wellbore wall
        public readonly double dw = 5.0e4;                      // percent of mass imbalance compared to total mass

        // Geometry to be configured
        private List<BoreHoleSize> boreHoleSizes =
            new List<BoreHoleSize>() {
                new BoreHoleSize(){Depth = 120, ID = 21 * Constants.in2m },
                new BoreHoleSize(){Depth = 2181, ID = 9.0*Constants.in2m } 
            };
        public Vector<double> rHole;                             // [m] Wellbore radius calculation
        public Vector<double> rc;                                // [m] drillstring radial clearance to the borehole wall

        public Wellbore(in Drillstring d,
                        in LumpedCells l,
                        List<BoreHoleSize> boreHoleSizes,
                        double topDriveMomentOfInertia,
                        double fluidDamping)
        {
            I_TD = topDriveMomentOfInertia;
            Df = fluidDamping;

            this.boreHoleSizes = boreHoleSizes; 
            rHole = Vector<double>.Build.Dense(d.OuterRadius.Count);
            rc = Vector<double>.Build.Dense(d.OuterRadius.Count);

            UpdateWellbore(d, l);
        }

        public void UpdateWellbore(in Drillstring d, in LumpedCells l)
        {
            // Wellbore radius calculation
            rHole = Vector<double>.Build.Dense(d.OuterRadius.Count);
            int idx = 0;
            for (int i = 1; i < l.xL.Count(); i++)
            {
                if (idx < boreHoleSizes.Count)
                {
                    if (l.xL[i] <= boreHoleSizes[idx].Depth)
                    {
                        rHole[i - 1] = boreHoleSizes[idx].ID / 2;
                    }
                    else
                    {
                        while (idx < boreHoleSizes.Count)
                        {
                            idx++;
                            if (idx < boreHoleSizes.Count && l.xL[i] <= boreHoleSizes[idx].Depth)
                            {
                                rHole[i - 1] = boreHoleSizes[idx].ID / 2;
                                break;
                            }
                        }
                    }
                }
                if (idx >= boreHoleSizes.Count)
                {
                    rHole[i - 1] = d.Rb;
                }
            }

            rc = rHole - d.OuterRadius;
            foreach (int i in d.SleeveIndexPosition)
                rc[i] = rHole[i] - d.SleeveOuterRadius;
        }
    }
}
