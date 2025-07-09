using MathNet.Numerics.LinearAlgebra;
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

        // Geomertry to be configured
        private double casingShoeDepth = 2181;                   // [m] Casing shoe depth
        private double linerShoeDepth = 0;                       // [m] Liner shoe depth, set to 0 if there is no liner
        private double casingID = 9.0 * Constants.in2m;          // [m] Casing inner diameter
        private double linerID = 0 * Constants.in2m;             // [m] Casing outer diameter
        private double wellheadDepth = 120;                      // [m] Well head depth
        private double riserID = 21 * Constants.in2m;            // [m] Riser inner diameter

        public Vector<double> rHole;                             // [m] Wellbore radius calculation
        public Vector<double> rc;                                // [m] drillstring radial clearance to the borehole wall

        public Wellbore(in Drillstring d,
                        in LumpedCells l,
                        double casingShoeDepth,
                        double linerShoeDepth,
                        double casingID,
                        double linerID,
                        double wellheadDepth,
                        double riserID,
                        double topDriveMomentOfInertia,
                        double fluidDamping)
        {
            I_TD = topDriveMomentOfInertia; 
            Df = fluidDamping;
            this.casingShoeDepth = casingShoeDepth;
            this.linerShoeDepth = linerShoeDepth;
            this.linerID = linerID;
            this.wellheadDepth = wellheadDepth;
            this.riserID = riserID;
            this.casingID = casingID;
            rHole = Vector<double>.Build.Dense(d.ro.Count);
            rc = Vector<double>.Build.Dense(d.ro.Count);

            UpdateWellbore(d, l);
        }

        public void UpdateWellbore(in Drillstring d, in LumpedCells l)
        {
            // Wellbore radius calculation
            rHole = Vector<double>.Build.Dense(d.ro.Count);
            for (int i = 1; i < l.xL.Count(); i++)
            {
                if (wellheadDepth == 0)
                {
                    rHole[i - 1] = l.xL[i] <= casingShoeDepth ? casingID / 2 : d.Rb;
                    if (linerShoeDepth != 0 && l.xL[i] > casingShoeDepth && l.xL[i] <= linerShoeDepth)
                        rHole[i - 1] = linerID / 2;
                }
                else
                {
                    rHole[i - 1] = l.xL[i] <= wellheadDepth ? riserID / 2 : l.xL[i] <= casingShoeDepth ? casingID / 2 : d.Rb;
                    if (linerShoeDepth != 0 && l.xL[i] > casingShoeDepth && l.xL[i] <= linerShoeDepth)
                        rHole[i - 1] = linerID / 2;
                }
            }

            rc = rHole - d.ro;
            foreach (int i in d.iS)
                rc[i] = rHole[i] - d.r_So;
        }
    }
}
