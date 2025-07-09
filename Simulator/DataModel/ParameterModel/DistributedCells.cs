using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class DistributedCells
    {
        public Vector<double> x;                // [m] All sections
        public double dx;                       // [m] Length of distributed section + lumped section
        public double dxM;                      // [m] Length of only distributed section; all distributed sections should be same length
        public readonly double dt_BRI = .002;   // [s] Time step for depth of cut PDE
        public double omegaMAX;                 // [rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
        public int Pl;                          // Number of cells in depth of cut PDE

        public DistributedCells(LumpedCells lc, Drillstring ds, double omega0)
        {
            x = Vector<double>.Build.Dense(Enumerable.Range(0, lc.Pt)
                                                     .Select(i => i * lc.L / (lc.Pt - 1))
                                                     .ToArray());

            dx = x.Skip(1).Zip(x, (a, b) => a - b).Average();


            // l_L is lumped drill pipe length
            double mean_l_L = ds.l_L.Average();

            double start = 0;
            double end = lc.L - mean_l_L * lc.NL;
            double[] linspace = Linspace(start, end, lc.Pt);
            double[] diffArray = Diff(linspace);
            dxM = diffArray.Average();                              //[m] Length of only distributed section; all distributed sections should be same length
            //dxM = Math.Min(1.0, dxM);

            double dtTemp = dxM / Math.Max(ds.c_t, ds.c_a) * 0.80;  //As per the CFL condition for the axial / torsional wave equations
            omegaMAX = Math.Max(omega0 * 5, 2 * Math.PI);           //[rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
            double dxl = dt_BRI * omegaMAX;                         //[m] length of cell in depth of cut PDE
            Pl = (int)Math.Floor(1 / dxl);                          // Number of cells in depth of cut PDE
        }


    }
}
