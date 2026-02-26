using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class DistributedCells
    {
        // [m] All sections
        public Vector<double> x;
        // [m] Length of distributed section + lumped section
        public double DistributedSectionAndLumpedLength;
        // [m] Length of only distributed section; all distributed sections should be same length
        public double DistributedSectionLength;
        // [s] Time step for depth of cut PDE
        public readonly double TimeStepForDepthOfCutPDE = 0.002;//.002;
        // [rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
        public double OmegaMax;
        // Number of cells in depth of cut PDE
        public int CellsInDepthOfCut;

        public DistributedCells(LumpedCells lc, SimulatorDrillString ds, double omega0)
        {
            x = Vector<double>.Build.Dense(Enumerable.Range(0, lc.DiscretiazionCellsInDistributedSections)
                                                     .Select(i => i * lc.Length / (lc.DiscretiazionCellsInDistributedSections - 1))
                                                     .ToArray());

            DistributedSectionAndLumpedLength = x.Skip(1).Zip(x, (a, b) => a - b).Average();


            // l_L is lumped drill pipe length
            double mean_l_L = ds.LumpedElementMassMomentOfInertia.Average();

            double start = 0;
            double end = lc.Length - mean_l_L * lc.NumberOfLumpedElements;
            double[] linspace = Linspace(start, end, lc.DiscretiazionCellsInDistributedSections);
            double[] diffArray = Diff(linspace);
            DistributedSectionLength = diffArray.Average();                              //[m] Length of only distributed section; all distributed sections should be same length
            //dxM = Math.Min(1.0, dxM);

            // As per the CFL condition for the axial / torsional wave equations
            double dtTemp = DistributedSectionLength / Math.Max(ds.TorsionalWaveSpeed, ds.AxialWaveSpeed) * 0.80;
            // [rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
            OmegaMax = Math.Max(omega0 * 5, 2 * Math.PI);
            // [m] length of cell in depth of cut PDE
            double dxl = TimeStepForDepthOfCutPDE * OmegaMax;
            // Number of cells in depth of cut PDE
            CellsInDepthOfCut = (int)Math.Floor(1 / dxl);
        }


    }
}
