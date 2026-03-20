using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class DistributedCells
    {
        // [m] All sections
        //public Vector<double> x;
        // [m] Length of distributed section + lumped section
        //public double DistributedSectionAndLumpedLength;
        // [m] Length of only distributed section; all distributed sections should be same length
        //public double DistributedSectionLength;
        // [s] Time step for depth of cut PDE
        public readonly double TimeStepForDepthOfCutPDE = 0.002;//.002;
        // [rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
        public double OmegaMax;
        // Number of cells in depth of cut PDE
        public int CellsInDepthOfCut;
        public int NumberOfElements;
        public double ElementLength;
        public DistributedCells(in SimulatorDrillString drillString, 
            in double omega0, in double lengthBetweenWaveNodes)
        {

            NumberOfElements = (int) Math.Ceiling(drillString.TotalLength / lengthBetweenWaveNodes);
            ElementLength = drillString.TotalLength / ((double) NumberOfElements);
            //x = Vector<double>.Build.Dense(Enumerable.Range(0, lumpedCells.DiscretiazionCellsInDistributedSections)
            //                                         .Select(i => i * lumpedCells.Length / (lumpedCells.DiscretiazionCellsInDistributedSections - 1))
            //                                         .ToArray());

            //DistributedSectionAndLumpedLength = x.Skip(1).Zip(x, (a, b) => a - b).Average();

            // l_L is lumped drill pipe length
            //double mean_l_L = drillString.LumpedElementMassMomentOfInertia.Average();
            //double start = 0;
            //double end = lumpedCells.Length - mean_l_L * lumpedCells.NumberOfLumpedElements;
            //double[] linspace = Linspace(start, end, lumpedCells.DiscretiazionCellsInDistributedSections);
            //double[] diffArray = Diff(linspace);
            //DistributedSectionLength = diffArray.Average();                              //[m] Length of only distributed section; all distributed sections should be same length
            //dxM = Math.Min(1.0, dxM);

            // As per the CFL condition for the axial / torsional wave equations
            //double dtTemp = ElementLength / Math.Max(drillString.TorsionalWaveSpeed, drillString.AxialWaveSpeed) * 0.80;
            // [rad/s] Maximum bit angular velocity for enforcing CFL condition in depth of cut PDE
            OmegaMax = Math.Max(omega0 * 5, 2 * Math.PI);
            // [m] length of cell in depth of cut PDE
            double dxl = TimeStepForDepthOfCutPDE * OmegaMax;
            // Number of cells in depth of cut PDE
            CellsInDepthOfCut = (int)Math.Floor(1 / dxl);
        }


    }
}
