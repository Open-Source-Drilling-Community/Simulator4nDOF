using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class LumpedCells
    {
        // Move somewhere else? todo
        public double L;                                // [m] Drillstring length
        public int NL;                                  // Total number of lumped elements
        public Vector<double> xL;
        public double dxL = 30;                         // [m] Length between two lumped elements        
        public int Pt;                                  // Actual number of cells in discretization of distributed sections
        public readonly int PL = 5;                     // Ratio of distributed cells to lumped cells (rounded to nearest integer)

        public LumpedCells(double MD, double LengthBetweenLumpedElements)
        {
            dxL = LengthBetweenLumpedElements;
            NL = (int)Math.Round(MD / dxL, MidpointRounding.AwayFromZero);// Total number of lumped elements              
            Pt = PL * NL;                               // Actual number of cells in discretization of distributed sections
            L = MD;                                     // [m] Drillstring length
            xL = ToVector(Linspace(0, L, NL + 1));
            this.dxL = Diff(xL.ToArray()).ToList().Average(); // recompute dxL due to possible rounding differences
            //PL = (int)Math.Round((double)Pt / (double)NL, MidpointRounding.AwayFromZero);
        }
    }
}
