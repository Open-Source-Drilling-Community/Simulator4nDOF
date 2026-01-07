using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class LumpedCells
    {
        // Move somewhere else? todo
        public double Length;                   // [m] Drillstring length
        public int NumberOfLumpedElements;      // Total number of lumped elements
        public Vector<double> ElementLength;
        public double DistanceBetweenElements;  // [m] Length between two lumped elements        
        public int DiscretiazionCellsInDistributedSections; // Actual number of cells in discretization of distributed sections
        public readonly int DistributedToLumpedRatio = 5;    // Ratio of distributed cells to lumped cells (rounded to nearest integer)

        public LumpedCells(double MD, double LengthBetweenLumpedElements)
        {
            DistanceBetweenElements = LengthBetweenLumpedElements;
            NumberOfLumpedElements = (int)Math.Round(MD / DistanceBetweenElements, MidpointRounding.AwayFromZero);// Total number of lumped elements              
            DiscretiazionCellsInDistributedSections = DistributedToLumpedRatio * NumberOfLumpedElements;                               // Actual number of cells in discretization of distributed sections
            Length = MD;                                     // [m] Drillstring length
            ElementLength = ToVector(Linspace(0, Length, NumberOfLumpedElements + 1));
            this.DistanceBetweenElements = Diff(ElementLength.ToArray()).ToList().Average(); // recompute dxL due to possible rounding differences
            //PL = (int)Math.Round((double)Pt / (double)NL, MidpointRounding.AwayFromZero);
        }
    }
}
