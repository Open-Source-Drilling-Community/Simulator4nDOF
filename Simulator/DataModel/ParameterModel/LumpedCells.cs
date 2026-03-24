using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class LumpedCells
    {
        public double ElementLength;                   // [m] Drillstring length
        public int NumberOfLumpedElements;      // Total number of lumped elements
        public Vector<double> CumulativeElementLength;
        public double DistanceBetweenElements;  // [m] Length between two lumped elements        
        public LumpedCells(double meanDistance, double lengthBetweenLumpedElements)
        {
            DistanceBetweenElements = lengthBetweenLumpedElements;
            NumberOfLumpedElements = (int)Math.Round(meanDistance / DistanceBetweenElements, MidpointRounding.AwayFromZero);// Total number of lumped elements              
            ElementLength = meanDistance;                                     // [m] Drillstring length
            CumulativeElementLength = ToVector(Linspace(0, ElementLength, NumberOfLumpedElements + 1));
            this.DistanceBetweenElements = Diff(CumulativeElementLength.ToArray()).ToList().Average(); // recompute dxL due to possible rounding differences
            //PL = (int)Math.Round((double)Pt / (double)NL, MidpointRounding.AwayFromZero);
        }
    }
}
