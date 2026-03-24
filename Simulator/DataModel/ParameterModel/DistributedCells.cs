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
        public int WaveToLumpedRatio;
        public DistributedCells(in SimulatorDrillString drillString,
            in double omega0, in int waveToLumpedRatio, in double elementLength)
        {
            WaveToLumpedRatio = waveToLumpedRatio; 
            double lengthBetweenWaveNodes = elementLength * waveToLumpedRatio;
            NumberOfElements = (int) Math.Ceiling(drillString.TotalLength / lengthBetweenWaveNodes);
            ElementLength = drillString.TotalLength / ((double) NumberOfElements);
          
            OmegaMax = Math.Max(omega0 * 5, 2 * Math.PI);
            // [m] length of cell in depth of cut PDE
            double dxl = TimeStepForDepthOfCutPDE * OmegaMax;
            // Number of cells in depth of cut PDE
            CellsInDepthOfCut = (int)Math.Floor(1 / dxl);
        }


    }
}
