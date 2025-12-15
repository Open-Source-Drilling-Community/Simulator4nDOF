using MathNet.Numerics.LinearAlgebra; 
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class State
    {
        public Matrix<double> PipeShearStrain;                // Pipe shear strain
        public Matrix<double> PipeAngularVelocity;                // Pipe angular velocity
        public Matrix<double> PipeAxialStrain;                // Pipe axial strain
        public Matrix<double> PipeAxialVelocity;                // Pipe axial velocity

        public Vector<double> LumpedElementAngularVelocity;               // Lumped element angular velocity
        public Vector<double> LumpedElementAxialVelocity;               // Lumped element axial velocity
        public double TopDriveAngularVelocity;                      // Top drive angular velocity
        public Vector<double> SleeveAngularVelocity;               // Sleeve angular velocity
        public Vector<double> DepthOfCut;                // Depth of cut

        public Vector<double> Theta;
        public Vector<double> Theta_S;
        public Vector<double> Xc;
        public Vector<double> Xc_dot;
        public Vector<double> Yc;
        public Vector<double> Yc_dot;
        public Vector<double> slip_condition;

        public Matrix<double> TorsionalDownwardTravelingWave; // Downward traveling wave, torsional
        public Matrix<double> TorsionalUpwardTravelingWave;  // Upward traveling wave, torsional
        public Matrix<double> AxialDownwardTravelingWave; // Downward traveling wave, axial
        public Matrix<double> AxialUpwardTravelingWave; // Upward traveling wave, axial


        


        // Mud motor stator and rotor angular velocity
        public double MudStatorAngularVelocity;
        public double MudRotorAngularVelocity;

        public double BitDepth;
        public double HoleDepth;
        public double TopOfStringPosition;
        public double BitVelocity;
 
        public double previousCalculatedBitDepth; // Used for update of trajectory 
        public bool make_connection;
        public bool pooh_before_connection;
        public double onBottom_startIdx;        // start index of onBottom in results array
        public bool onBottom;
        public int step;

        public double t_start_connection;       // [s] connection start time
        public double t_topdrive_startup;       // [s] top drive start up time

        public State(in SimulationParameters simulationParameters, double BitDepth, double HoleDepth, double TopOfStringPosition)
        {
            step = 0;

            PipeShearStrain = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.PL, simulationParameters.LumpedCells.NL);       // Pipe shear strain
            PipeAngularVelocity = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.PL, simulationParameters.LumpedCells.NL);       // Pipe angular velocity
            PipeAxialStrain = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.PL, simulationParameters.LumpedCells.NL);       // Pipe axial strain
            PipeAxialVelocity = Matrix<double>.Build.Dense(simulationParameters.LumpedCells.PL, simulationParameters.LumpedCells.NL);       // Pipe axial velocity;

            LumpedElementAngularVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);               // Lumped element angular velocity
            LumpedElementAxialVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);               // Lumped element axial velocity
            TopDriveAngularVelocity = 0;                                                // Top drive angular velocity
            SleeveAngularVelocity = Vector<double>.Build.Dense(simulationParameters.Drillstring.TotalSleeveNumber);               // Sleeve angular velocity, can be empty vector if no sleeves
            Theta = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);            // Lumped element angular displacement
            Theta_S = Vector<double>.Build.Dense(simulationParameters.Drillstring.TotalSleeveNumber);          // Sleeve angular displacement, can be empty vector if no sleeves
            DepthOfCut = Vector<double>.Build.Dense(simulationParameters.DistributedCells.CellsInDepthOfCut);                // Depth of cut
            Xc = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);               // Lumped element lateral displacement, x direction
            Xc_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);           // Lumped element lateral velocity, x direction
            Yc = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);               // Lumped element lateral displacement, y direction
            Yc_dot = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);           // Lumped element lateral velocity, y direction
            slip_condition = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NL);   // Slip condition evaluated at each lumped element

            MudStatorAngularVelocity = 0;
            MudRotorAngularVelocity = 0;

            this.BitDepth = BitDepth;                               // Initial values set in SimulationParameters - to be moved
            previousCalculatedBitDepth = BitDepth;
            this.HoleDepth = HoleDepth;
            this.TopOfStringPosition = TopOfStringPosition;
            onBottom = false;

            make_connection = false;
            pooh_before_connection = false;
            t_start_connection = 0;                                 // [s] connection start time
            t_topdrive_startup = 0;
            onBottom_startIdx = -1;
            TorsionalDownwardTravelingWave = this.PipeAngularVelocity + simulationParameters.Drillstring.TorsionalWaveSpeed * this.PipeShearStrain; // Downward traveling wave, torsional
            TorsionalUpwardTravelingWave = this.PipeAngularVelocity - simulationParameters.Drillstring.TorsionalWaveSpeed * this.PipeShearStrain; // Upward traveling wave, torsional
            AxialDownwardTravelingWave = this.PipeAxialVelocity + simulationParameters.Drillstring.AxialWaveSpeed * this.PipeAxialStrain; // Downward traveling wave, axial
            AxialUpwardTravelingWave = this.PipeAxialVelocity - simulationParameters.Drillstring.AxialWaveSpeed * this.PipeAxialStrain; // Upward traveling wave, axial
            BitVelocity = 0.5 * (AxialDownwardTravelingWave[simulationParameters.LumpedCells.PL - 1, AxialDownwardTravelingWave.ColumnCount - 1] 
                + AxialUpwardTravelingWave[simulationParameters.LumpedCells.PL - 1, AxialUpwardTravelingWave.ColumnCount - 1]);

        }

        public void AddNewLumpedElement()
        {
            var fCol0 = Vector<double>.Build.Dense(PipeShearStrain.RowCount, PipeShearStrain[0, 0]).ToColumnMatrix();// Pipe shear strain              
            PipeShearStrain = fCol0.Append(PipeShearStrain);

            var oCol0 = Vector<double>.Build.Dense(PipeAngularVelocity.RowCount, PipeAngularVelocity[0, 0]).ToColumnMatrix();// Pipe angular velocity              
            PipeAngularVelocity = oCol0.Append(PipeAngularVelocity);

            var eCol0 = Vector<double>.Build.Dense(PipeAxialStrain.RowCount, PipeAxialStrain[0, 0]).ToColumnMatrix();// Pipe axial strain
            PipeAxialStrain = eCol0.Append(PipeAxialStrain);
              
            var vCol0 = Vector<double>.Build.Dense(PipeAxialVelocity.RowCount, PipeAxialVelocity[0, 0]).ToColumnMatrix();// Pipe axial velocity;
            PipeAxialVelocity = vCol0.Append(PipeAxialVelocity);

            LumpedElementAngularVelocity = ExtendVectorStart(LumpedElementAngularVelocity[0], LumpedElementAngularVelocity);                      // Lumped element angular velocity
            LumpedElementAxialVelocity = ExtendVectorStart(LumpedElementAxialVelocity[0], LumpedElementAxialVelocity);                      // Lumped element axial velocity
            Theta = ExtendVectorStart(Theta[0], Theta);             // Lumped element angular displacement
            Xc = ExtendVectorStart(Xc[0], Xc);                      // Lumped element lateral displacement, x direction
            Xc_dot = ExtendVectorStart(Xc_dot[0], Xc_dot);          // Lumped element lateral velocity, x direction
            Yc = ExtendVectorStart(Yc[0], Yc);                      // Lumped element lateral displacement, y direction
            Yc_dot = ExtendVectorStart(Yc_dot[0], Yc_dot);          // Lumped element lateral velocity, y direction
            slip_condition = ExtendVectorStart(slip_condition[0], slip_condition); //Slip condition evaluated at each lumped element
        }
    }

}
