using MathNet.Numerics.LinearAlgebra; 
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class State
    {
        public Matrix<double> f;                // Pipe shear strain
        public Matrix<double> o;                // Pipe angular velocity
        public Matrix<double> e;                // Pipe axial strain
        public Matrix<double> v;                // Pipe axial velocity

        public Vector<double> OL;               // Lumped element angular velocity
        public Vector<double> VL;               // Lumped element axial velocity
        public double Otd;                      // Top drive angular velocity
        public Vector<double> OS;               // Sleeve angular velocity
        public Vector<double> l;                // Depth of cut

        public Vector<double> Theta;
        public Vector<double> Theta_S;
        public Vector<double> Xc;
        public Vector<double> Xc_dot;
        public Vector<double> Yc;
        public Vector<double> Yc_dot;
        public Vector<double> slip_condition;

        // Mud motor stator and rotor angular velocity
        public double Ostator;
        public double Orotor;

        public double BitDepth;
        public double HoleDepth;
        public double TopOfStringPosition;
 
        public double previousCalculatedBitDepth; // Used for update of trajectory 
        public bool make_connection;
        public bool pooh_before_connection;
        public double onBottom_startIdx;        // start index of onBottom in results array
        public bool onBottom;
        public int step;

        public double t_start_connection;       // [s] connection start time
        public double t_topdrive_startup;       // [s] top drive start up time

        public State(in Parameters p, double BitDepth, double HoleDepth, double TopOfStringPosition)
        {
            step = 0;

            f = Matrix<double>.Build.Dense(p.lc.PL, p.lc.NL);       // Pipe shear strain
            o = Matrix<double>.Build.Dense(p.lc.PL, p.lc.NL);       // Pipe angular velocity
            e = Matrix<double>.Build.Dense(p.lc.PL, p.lc.NL);       // Pipe axial strain
            v = Matrix<double>.Build.Dense(p.lc.PL, p.lc.NL);       // Pipe axial velocity;

            OL = Vector<double>.Build.Dense(p.lc.NL);               // Lumped element angular velocity
            VL = Vector<double>.Build.Dense(p.lc.NL);               // Lumped element axial velocity
            Otd = 0;                                                // Top drive angular velocity
            OS = Vector<double>.Build.Dense(p.ds.NS);               // Sleeve angular velocity, can be empty vector if no sleeves
            Theta = Vector<double>.Build.Dense(p.lc.NL);            // Lumped element angular displacement
            Theta_S = Vector<double>.Build.Dense(p.ds.NS);          // Sleeve angular displacement, can be empty vector if no sleeves
            l = Vector<double>.Build.Dense(p.dc.Pl);                // Depth of cut
            Xc = Vector<double>.Build.Dense(p.lc.NL);               // Lumped element lateral displacement, x direction
            Xc_dot = Vector<double>.Build.Dense(p.lc.NL);           // Lumped element lateral velocity, x direction
            Yc = Vector<double>.Build.Dense(p.lc.NL);               // Lumped element lateral displacement, y direction
            Yc_dot = Vector<double>.Build.Dense(p.lc.NL);           // Lumped element lateral velocity, y direction
            slip_condition = Vector<double>.Build.Dense(p.lc.NL);   // Slip condition evaluated at each lumped element

            Ostator = 0;
            Orotor = 0;

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
        }

        public void AddNewLumpedElement()
        {
            var fCol0 = Vector<double>.Build.Dense(f.RowCount, f[0, 0]).ToColumnMatrix();// Pipe shear strain              
            f = fCol0.Append(f);

            var oCol0 = Vector<double>.Build.Dense(o.RowCount, o[0, 0]).ToColumnMatrix();// Pipe angular velocity              
            o = oCol0.Append(o);

            var eCol0 = Vector<double>.Build.Dense(e.RowCount, e[0, 0]).ToColumnMatrix();// Pipe axial strain
            e = eCol0.Append(e);
              
            var vCol0 = Vector<double>.Build.Dense(v.RowCount, v[0, 0]).ToColumnMatrix();// Pipe axial velocity;
            v = vCol0.Append(v);

            OL = ExtendVectorStart(OL[0], OL);                      // Lumped element angular velocity
            VL = ExtendVectorStart(VL[0], VL);                      // Lumped element axial velocity
            Theta = ExtendVectorStart(Theta[0], Theta);             // Lumped element angular displacement
            Xc = ExtendVectorStart(Xc[0], Xc);                      // Lumped element lateral displacement, x direction
            Xc_dot = ExtendVectorStart(Xc_dot[0], Xc_dot);          // Lumped element lateral velocity, x direction
            Yc = ExtendVectorStart(Yc[0], Yc);                      // Lumped element lateral displacement, y direction
            Yc_dot = ExtendVectorStart(Yc_dot[0], Yc_dot);          // Lumped element lateral velocity, y direction
            slip_condition = ExtendVectorStart(slip_condition[0], slip_condition); //Slip condition evaluated at each lumped element
        }
    }

}
