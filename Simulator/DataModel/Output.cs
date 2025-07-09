using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Output
    {
        public Vector<double> F_N;                  // Normal force profile stiff string
        public Vector<double> F_N_softstring;       // Normal force profile soft string
        public Vector<double> tension;              // tension profile
        public Vector<double> Mb_x;                 // Bending moment y-component profile
        public Vector<double> Mb_y;                 // Bending moment z-component profile
        public Vector<double> Fc_t;                 // Tangential force profile

        public double wob;                          // Weight on bit
        public double tob;                          // torque on bit

        public double omega_b;                      // Bit rpm;
        public double omega_td;                     // Top drive rpm;
        public double vb;                           // [m/s] bit velodity

        public double sensorAxialVelocity;
        public double sensorAxialDisplacement;
        public double sensorAngularPosition;
        public double sensorAngularVelocity;
        public double sensorRadialPosition;
        public double sensorWhirlAngle;
        public double sensorRadialSpeed;
        public double sensorWhirlSpeed;
        public double sensorAxialAcceleration;
        public double sensorAngularAcceleration;
        public double sensorRadialAcceleration;
        public double sensorWhirlAcceleration;
        public double sensorBendingAngleX;
        public double sensorBendingAngleY;
        public double sensorBendingAngleX_SecondDerivative;
        public double sensorBendingAngleY_SecondDerivative;
        public double sensorPipeInclination;
        public double sensorPipeAzimuthAt;
        public double sensorMb_x;
        public double sensorMb_y;
        public double sensorBendingMomentX;
        public double sensorBendingMomentY;

        public double sensorRadialAccelerationLocalFrame;
        public double sensorTangentialAccelerationLocalFrame;
        public double sensorAxialAccelerationLocalFrame;

        public Vector<double> rc;                   // Lateral displacement
        public Vector<double> phi;
        public Vector<double> phi_dot;
        public Vector<double> Mb;
        public Vector<double> torque;               // Torque profile
        public Vector<double> depths;

        public double ssi { get; private set; }
        public double cumulative_ssi { get; private set; }         // cumulative stick slip severity for the entire run
        public double average_cumulative_ssi { get; private set; } // cumulative stick slip severity for the entire run
        public Queue<double> bitRPMQueue { get; private set; }
        public Queue<double> surfaceRPMQueue { get; private set; }

        private readonly Configuration c;
        private int queueSize;

        public Output(in Parameters p, in Configuration c)
        {
            F_N = Vector<double>.Build.Dense(p.lc.NL);
            F_N_softstring = Vector<double>.Build.Dense(p.lc.NL);
            tension = Vector<double>.Build.Dense(p.lc.NL + 1);
            Mb_x = Vector<double>.Build.Dense(p.lc.NL);
            Mb_y = Vector<double>.Build.Dense(p.lc.NL);
            Fc_t = Vector<double>.Build.Dense(p.lc.NL);
            depths = Vector<double>.Build.Dense(p.lc.NL);
            rc = Vector<double>.Build.Dense(p.lc.NL);
            phi = Vector<double>.Build.Dense(p.lc.NL);
            phi_dot = Vector<double>.Build.Dense(p.lc.NL);
            Mb = Vector<double>.Build.Dense(p.lc.NL);
            torque = Vector<double>.Build.Dense(p.lc.NL * p.lc.PL);

            queueSize = c.SSIWindowSize * (int)Math.Round(1 / c.TimeStep);
            bitRPMQueue = new Queue<double>(queueSize);
            surfaceRPMQueue = new Queue<double>(queueSize);
            cumulative_ssi = 0.0;
            average_cumulative_ssi = 0.0;
            ssi = 0.0;

            this.c = c;
        }

        private void AddSurfaceRPM(double value)
        {
            if (surfaceRPMQueue.Count >= queueSize)
            {
                surfaceRPMQueue.Dequeue(); // Remove oldest element
            }
            surfaceRPMQueue.Enqueue(value);
        }

        private void AddBitRPM(double value)
        {
            if (bitRPMQueue.Count >= queueSize)
            {
                bitRPMQueue.Dequeue(); // Remove oldest element
            }
            bitRPMQueue.Enqueue(value);
        }

        private double GetOldestSurfaceRPM()
        {
            return surfaceRPMQueue.Count > 0 ? surfaceRPMQueue.Peek() : 0.0;
        }

        private double GetAverageBitRPM()
        {
            return bitRPMQueue.Count > 0 ? bitRPMQueue.Average() : 0.0;
        }

        private double GetMaxBitRPM()
        {
            return bitRPMQueue.Count > 0 ? bitRPMQueue.Max() : 0.0;
        }

        private double GetMinBitRPM()
        {
            return bitRPMQueue.Count > 0 ? bitRPMQueue.Min() : 0.0;
        }

        private bool SurfaceRPMQueIsFull()
        {
            return surfaceRPMQueue.Count >= queueSize;
        }

        public void UpdateSSI(double time)
        {
            AddSurfaceRPM(omega_td); // Add value to the queue when it's set
            AddBitRPM(omega_b); // Add value to the queue when it's set
            cumulative_ssi = cumulative_ssi + SSI * c.TimeStep;
            average_cumulative_ssi = cumulative_ssi/ time;
        }

        public double SSI
        {
            get
            {
                if (SurfaceRPMQueIsFull() && GetOldestSurfaceRPM() > 2 * Math.PI) // RPM > 60
                {
                    return (GetMaxBitRPM() - GetMinBitRPM()) / (2 * GetAverageBitRPM());
                }
                else
                {
                    return 0;
                }
            }
        }
    }
}
