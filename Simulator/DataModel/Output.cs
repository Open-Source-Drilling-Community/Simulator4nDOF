using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Output
    {
        public Vector<double> NormalForceProfileStiffString;                  // Normal force profile stiff string
        public Vector<double> NormalForceProfileSoftString;       // Normal force profile soft string
        public Vector<double> TensionProfile;              // tension profile
        public Vector<double> BendingMomentX;                 // Bending moment y-component profile
        public Vector<double> BendingMomentY;                 // Bending moment z-component profile
        public Vector<double> TangentialForceProfile;                 // Tangential force profile

        public double WeightOnBit;                          // Weight on bit
        public double TorqueOnBit;                          // torque on bit

        public double BitRotationInRPM;                      // Bit rpm;
        public double TopDriveRotationInRPM;                     // Top drive rpm;
        public double BitVelocity;                           // [m/s] bit velodity

        public double SensorAxialVelocity;
        public double SensorAxialDisplacement;
        public double SensorAngularPosition;
        public double SensorAngularVelocity;
        public double SensorRadialPosition;
        public double SensorWhirlAngle;
        public double SensorRadialSpeed;
        public double SensorWhirlSpeed;
        public double SensorAxialAcceleration;
        public double SensorAngularAcceleration;
        public double SensorRadialAcceleration;
        public double SensorWhirlAcceleration;
        public double SensorBendingAngleX;
        public double SensorBendingAngleY;
        public double SecondDerivativeSensorBendingAngleX;
        public double SecondDerivativeSensorBendingAngleY;
        public double SensorPipeInclination;
        public double SensorPipeAzimuthAt;
        public double SensorMb_x;
        public double SensorMb_y;
        public double SensorBendingMomentX;
        public double SensorBendingMomentY;
        public double SensorTension;
        public double SensorTorque;

        public double SensorRadialAccelerationLocalFrame;
        public double SensorTangentialAccelerationLocalFrame;
        public double SensorAxialAccelerationLocalFrame;

        public Vector<double> RadialDisplacement;                   // Lateral displacement
        public Vector<double> WhirlAngle;
        public Vector<double> WhirlSpeed;
        public Vector<double> BendingMoment;
        public Vector<double> Torque;               // Torque profile
        public Vector<double> Depth;

        public double StickSlipIndex { get; private set; }
        public double CummulativeStickSlipIndex { get; private set; }         // cumulative stick slip severity for the entire run
        public double AverageStickSlipIndex { get; private set; } // cumulative stick slip severity for the entire run
        public Queue<double> BitRPMQueue { get; private set; }
        public Queue<double> SurfaceRPMQueue { get; private set; }

        private readonly Configuration configuration;
        private int queueSize;

        public bool SimulationHealthy;
        public Output(in SimulationParameters simulationParameters, in Configuration config)
        {
            NormalForceProfileStiffString = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            NormalForceProfileSoftString = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            TensionProfile = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements + 1);
            BendingMomentX = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            BendingMomentY = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            TangentialForceProfile = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            Depth = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            RadialDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            WhirlAngle = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            WhirlSpeed = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            BendingMoment = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            Torque = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements * simulationParameters.LumpedCells.DistributedToLumpedRatio);

            queueSize = config.SSIWindowSize * (int)Math.Round(1 / simulationParameters.OuterLoopTimeStep);
            BitRPMQueue = new Queue<double>(queueSize);
            SurfaceRPMQueue = new Queue<double>(queueSize);
            CummulativeStickSlipIndex = 0.0;
            AverageStickSlipIndex = 0.0;
            StickSlipIndex = 0.0;
            SimulationHealthy = true;
            this.configuration = config;
        }

        private void AddSurfaceRPM(double value)
        {
            if (SurfaceRPMQueue.Count >= queueSize)
            {
                SurfaceRPMQueue.Dequeue(); // Remove oldest element
            }
            SurfaceRPMQueue.Enqueue(value);
        }

        private void AddBitRPM(double value)
        {
            if (BitRPMQueue.Count >= queueSize)
            {
                BitRPMQueue.Dequeue(); // Remove oldest element
            }
            BitRPMQueue.Enqueue(value);
        }

        private double GetOldestSurfaceRPM()
        {
            return SurfaceRPMQueue.Count > 0 ? SurfaceRPMQueue.Peek() : 0.0;
        }

        private double GetAverageBitRPM()
        {
            return BitRPMQueue.Count > 0 ? BitRPMQueue.Average() : 0.0;
        }

        private double GetMaxBitRPM()
        {
            return BitRPMQueue.Count > 0 ? BitRPMQueue.Max() : 0.0;
        }

        private double GetMinBitRPM()
        {
            return BitRPMQueue.Count > 0 ? BitRPMQueue.Min() : 0.0;
        }

        private bool SurfaceRPMQueIsFull()
        {
            return SurfaceRPMQueue.Count >= queueSize;
        }

        public void UpdateSSI(double time)
        {
            AddSurfaceRPM(TopDriveRotationInRPM); // Add value to the queue when it's set
            AddBitRPM(BitRotationInRPM); // Add value to the queue when it's set
            CummulativeStickSlipIndex = CummulativeStickSlipIndex + SSI * configuration.TimeStep;
            AverageStickSlipIndex = CummulativeStickSlipIndex/ time;
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
