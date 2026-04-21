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
        public Output(in SimulationParameters parameters, in Configuration config)
        {
            NormalForceProfileStiffString = Vector<double>.Build.Dense(parameters.NumberOfElements);
            NormalForceProfileSoftString = Vector<double>.Build.Dense(parameters.NumberOfElements);
            TensionProfile = Vector<double>.Build.Dense(parameters.NumberOfElements + 1);
            BendingMomentX = Vector<double>.Build.Dense(parameters.NumberOfElements);
            BendingMomentY = Vector<double>.Build.Dense(parameters.NumberOfElements);
            TangentialForceProfile = Vector<double>.Build.Dense(parameters.NumberOfElements);
            Depth = Vector<double>.Build.Dense(parameters.NumberOfElements);
            RadialDisplacement = Vector<double>.Build.Dense(parameters.NumberOfElements);
            WhirlAngle = Vector<double>.Build.Dense(parameters.NumberOfElements);
            WhirlSpeed = Vector<double>.Build.Dense(parameters.NumberOfElements);
            BendingMoment = Vector<double>.Build.Dense(parameters.NumberOfElements);
            Torque = Vector<double>.Build.Dense(parameters.NumberOfElements);

            queueSize = config.SSIWindowSize * (int)Math.Round(1 / parameters.OuterLoopTimeStep);
            BitRPMQueue = new Queue<double>(queueSize);
            SurfaceRPMQueue = new Queue<double>(queueSize);
            CummulativeStickSlipIndex = 0.0;
            AverageStickSlipIndex = 0.0;
            StickSlipIndex = 0.0;
            SimulationHealthy = true;
            this.configuration = config;
        }
        public void UpdateOutput(in State state, in SimulationParameters parameters)
        {
            double bendingAngleX = 0.0;
            double bendingAngleY = 0.0;
            double bendingAccelerationX = 0.0;
            double bendingAccelerationY = 0.0;
            // radial position of accelerometer relative to pipe centerline
            double initialRadialPosition; 
            double xDisplacement;
            double yDisplacement;
            double xAcceleration;
            double uAcceleration;
            double previousXDisplacement;
            double previousYDisplacement;
            double previousXAcceleration;
            double previousYAcceleration;
            double torsionalAcceleration = 0.0;
            double radialAcceleration = 0.0;
            double radialVelocity = 0.0;
            // tangential acceleration
            double tangentialAcceleration = 0.0;
            // whirl position of accelerometer relative to pipe centerline
            double initialTangentialPosition = 0;
                
            int sensorIndex = (parameters.Drillstring.SleeveIndexPosition.Contains(parameters.Drillstring.IndexSensor)) ?parameters.Drillstring.SleeveIndexPosition.IndexOf(parameters.Drillstring.IndexSensor) : parameters.Drillstring.IndexSensor;

            if (parameters.UsePipeMovementReconstruction) // % compute additional variables used for pipe movement reconstruction
            {
                double cosWhirlAngle = Math.Cos(state.WhirlAngle[sensorIndex]);
                double sinWhirlAngle = Math.Sin(state.WhirlAngle[sensorIndex]);
                radialVelocity = state.XVelocity[sensorIndex] * cosWhirlAngle + state.YVelocity[sensorIndex] * sinWhirlAngle;
                radialAcceleration = state.XAcceleration[sensorIndex] * cosWhirlAngle + state.YAcceleration[sensorIndex] * sinWhirlAngle 
                                - state.WhirlVelocity[sensorIndex] * state.XVelocity[sensorIndex] * cosWhirlAngle 
                                + state.WhirlVelocity[sensorIndex] * state.YVelocity[sensorIndex] * sinWhirlAngle;

                tangentialAcceleration = - (state.XAcceleration[sensorIndex] * sinWhirlAngle - 
                                state.YAcceleration[sensorIndex] * cosWhirlAngle + 2 * state.WhirlVelocity[sensorIndex] * radialVelocity) / state.RadialDisplacement[sensorIndex];
            
                initialRadialPosition = 0.5 * (parameters.Drillstring.NodeInnerRadius[sensorIndex] + parameters.Drillstring.NodeOuterRadius[sensorIndex]); 
                torsionalAcceleration = state.SlipCondition[sensorIndex] * state.AngularAcceleration[sensorIndex] + (1 - state.SlipCondition[sensorIndex]) * state.ThetaDotNoSlipSensor;
                xDisplacement = state.XDisplacement[sensorIndex] + initialRadialPosition * Math.Cos(state.AngularDisplacement[sensorIndex]) - initialTangentialPosition * Math.Sin(state.AngularDisplacement[sensorIndex]);
                yDisplacement = state.YDisplacement[sensorIndex] + initialTangentialPosition * Math.Cos(state.AngularDisplacement[sensorIndex]) + initialRadialPosition * Math.Sin(state.AngularDisplacement[sensorIndex]);
                xAcceleration = state.XAcceleration[sensorIndex] + initialRadialPosition * (-Math.Pow(state.WhirlVelocity[sensorIndex], 2) * Math.Cos(state.AngularDisplacement[sensorIndex]) - state.AngularAcceleration[sensorIndex] * Math.Sin(state.AngularDisplacement[sensorIndex])) +
                    initialTangentialPosition * (Math.Pow(state.WhirlVelocity[sensorIndex], 2) * Math.Sin(state.AngularDisplacement[sensorIndex]) -
                    state.AngularAcceleration[sensorIndex] * Math.Sin(state.AngularDisplacement[sensorIndex]));
                uAcceleration = state.YAcceleration[sensorIndex] + initialTangentialPosition * (-Math.Pow(state.WhirlVelocity[sensorIndex], 2) * Math.Cos(state.AngularDisplacement[sensorIndex]) - state.AngularAcceleration[sensorIndex] * Math.Sin(state.AngularDisplacement[sensorIndex])) -
                    initialRadialPosition * (Math.Pow(state.WhirlVelocity[sensorIndex], 2) * Math.Sin(state.AngularDisplacement[sensorIndex]) - state.AngularAcceleration[sensorIndex] * Math.Cos(state.AngularDisplacement[sensorIndex]));
            
                initialRadialPosition = 0.5 * (parameters.Drillstring.NodeInnerRadius[sensorIndex - 1] + parameters.Drillstring.NodeOuterRadius[sensorIndex - 1]);
                previousXDisplacement = state.XDisplacement[sensorIndex - 1] + initialRadialPosition * Math.Cos(state.AngularDisplacement[sensorIndex - 1]) - initialTangentialPosition * Math.Sin(state.AngularDisplacement[sensorIndex - 1]);
                previousYDisplacement = state.YDisplacement[sensorIndex - 1] + initialTangentialPosition * Math.Cos(state.AngularDisplacement[sensorIndex - 1]) + initialRadialPosition * Math.Sin(state.AngularDisplacement[sensorIndex - 1]);
                previousXAcceleration = state.XAcceleration[sensorIndex - 1] + initialRadialPosition * (-Math.Pow(state.WhirlVelocity[sensorIndex - 1], 2) * Math.Cos(state.AngularDisplacement[sensorIndex - 1]) -
                    state.AngularAcceleration[sensorIndex - 1] * Math.Sin(state.AngularDisplacement[sensorIndex - 1])) + initialTangentialPosition * (Math.Pow(state.WhirlVelocity[sensorIndex - 1], 2) * Math.Sin(state.AngularDisplacement[sensorIndex - 1]) -
                    state.AngularAcceleration[sensorIndex - 1] * Math.Sin(state.AngularDisplacement[sensorIndex - 1]));
                previousYAcceleration = state.YAcceleration[sensorIndex - 1] + initialTangentialPosition * (-Math.Pow(state.WhirlVelocity[sensorIndex - 1], 2) * Math.Cos(state.AngularDisplacement[sensorIndex - 1]) - state.AngularAcceleration[sensorIndex - 1] * Math.Sin(state.AngularDisplacement[sensorIndex - 1])) -
                   initialRadialPosition * (Math.Pow(state.WhirlVelocity[sensorIndex - 1], 2) * Math.Sin(state.AngularDisplacement[sensorIndex - 1]) - state.AngularAcceleration[sensorIndex - 1] * Math.Cos(state.AngularDisplacement[sensorIndex - 1]));
          
                // Bending angles
                bendingAngleX = -(yDisplacement - previousYDisplacement) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle x-component
                bendingAngleY = (xDisplacement - previousXDisplacement) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1];// Bending angle y-component
                bendingAccelerationX = -(uAcceleration - previousYAcceleration) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle second derivative x-component
                bendingAccelerationY = (xAcceleration - previousXAcceleration) / parameters.Drillstring.ElementLength[parameters.Drillstring.IndexSensor - 1]; // Bending angle second derivative y-component*/
            }

            Depth[0] = state.ZDisplacement[0] + state.TopDrive.AxialPosition;
            for (int i = 1; i < parameters.NumberOfElements; i ++)
            {
                Depth[i] = state.ZDisplacement[i] + parameters.Drillstring.RelativeNodeDepth[i];   
            }
            BitVelocity = state.ZVelocity[state.ZVelocity.Count - 1];//Bit Velocity
            // Parse outputs
            NormalForceProfileStiffString = state.NormalCollisionForce; // Pipe shear strain 
            NormalForceProfileSoftString = state.SoftStringNormalForce;
            TensionProfile = state.Tension; // Tension profile
           
            Torque = state.Torque; // Torque profile vs. depth
            BendingMomentX = state.BendingMomentX;// Bending moment x-component profile
            BendingMomentY = state.BendingMomentY;// Bending moment y-component profile
            //TangentialForceProfile = TangentialCoulombFrictionForce;// Bending moment y-component profile Tangential force profile
            WeightOnBit = state.WeightOnBit;  // Weight on bit 
            TorqueOnBit = state.TorqueOnBit;  // Torque on bit
            

            SensorMb_x = state.BendingMomentX[parameters.Drillstring.IndexSensor];
            SensorMb_y = state.BendingMomentY[parameters.Drillstring.IndexSensor];
            RadialDisplacement = state.RadialDisplacement;
            WhirlAngle = state.WhirlAngle;
            WhirlSpeed = state.WhirlVelocity;
            BendingMoment = (Square(state.BendingMomentX) + Square(state.BendingMomentY)).PointwiseSqrt(); // the bending due to curvature is already included in the bending moment components + simulationParameters.Drillstring.E.PointwiseMultiply(simulationParameters.Drillstring.I).PointwiseMultiply(simulationParameters.Trajectory.curvature);

            if (parameters.UsePipeMovementReconstruction)
            {
                SensorAxialVelocity = state.ZVelocity[parameters.Drillstring.IndexSensor]; // sleeve angular displacement;
                SensorAxialDisplacement = SensorAxialDisplacement + SensorAxialVelocity * parameters.OuterLoopTimeStep;
                SensorAngularPosition = state.AngularDisplacement[sensorIndex]; //pipe angular displacement
                SensorAngularVelocity = state.AngularVelocity[sensorIndex]; //pipe angular velocity
                
                SensorRadialPosition = state.RadialDisplacement[parameters.Drillstring.IndexSensor]; //radial position
                SensorWhirlAngle = state.WhirlAngle[parameters.Drillstring.IndexSensor]; //whirl angle
                SensorRadialSpeed =state.RadialVelocity[parameters.Drillstring.IndexSensor]; //radial velocity
                SensorWhirlSpeed = state.WhirlVelocity[parameters.Drillstring.IndexSensor]; //whirl velocity
                SensorAxialAcceleration = state.ZAcceleration[parameters.Drillstring.IndexSensor]; //axial acceleration
                SensorAngularAcceleration = torsionalAcceleration; // angular acceleration
                SensorRadialAcceleration = radialAcceleration; // radial acceleration
                SensorWhirlAcceleration = tangentialAcceleration; // whirl acceleration
                SensorBendingAngleX = bendingAngleX; // bending angle x-component
                SensorBendingAngleY = bendingAngleY; // bending angle y-component
                SecondDerivativeSensorBendingAngleX = bendingAccelerationX; // bending angle second derivative x-component
                SecondDerivativeSensorBendingAngleY = bendingAccelerationY; // bending angle second derivative y-component
                SensorPipeInclination = parameters.Trajectory.InterpolatedThetaAtNode[parameters.Drillstring.IndexSensor];
                SensorPipeAzimuthAt = parameters.Trajectory.InterpolatedPhiAtNode[parameters.Drillstring.IndexSensor];
                SensorTension = state.Tension[parameters.Drillstring.IndexSensor];
                SensorTorque = Torque[parameters.Drillstring.IndexSensor];

                CalculateInLocalFrame(state, parameters);
            }
        }
        public void CalculateInLocalFrame(in State state,in SimulationParameters parameters)
        {
            Matrix<double> SensorToPipeLocal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), parameters.Drillstring.SensorRadialDistance },
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { -Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle), 0, Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle), 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeLocalToPipeNonRotating = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(SensorAngularPosition), -Math.Sin(SensorAngularPosition), SensorBendingAngleY * Math.Cos(SensorAngularPosition) + SensorBendingAngleX * Math.Sin(SensorAngularPosition), 0 },
                        { Math.Sin(SensorAngularPosition), Math.Cos(SensorAngularPosition), SensorBendingAngleY * Math.Sin(SensorAngularPosition) - SensorBendingAngleX * Math.Cos(SensorAngularPosition), 0 },
                        { -SensorBendingAngleY, SensorBendingAngleX, 1, 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeNonRotatingToLateralRotationCenter = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { 1, 0, 0, SensorRadialPosition * Math.Cos(SensorWhirlAngle) },
                        { 0, 1, 0, SensorRadialPosition * Math.Sin(SensorWhirlAngle) },
                        { 0, 0, 1, SensorAxialDisplacement },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> LateralRotationCenterToGlobal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(SensorPipeAzimuthAt), Math.Cos(SensorPipeInclination) * Math.Sin(SensorPipeAzimuthAt), -Math.Sin(SensorPipeInclination) * Math.Sin(SensorPipeAzimuthAt), 0 },
                        { -Math.Sin(SensorPipeAzimuthAt), Math.Cos(SensorPipeInclination) * Math.Cos(SensorPipeAzimuthAt), -Math.Sin(SensorPipeInclination) * Math.Cos(SensorPipeAzimuthAt), 0 },
                        { 0, Math.Sin(SensorPipeInclination), Math.Cos(SensorPipeInclination), 0 },
                        { 0, 0, 0, 1 }
            });

            Matrix<double> PipeLocalToSensor = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), -Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) },
                        { -Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), 0 },
                        { Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Cos(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Sin(parameters.Drillstring.SensorMisalignmentPolarAngle) * Math.Sin(parameters.Drillstring.SensorMisalignmentAzimuthAngle), Math.Cos(parameters.Drillstring.SensorMisalignmentPolarAngle) }
            });

            Matrix<double> PipeNonRotatingToPipeLocal = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(SensorAngularPosition), Math.Sin(SensorAngularPosition), -SensorBendingAngleY },
                        { -Math.Sin(SensorAngularPosition), Math.Cos(SensorAngularPosition), SensorBendingAngleX },
                        { SensorBendingAngleY * Math.Cos(SensorAngularPosition) + SensorBendingAngleX * Math.Sin(SensorAngularPosition), SensorBendingAngleY * Math.Sin(SensorAngularPosition) - SensorBendingAngleX * Math.Cos(SensorAngularPosition), 1 }
            });

            Matrix<double> LateralRotationCenterToPipeNonRotating = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 }
            });

            Matrix<double> GlobalToLateralRotationCenter = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { Math.Cos(SensorPipeAzimuthAt), -Math.Sin(SensorPipeAzimuthAt), 0 },
                        { Math.Cos(SensorPipeInclination) * Math.Sin(SensorPipeAzimuthAt), Math.Cos(SensorPipeInclination) * Math.Cos(SensorPipeAzimuthAt), Math.Sin(SensorPipeInclination) },
                        { -Math.Sin(SensorPipeInclination) * Math.Sin(SensorPipeAzimuthAt), -Math.Sin(SensorPipeInclination) * Math.Cos(SensorPipeAzimuthAt), Math.Cos(SensorPipeInclination) }
            });

            Matrix<double> PipeLocalToLateralRotationCenterSecondDerivative = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                        { -Math.Pow(SensorAngularVelocity, 2) * Math.Cos(SensorAngularPosition) - SensorAngularAcceleration * Math.Sin(SensorAngularPosition), Math.Pow(SensorAngularVelocity, 2) * Math.Sin(SensorAngularPosition) - SensorAngularAcceleration * Math.Cos(SensorAngularPosition), 0, (SensorRadialAcceleration - SensorRadialPosition * Math.Pow(SensorWhirlSpeed, 2)) * Math.Cos(SensorWhirlAngle) -
                        (2 * SensorRadialSpeed * SensorWhirlSpeed + SensorRadialPosition * SensorWhirlAcceleration) * Math.Sin(SensorWhirlAngle) },
                        { -Math.Pow(SensorAngularVelocity, 2) * Math.Sin(SensorAngularPosition) + SensorAngularAcceleration * Math.Cos(SensorAngularPosition), -Math.Pow(SensorAngularVelocity, 2) * Math.Cos(SensorAngularPosition) - SensorAngularAcceleration * Math.Sin(SensorAngularPosition), 0, (SensorRadialAcceleration - SensorRadialPosition * Math.Pow(SensorWhirlSpeed, 2)) * Math.Sin(SensorWhirlAngle) +
                        (2 * SensorRadialSpeed * SensorWhirlSpeed + SensorRadialPosition * SensorWhirlAcceleration) * Math.Cos(SensorWhirlAngle) },
                        { -SecondDerivativeSensorBendingAngleY, SecondDerivativeSensorBendingAngleX, 0, SensorAxialAcceleration },
                        { 0, 0, 0, 0 }
            });

            // Calculate acceleration in global frame
            var accelerationInGlobalFrameM = LateralRotationCenterToGlobal * PipeLocalToLateralRotationCenterSecondDerivative * SensorToPipeLocal * ToVector((parameters.Drillstring.SensorDisplacementsInLocalFrame.Append(1).ToArray()));
            var accelerationInGlobalFrame = accelerationInGlobalFrameM.SubVector(0, 3) - Vector<double>.Build.DenseOfArray(new double[] { 0, 0, Constants.GravitationalAcceleration });

            var accelerationInLocalFrame = PipeLocalToSensor * PipeNonRotatingToPipeLocal * LateralRotationCenterToPipeNonRotating * GlobalToLateralRotationCenter * accelerationInGlobalFrame;

            SensorRadialAccelerationLocalFrame = -accelerationInLocalFrame[0];
            SensorTangentialAccelerationLocalFrame = accelerationInLocalFrame[1];
            SensorAxialAccelerationLocalFrame = accelerationInLocalFrame[2];

            SensorBendingMomentX = Math.Sqrt(Math.Pow(SensorMb_x, 2) + Math.Pow(SensorMb_y, 2)) * Math.Sin((SensorAngularVelocity - SensorWhirlSpeed) * state.Step * parameters.OuterLoopTimeStep);
            SensorBendingMomentY = Math.Sqrt(Math.Pow(SensorMb_x, 2) + Math.Pow(SensorMb_y, 2)) * Math.Cos((SensorAngularVelocity - SensorWhirlSpeed) * state.Step * parameters.OuterLoopTimeStep);
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
