using MathNet.Numerics.LinearAlgebra; 
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class TopDriveAndDrawworkState
    {
        // Top Drive state variables
        public double AngularVelocity;                // Top drive angular velocity        
        public double RotationAngle;
        public double TopDriveMotorTorque;                           // Top drive motor torque
        public double TopDriveRPMSetPoint;
        public double MaximumTopDriveTorque;                        // [N.m] maximum available top drive torque
        public double AxialVelocity;
        public double AxialPosition;                    // Top of string position
        public double RelativeAxialPosition;
        
    }
}
