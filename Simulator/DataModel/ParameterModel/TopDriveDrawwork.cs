namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class TopDriveDrawwork
    {
        public int TopDriveControllerType {get; set; } = 1;
        public bool UseHeave {get; set; } = false;
        public double HeaveAmplitude {get; set; } = 0.0; // [m] heave amplitude
        public double HeavePeriod {get; set; } = 10.0; // [s] heave period
        public double SurfaceAxialVelocity {get; set; } = 100 / 3600;                      // [m/s] surface axial velocity 
        public double SurfaceRotation {get; set; } = 138 / 60 * 2 * Math.PI;  // [rad/s] surface angular velocity
        public double PullingOutOfHoleTopVelocity {get; set; } = -0.1;                       // [m/s] top of string velocity for pulling out of hole        
        public double TopDriveMotorTorque {get; set; } = 0;                        // top drive torque matlab: u.tau_Motor
        public double MaximumTopDriveTorque {get; set; } = 60e3;                        // [N.m] maximum available top drive torque
        public double TopDriveRPMSetPoint {get; set; } = 0.0;                       // [rad/s] top drive rpm setpoint (different from omega_surf)
        public double TopDriveStartupTime {get; set; } = 10.0; // [s] time for top drive to reach the surface rotation during startup 
        public double ConnectionTime {get; set; } = 60.0; // [s] time for making connection
    }
}
