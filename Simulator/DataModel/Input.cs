namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Input
    {
        public double SurfaceAxialVelocity = 100 / 3600;                      // [m/s] surface axial velocity 
        public double SurfaceRotation = 138 / 60 * 2 * Math.PI;  // [rad/s] surface angular velocity
        public double PullingOutOfHoleTopVelocity = -0.1;                       // [m/s] top of string velocity for pulling out of hole
        public double CalculateSurfaceAxialVelocity = 0.0;                             // [m/s] surface axial velocity  
        public double TopDriveTorque = 0;                        // top drive torque matlab: u.tau_Motor
        public double MaximumTopDriveTorque = 60e3;                        // [N.m] maximum available top drive torque
        public double TopDriveRPMSetPoint = 0.0;                       // [rad/s] top drive rpm setpoint (different from omega_surf)
        public double ForceToInduceBitWhirl = 0.0;                         // [N] add an extra shock force above the bit to induce whirl
        public double BottomExtraNormalForce = 0.0;       // [N] add an extra vertical force (weight) at the bit to simulate hole collapse
        public double DifferenceStaticKineticFriction = 0.0;
        public double StribeckCriticalVelocity = 0.05; // [m/s]
        public bool StickingBoolean = false; // request to stick the bit
    }
}
