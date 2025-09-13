namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Input
    {
        public double V0 = 100 / 3600;                      // [m/s] surface axial velocity 
        public double omega_surf = 138 / 60 * 2 * Math.PI;  // [rad/s] surface angular velocity
        public double V0_pooh = -0.1;                       // [m/s] top of string velocity for pulling out of hole
        public double v0 = 0.0;                             // [m/s] surface axial velocity  
        public double tau_Motor = 0;                        // top drive torque matlab: u.tau_Motor
        public double tauMax = 60e3;                        // [N.m] maximum available top drive torque
        public double omega_sp = 0.0;                       // [rad/s] top drive rpm setpoint (different from omega_surf)
        public double Fshock = 0.0;                         // [N] add an extra shock force above the bit to induce whirl
        public double BottomExtraNormalForce = 0.0;       // [N] add an extra vertical force (weight) at the bit to simulate hole collapse
        public double differenceStaticKineticFriction = 0.0;
        public double stribeckCriticalVelocity = 0.05; // [m/s]
        public bool sticking = false; // request to stick the bit
    }
}
