namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class MudMotor
    {
        public readonly double omega0_motor = 11.31;    // [rad / s] Angular velocity at zero load torque
        public readonly double P0_motor = 40e5;         // [Pa] Pressure at zero load torque
        public readonly double T_max_motor = 12000;     // [N.m] Maximum PDM torque
        public readonly double alpha_motor = 4.538;     // exponent of PDM RPM - torque curve
        public readonly double I_rotor = 3.0;           // rotor mass moment of inertia, kg·m2
        public readonly double M_rotor = 350;           // rotor mass, kg
        public readonly double N_stator = 6;            // number of stator lobes
        public readonly double N_rotor = 5;             // number of rotor lobes
        public readonly double delta_rotor = 0.01;      // [m] eccentricity of rotor
        public readonly double rev_per_volume = 0.02;   // [rev / L] PDM revolutions per volume

        public double V_motor;                          // volume of motor, m^3

        public MudMotor()
        {
            V_motor = 1 / (N_rotor * rev_per_volume) / 2 / Math.PI * 1e-3; // volume of motor, m^3
        }
    }
}
