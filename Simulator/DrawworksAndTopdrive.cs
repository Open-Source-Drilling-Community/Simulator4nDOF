using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class DrawworksAndTopdrive
    {
        private double t_start_connection = 0;               // [s] connection start time
        private double t_topdrive_startup = 0;               // [s] top drive start up time

        // variables for rig activity state machine
        private bool make_connection = false;
        private bool pooh_before_connection = false;

        public void Step(in DataModel.Configuration c,State state, Input u)
        {
            double Vdf; //[m/s] drillfloor velocity
            if (c.UseHeave)
                Vdf = c.HeaveAmplitude * 2 * Math.PI / c.HeavePeriod * Math.Cos(2 * Math.PI / c.HeavePeriod * state.step * c.TimeStep); // [m / s] drillfloor velocity
            else
                Vdf = 0;

            if (state.step * c.TimeStep - t_topdrive_startup > c.TopdriveStartupTime && !make_connection && !pooh_before_connection)
                u.v0 = u.V0;
            else if (pooh_before_connection)
                u.v0 = u.V0_pooh;
            else
                u.v0 = 0.0;

            // add drilfloor velocity to top of string velocity setpoint
            u.v0 = u.v0 + Vdf;

            if (state.TopOfStringPosition < 1 && !pooh_before_connection)
            {
                pooh_before_connection = true;
                state.onBottom_startIdx = -1;
            }

            if (state.TopOfStringPosition > 2 && pooh_before_connection)
            {
                pooh_before_connection = false;
                make_connection = true;
                t_start_connection = state.step * c.TimeStep;
            }

            if (make_connection && state.step * c.TimeStep - t_start_connection > c.ConnectionTime)
            {
                make_connection = false;
                state.TopOfStringPosition = 31; // [m]
                t_topdrive_startup = state.step * c.TimeStep;
            }

            if (make_connection)
                u.omega_sp = u.omega_sp + (0 - u.omega_sp) / (0.5 / c.TimeStep);
            else
                u.omega_sp = u.omega_sp + (u.omega_surf - u.omega_sp) / (0.5 / c.TimeStep);
        }
    }
}
