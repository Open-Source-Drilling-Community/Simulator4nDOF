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

        public void Step(in DataModel.Configuration configuration, State state, Input input)
        {
            double Vdf; //[m/s] drillfloor velocity
            if (configuration.UseHeave)
                Vdf = configuration.HeaveAmplitude * 2 * Math.PI / configuration.HeavePeriod * Math.Cos(2 * Math.PI / configuration.HeavePeriod * state.Step * configuration.TimeStep); // [m / s] drillfloor velocity
            else
                Vdf = 0;

            if (state.Step * configuration.TimeStep - t_topdrive_startup > configuration.TopdriveStartupTime && !make_connection && !pooh_before_connection)
                input.CalculateSurfaceAxialVelocity = input.SurfaceAxialVelocity;
            else if (pooh_before_connection)
                input.CalculateSurfaceAxialVelocity = input.PullingOutOfHoleTopVelocity;
            else
                input.CalculateSurfaceAxialVelocity = 0.0;

            // add drilfloor velocity to top of string velocity setpoint
            input.CalculateSurfaceAxialVelocity = input.CalculateSurfaceAxialVelocity + Vdf;

            if (state.TopOfStringPosition < 1 && !pooh_before_connection)
            {
                pooh_before_connection = true;
                state.onBottom_startIdx = -1;
            }

            if (state.TopOfStringPosition > 2 && pooh_before_connection)
            {
                pooh_before_connection = false;
                make_connection = true;
                t_start_connection = state.Step * configuration.TimeStep;
            }

            if (make_connection && state.Step * configuration.TimeStep - t_start_connection > configuration.ConnectionTime)
            {
                make_connection = false;
                state.TopOfStringPosition = 31; // [m]
                t_topdrive_startup = state.Step * configuration.TimeStep;
            }

            if (make_connection)
                input.TopDriveRPMSetPoint = input.TopDriveRPMSetPoint + (0 - input.TopDriveRPMSetPoint) / (0.5 / configuration.TimeStep);
            else
                input.TopDriveRPMSetPoint = input.TopDriveRPMSetPoint + (input.SurfaceRotation - input.TopDriveRPMSetPoint) / (0.5 / configuration.TimeStep);
        }
    }
}
