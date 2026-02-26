using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class DrawworksAndTopdriveController
    {
        private double t_start_connection = 0;               // [s] connection start time
        private double t_topdrive_startup = 0;               // [s] top drive start up time

        // variables for rig activity state machine
        private bool make_connection = false;
        private bool pooh_before_connection = false;

        public void Step(State state, in SimulationParameters parameters)
        {
            double Vdf; //[m/s] drillfloor velocity
            if (parameters.TopDriveDrawwork.UseHeave)
                Vdf = parameters.TopDriveDrawwork.HeaveAmplitude * 2 * Math.PI / parameters.TopDriveDrawwork.HeavePeriod * Math.Cos(2 * Math.PI / parameters.TopDriveDrawwork.HeavePeriod * state.Step * parameters.OuterLoopTimeStep); // [m / s] drillfloor velocity
            else
                Vdf = 0;

            if (state.Step * parameters.OuterLoopTimeStep - t_topdrive_startup > parameters.TopDriveDrawwork.TopDriveStartupTime && !make_connection && !pooh_before_connection)
                state.TopDrive.CalculateSurfaceAxialVelocity = parameters.TopDriveDrawwork.SurfaceAxialVelocity;
            else if (pooh_before_connection)
                state.TopDrive.CalculateSurfaceAxialVelocity = parameters.TopDriveDrawwork.PullingOutOfHoleTopVelocity;
            else
                state.TopDrive.CalculateSurfaceAxialVelocity = 0.0;

            // add drilfloor velocity to top of string velocity setpoint
            state.TopDrive.CalculateSurfaceAxialVelocity = state.TopDrive.CalculateSurfaceAxialVelocity + Vdf;

            if (state.TopOfStringPosition < 1 && !pooh_before_connection)
            {
                pooh_before_connection = true;
                state.onBottom_startIdx = -1;
            }

            if (state.TopOfStringPosition > 2 && pooh_before_connection)
            {
                pooh_before_connection = false;
                make_connection = true;
                t_start_connection = state.Step * parameters.OuterLoopTimeStep;
            }

            if (make_connection && state.Step * parameters.OuterLoopTimeStep - t_start_connection > parameters.TopDriveDrawwork.ConnectionTime)
            {
                make_connection = false;
                state.TopOfStringPosition = 31; // [m]
                t_topdrive_startup = state.Step * parameters.OuterLoopTimeStep;
            }

            if (make_connection)
                state.TopDrive.TopDriveRPMSetPoint = state.TopDrive.TopDriveRPMSetPoint + 2 * parameters.OuterLoopTimeStep * (0 - state.TopDrive.TopDriveRPMSetPoint);
            else
                state.TopDrive.TopDriveRPMSetPoint = state.TopDrive.TopDriveRPMSetPoint + 2 * parameters.OuterLoopTimeStep * (parameters.TopDriveDrawwork.SurfaceRotation - state.TopDrive.TopDriveRPMSetPoint);
        }
    }
}
