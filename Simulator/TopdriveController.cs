using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class TopdriveController
    {
        // Top drive ZTorque controller parameters
        readonly double t_vfd = 0.005;                       // [s] VFD filter time constant
        readonly double t_en = 0.002;                        // [s] encoder time constant
        readonly double t_sp = 0.005;                        // [s] acceleration filter time constant
        readonly double t_hp = 20;                           // [s] torque high pass filter time constant
        readonly double t_lp = 0.005;                        // [s] torque low pass filter time constant
        
        readonly double z_factor = 0.25;                     // additional ZTorque tuning factor between 0 - 1
        readonly double inertia_correction_factor = 1.00;    // correction factor for estimated inertia

        double ZT;                                           // ZTorque factor

        // Top drive controller gains
        double KpSoftTorqueSpeed = 4;                   
        double KiSoftTorqueSpeed = 0;
        double TuningFrequenceySoftTorqueSpeed = 0.2;       // [Hz] tuning frequency
        double KpFactor = 50;
        double KiFactor = 5;

        double I_comp;                              // [kg.m ^ 2] inertia compensation
        double II = 0.0;                            // integral error for top drive speed controller

        double omega_0_ZT;             
        double tau_vfd;                
        double td_accel;
        double tau_0_ZT;
        double tau_0_highpass;
        double tau_0_lowpass;

        double Otd0;                                 // Previous TopDriveAngularVelocity (top drive angular velocity) 

        public TopdriveController(in DataModel.Configuration c, in SimulationParameters simulationParameters)
        {
            KpSoftTorqueSpeed = c.KpSoftTorqueSpeed;
            KiSoftTorqueSpeed = c.KiSoftTorqueSpeed;
            TuningFrequenceySoftTorqueSpeed = c.TuningFrequenceySoftTorqueSpeed;
            KpFactor = c.KpStiffAndZTorque;
            KiFactor = c.KiStiffAndZTorque;
            t_vfd = c.VFDFilterTimeconstantZTorque;
            t_en = c.EncoderTimeConstantZTorque;
            t_sp = c.AccelerationFilterTimeConstantZTorque;
            t_hp = c.TorqueHighPassFilterTimeConstantZTorque;
            t_lp = c.TorqueLowPassFilterTimeConstantZTorque;
            z_factor = c.AdditionalTuningFactorZTorque;
            inertia_correction_factor = c.InertiaCorrectionFactorZTorque;

            if (c.TopDriveController == 2)
            {
                //Tuned PI controller parameters
                I_comp = KiSoftTorqueSpeed * simulationParameters.Wellbore.TopDriveInertia; // [kg.m ^ 2] inertia compensation
                KpFactor = KpSoftTorqueSpeed * simulationParameters.Drillstring.CharacteristicDrillPipeImpedance; // top drive controller P gain
                KiFactor = Math.Pow(2 * Math.PI * TuningFrequenceySoftTorqueSpeed, 2) * (simulationParameters.Wellbore.TopDriveInertia - I_comp); // top drive controller I gain
            }
            else
            {    // Stiff PI controller parameters
                KpFactor = KpFactor * simulationParameters.Drillstring.CharacteristicDrillPipeImpedance;
                KiFactor = KiFactor * simulationParameters.Wellbore.TopDriveInertia;
            }

            if (c.TopDriveController == 3)
            {
                //TODO move
                ZT = 1.0 / simulationParameters.Drillstring.CharacteristicDrillPipeImpedance; // ZTorque factor
            }

            omega_0_ZT = 0.0;
            tau_vfd = 0.0;
            td_accel = 0.0;
            tau_0_ZT = 0.0;
            tau_0_highpass = 0.0; 
            tau_0_lowpass = 0.0;
            Otd0 = 0.0; // Previous TopDriveAngularVelocity (top drive angular velocity)  
        }

        public void Step(in DataModel.Configuration c, in State state, in SimulationParameters simulationParameters, ref Input u)
        {
            double delta_Otd = state.TopDriveAngularVelocity - Otd0;
            Otd0 = state.TopDriveAngularVelocity;
            double e_PI;

            if (c.TopDriveController == 3)
            {
                if (state.Step > 0)
                {
                    omega_0_ZT = omega_0_ZT * Math.Exp(-2 * Math.PI * c.TimeStep / t_en) +
                        state.TopDriveAngularVelocity * (1 - Math.Exp(-2 * Math.PI * c.TimeStep / t_en));                    

                    tau_vfd = tau_vfd * Math.Exp(-2 * Math.PI * c.TimeStep / t_vfd) +
                        u.TopDriveTorque * (1 - Math.Exp(-2 * Math.PI * c.TimeStep / t_vfd));

                    td_accel = td_accel * Math.Exp(-2 * Math.PI * c.TimeStep / t_sp) +
                        delta_Otd / c.TimeStep * (1 - Math.Exp(-2 * Math.PI * c.TimeStep / t_sp));

                    double previous_tau_0_ZT = tau_0_ZT;
                    tau_0_ZT = tau_vfd- inertia_correction_factor * simulationParameters.Wellbore.TopDriveInertia * td_accel;  // estimated pipe torque

                    tau_0_highpass = tau_0_highpass * (t_hp / 2.0 / Math.PI / (c.TimeStep + t_hp / 2.0 / Math.PI)) + 
                        (tau_0_ZT - previous_tau_0_ZT) * (t_hp / 2.0 / Math.PI / (c.TimeStep + t_hp / 2.0 / Math.PI));
                    tau_0_lowpass = tau_0_lowpass * Math.Exp(-2 * Math.PI * c.TimeStep / t_lp) + tau_0_highpass * (1 - Math.Exp(-2 * Math.PI * c.TimeStep / t_lp));                   
                }
                else
                {
                    omega_0_ZT = 0;
                    tau_0_ZT = 0;
                    tau_vfd = 0;
                    td_accel = 0;
                    tau_0_highpass = 0;
                    tau_0_lowpass = 0;
                }

                e_PI = u.TopDriveRPMSetPoint - omega_0_ZT - z_factor * ZT * tau_0_lowpass;
            }
            else
                e_PI = u.TopDriveRPMSetPoint - state.TopDriveAngularVelocity;

            if (c.TopDriveController == 2)
                u.TopDriveTorque = e_PI * KpFactor + II * KiFactor + I_comp * delta_Otd / c.TimeStep;
            else
                u.TopDriveTorque = e_PI * KpFactor + II * KiFactor;

            u.TopDriveTorque = Math.Min(u.TopDriveTorque, u.MaximumTopDriveTorque);
            u.TopDriveTorque = Math.Max(u.TopDriveTorque, 0);

            II = II + c.TimeStep * e_PI; // if c.dt is too large, controller may be too aggressive
        }
    }


}
