using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class TopdriveController
    {
        // Top drive ZTorque controller simulationParameters
        readonly double VFDTimeFilterConstant = 0.005;                       // [s] VFD filter time constant
        readonly double EnconderTimeConstant = 0.002;                        // [s] encoder time constant
        readonly double AccelerationFilterTimeConstant = 0.005;                        // [s] acceleration filter time constant
        readonly double TorqueHighPassTimeConstant = 20;                           // [s] torque high pass filter time constant
        readonly double TorqueLowPassTimeConstant = 0.005;                        // [s] torque low pass filter time constant
        
        readonly double TorqueTuningFactor = 0.25;                     // additional ZTorque tuning factor between 0 - 1
        readonly double IntertiaCorrectionFactor = 1.00;    // correction factor for estimated inertia

        double TorqueFactor;                                           // ZTorque factor

        // Top drive controller gains
        double KpSoftTorqueSpeed = 4;                   
        double KiSoftTorqueSpeed = 0;
        double TuningFrequenceySoftTorqueSpeed = 0.2;       // [Hz] tuning frequency
        double KpFactor = 50;
        double KiFactor = 5;

        double InertiaCompensation;                              // [kg.m ^ 2] inertia compensation
        double IntegralError = 0.0;                            // integral error for top drive speed controller

        double Omega0Torque;             
        double TorqueVFD;                
        double TopDriveAcceleration;
        double Torque0;
        double Torque0HighPass;
        double Torque0LowPass;

        double PreviousTopDriveAngularSpeed;                                 // Previous TopDriveAngularVelocity (top drive angular velocity) 

        public TopdriveController(in DataModel.Configuration configuration, in SimulationParameters simulationParameters)
        {
            KpSoftTorqueSpeed = configuration.KpSoftTorqueSpeed;
            KiSoftTorqueSpeed = configuration.KiSoftTorqueSpeed;
            TuningFrequenceySoftTorqueSpeed = configuration.TuningFrequenceySoftTorqueSpeed;
            KpFactor = configuration.KpStiffAndZTorque;
            KiFactor = configuration.KiStiffAndZTorque;
            VFDTimeFilterConstant = configuration.VFDFilterTimeconstantZTorque;
            EnconderTimeConstant = configuration.EncoderTimeConstantZTorque;
            AccelerationFilterTimeConstant = configuration.AccelerationFilterTimeConstantZTorque;
            TorqueHighPassTimeConstant = configuration.TorqueHighPassFilterTimeConstantZTorque;
            TorqueLowPassTimeConstant = configuration.TorqueLowPassFilterTimeConstantZTorque;
            TorqueTuningFactor = configuration.AdditionalTuningFactorZTorque;
            IntertiaCorrectionFactor = configuration.InertiaCorrectionFactorZTorque;

            if (simulationParameters.TopDriveDrawwork.TopDriveControllerType == 2)
            {
                //Tuned PI controller simulationParameters
                InertiaCompensation = KiSoftTorqueSpeed * simulationParameters.TopDriveDrawwork.TopDriveInertia; // [kg.m ^ 2] inertia compensation
                KpFactor = KpSoftTorqueSpeed * simulationParameters.Drillstring.CharacteristicDrillPipeImpedance; // top drive controller P gain
                KiFactor = Math.Pow(2 * Math.PI * TuningFrequenceySoftTorqueSpeed, 2) * (simulationParameters.TopDriveDrawwork.TopDriveInertia - InertiaCompensation); // top drive controller I gain
            }
            else
            {    // Stiff PI controller simulationParameters
                KpFactor = KpFactor * simulationParameters.Drillstring.CharacteristicDrillPipeImpedance;
                KiFactor = KiFactor * simulationParameters.TopDriveDrawwork.TopDriveInertia;
            }

            if (simulationParameters.TopDriveDrawwork.TopDriveControllerType == 3)
            {
                //TODO move
                TorqueFactor = 1.0 / simulationParameters.Drillstring.CharacteristicDrillPipeImpedance; // ZTorque factor
            }

            Omega0Torque = 0.0;
            TorqueVFD = 0.0;
            TopDriveAcceleration = 0.0;
            Torque0 = 0.0;
            Torque0HighPass = 0.0; 
            Torque0LowPass = 0.0;
            PreviousTopDriveAngularSpeed = 0.0; // Previous TopDriveAngularVelocity (top drive angular velocity)  
        }

        public void Step(State state, in SimulationParameters simulationParameters)
        {
            double deltaTopDriveSpeed = state.TopDrive.TopDriveAngularVelocity - PreviousTopDriveAngularSpeed;
            PreviousTopDriveAngularSpeed = state.TopDrive.TopDriveAngularVelocity;
            double e_PI;

            if (simulationParameters.TopDriveDrawwork.TopDriveControllerType == 3)
            {
                if (state.Step > 0)
                {
                    Omega0Torque = Omega0Torque * Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / EnconderTimeConstant) +
                        state.TopDrive.TopDriveAngularVelocity * (1 - Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / EnconderTimeConstant));                    

                    TorqueVFD = TorqueVFD * Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / VFDTimeFilterConstant) +
                        state.TopDrive.TopDriveMotorTorque * (1 - Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / VFDTimeFilterConstant));

                    TopDriveAcceleration = TopDriveAcceleration * Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / AccelerationFilterTimeConstant) +
                        deltaTopDriveSpeed / simulationParameters.OuterLoopTimeStep * (1 - Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / AccelerationFilterTimeConstant));

                    double previous_tau_0_ZT = Torque0;
                    Torque0 = TorqueVFD- IntertiaCorrectionFactor * simulationParameters.TopDriveDrawwork.TopDriveInertia * TopDriveAcceleration;  // estimated pipe torque

                    Torque0HighPass = Torque0HighPass * (TorqueHighPassTimeConstant / 2.0 / Math.PI / (simulationParameters.OuterLoopTimeStep + TorqueHighPassTimeConstant / 2.0 / Math.PI)) + 
                        (Torque0 - previous_tau_0_ZT) * (TorqueHighPassTimeConstant / 2.0 / Math.PI / (simulationParameters.OuterLoopTimeStep + TorqueHighPassTimeConstant / 2.0 / Math.PI));
                    Torque0LowPass = Torque0LowPass * Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / TorqueLowPassTimeConstant) + Torque0HighPass * (1 - Math.Exp(-2 * Math.PI * simulationParameters.OuterLoopTimeStep / TorqueLowPassTimeConstant));                   
                }
                else
                {
                    Omega0Torque = 0;
                    Torque0 = 0;
                    TorqueVFD = 0;
                    TopDriveAcceleration = 0;
                    Torque0HighPass = 0;
                    Torque0LowPass = 0;
                }

                e_PI = state.TopDrive.TopDriveRPMSetPoint - Omega0Torque - TorqueTuningFactor * TorqueFactor * Torque0LowPass;
            }
            else
                e_PI = state.TopDrive.TopDriveRPMSetPoint - state.TopDrive.TopDriveAngularVelocity;

            if (simulationParameters.TopDriveDrawwork.TopDriveControllerType == 2)
                state.TopDrive.TopDriveMotorTorque = e_PI * KpFactor + IntegralError * KiFactor + InertiaCompensation * deltaTopDriveSpeed / simulationParameters.OuterLoopTimeStep;
            else
                state.TopDrive.TopDriveMotorTorque = e_PI * KpFactor + IntegralError * KiFactor;

            state.TopDrive.TopDriveMotorTorque = Math.Min(state.TopDrive.TopDriveMotorTorque, state.TopDrive.MaximumTopDriveTorque);
            state.TopDrive.TopDriveMotorTorque = Math.Max(state.TopDrive.TopDriveMotorTorque, 0);

            IntegralError = IntegralError + simulationParameters.OuterLoopTimeStep * e_PI; // if configuration.dt is too large, controller may be too aggressive
        }
    }


}
