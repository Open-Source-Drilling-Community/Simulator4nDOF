using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator;
using NORCE.Drilling.Simulator.DataModel;
using NORCE.Drilling.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Config
    {
        public double VirtualSensorPositionFromBit { get; set; } = 63;
        public double LoggingIntervalScalarValues { get; set; } = 0.5;
        public double LoggingIntervalProfiles { get; set; } = 0.0;

        public double CoulombStaticFriction  { get; set; } = 0.3;         // [-] Coulomb static friction coefficient, f.mu_s_factor
        public double CoulombKineticFriction { get; set; } = 0.2;       // [-] coulomb kinetic friction coefficient, f.mu_k_factor

        public double HeaveAmplitude { get; set; } = 0.0;             // [m] heave amplitude(assumed constant over time)
        public double HeavePeriod { get; set; } = 11;              // [s] heave period(assumed constant over time)

        public int TopDriveController { get; set; } = 1;                 // 1 - stiff PI controller, 2 - tuned PI controller(SoftTorque/SoftSpeed), 3 - ZTorque

        public double VFDFilterTimeconstantZTorque { get; set; } = 0.005;          // [s] VFD filter time constant
        public double EncoderTimeConstantZTorque { get; set; } = 0.002;            // [s] encoder time constant
        public double AccelerationFilterTimeConstantZTorque { get; set; } = 0.005; // [s] acceleration filter time constant
        public double TorqueHighPassFilterTimeConstantZTorque { get; set; } = 20;  // [s] torque high pass filter time constant
        public double TorqueLowPassFilterTimeConstantZTorque { get; set; } = 0.005;// [s] torque low pass filter time constant
        public double AdditionalTuningFactorZTorque { get; set; } = 0.25;          // additional ZTorque tuning factor between 0 - 1
        public double InertiaCorrectionFactorZTorque { get; set; } = 1.00;         // correction factor for estimated inertia

        public double KpSoftTorqueSpeed { get; set; } = 4;                         // Soft torque/speed
        public double KiSoftTorqueSpeed { get; set; } = 0;
        public double TuningFrequenceySoftTorqueSpeed { get; set; } = 0.2;         // [Hz] tuning frequency
        public double KpStiffAndZTorque { get; set; } = 50;
        public double KiStiffAndZTorque { get; set; } = 5;

    }
}
