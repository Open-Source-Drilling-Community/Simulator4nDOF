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
        public double VirtualSensorPositionFromBit { get; set; }
        public double LoggingIntervalScalarValues { get; set; }
        public double LoggingIntervalProfiles { get; set; }

        public double CoulombStaticFriction  { get; set; }          // [-] Coulomb static friction coefficient, f.mu_s_factor
        public double CoulombKineticFriction { get; set; }          // [-] coulomb kinetic friction coefficient, f.mu_k_factor

        public double HeaveAmplitude { get; set; }                  // [m] heave amplitude(assumed constant over time)
        public double HeavePeriod { get; set; }                     // [s] heave period(assumed constant over time)

        public int TopDriveController { get; set; }                  // 1 - stiff PI controller, 2 - tuned PI controller(SoftTorque/SoftSpeed), 3 - ZTorque
    }
}
