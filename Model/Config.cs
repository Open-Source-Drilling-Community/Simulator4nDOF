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

        public bool UseHeave { get; set; } = false;                     // true - use heave, false - no heave
        public double HeaveAmplitude { get; set; } = 0.0;             // [m] heave amplitude(assumed constant over time)
        public double HeavePeriod { get; set; } = 11;              // [s] heave period(assumed constant over time)

        public double TopDriveStartupTime { get; set; } = 0;            // [s] time for starting top drive after connection

        public double LengthBetweenLumpedElements { get; set; } = 30;
    }
}
