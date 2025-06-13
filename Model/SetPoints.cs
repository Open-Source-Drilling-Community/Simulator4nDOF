using OSDC.DotnetLibraries.Drilling.DrillingProperties;
using OSDC.DotnetLibraries.General.DataManagement;
using System;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class SetPoints
    {
        public double TimeDuration { get; set; } = 1;              // [sec]
        public double SurfaceRPM { get; set; } = 0;                // [rad/s]
        public double TopOfStringVelocity { get; set; } = 0;       // [m/s] 

    }
}
