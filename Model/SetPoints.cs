using OSDC.DotnetLibraries.Drilling.DrillingProperties;
using OSDC.DotnetLibraries.General.DataManagement;
using System;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class SetPoints
    {
        public double TimeDuration { get; set; }               // [sec]
        public double SurfaceRPM { get; set; }                 // [rad/s]
        public double TopOfStringVelocity { get; set; }        // [m/s] 

    }
}
