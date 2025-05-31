using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Profiles
    {
        public double Time { get; set; } = 0.0;

        // depth based values
        public List<double> Depth { get; set; } = new List<double>() { };
        public List<double> DepthAll { get; set; } = new List<double>() { };
        public List<double> SleevesDepth { get; set; } = new List<double>() { };
        public List<double> SideForce { get; set; } = new List<double>() { };
        public List<double> SideForceSoftString { get; set; } = new List<double>() { };
        public List<double> PipeAngularVelocity { get; set; } = new List<double>() { };
        public List<double> SleevesAngularVelocity { get; set; } = new List<double>() { };
        public List<double> RadialClearance { get; set; } = new List<double>() { };
        public List<double> LateralDisplacement { get; set; } = new List<double>() { };
        public List<double> LateralDisplacementAngle { get; set; } = new List<double>() { };
        public List<double> BendingMoment { get; set; } = new List<double>() { };
        public List<double> Torque { get; set; } = new List<double>() { };
        public List<double> Tension { get; set; } = new List<double>() { };
        public List<double> AxialVelocityD { get; set; } = new List<double>() { };
    }
}
