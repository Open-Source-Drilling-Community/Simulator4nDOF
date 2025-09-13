using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Scalars
    {
        public double AvgCumulativeSSI { get; set; } = 0.0;

        // Time based values
        public List<double> BitAxialVelocity { get; set; } = new List<double>();
        public List<double> TopOfStringAxialVelocity { get; set; } = new List<double>();
        public List<double> WOB { get; set; } = new List<double>();
        public List<double> BitRPM { get; set; } = new List<double>();
        public List<double> SurfaceRPM { get; set; } = new List<double>();
        public List<double> BitDepth { get; set; } = new List<double>();
        public List<double> HoleDepth { get; set; } = new List<double>();
        public List<double> SurfaceTorque { get; set; } = new List<double>();
        public List<double> BitTorque { get; set; } = new List<double>();
        public List<double> SensorAngularVelocity { get; set; } = new List<double>();
        public List<double> SensorWhirlVelocity { get; set; } = new List<double>();
        public List<double> SensorAxialVelocity { get; set; } = new List<double>();
        public List<double> SensorRadialVelocity { get; set; } = new List<double>();
        public List<double> SensorRadialAcc { get; set; } = new List<double>();
        public List<double> SensorTangentialAcc { get; set; } = new List<double>();
        public List<double> SensorAxialAcc { get; set; } = new List<double>();
        public List<double> SensorBendingMomentX { get; set; } = new List<double>();
        public List<double> SensorBendingMomentY { get; set; } = new List<double>();

        public List<double> SensorTension { get; set; } = new List<double>();
        public List<double> SensorTorque { get; set; } = new List<double>();

        public List<double> Time { get; set; } = new List<double>();
        public List<double> SSI { get; set; } = new List<double>();
    }
}
