
namespace NORCE.Drilling.Simulator4nDOF.WebApp
{
    public class Results
    {
        public double AvgCumulativeSSI = 0.0;

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
        public List<double> Time { get; set; } = new List<double>();
        public List<double> SSI { get; set; } = new List<double>();

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
