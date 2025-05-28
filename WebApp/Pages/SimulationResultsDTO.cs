
namespace NORCE.Drilling.Simulator4nDOF.WebApp
{
    public class SimulationResultsDTO
    {
        public double AvgCumulativeSSI = 0.0;

        // Time based values
        public List<double> BitAxialVelocity = new List<double>();
        public List<double> TopOfStringAxialVelocity = new List<double>();
        public List<double> WOB = new List<double>();
        public List<double> BitRPM = new List<double>();
        public List<double> SurfaceRPM = new List<double>();
        public List<double> BitDepth = new List<double>();
        public List<double> HoleDepth = new List<double>();
        public List<double> SurfaceTorque = new List<double>();
        public List<double> BitTorque = new List<double>();
        public List<double> SensorAngularVelocity = new List<double>();
        public List<double> SensorWhirlVelocity = new List<double>();
        public List<double> SensorAxialVelocity = new List<double>();
        public List<double> SensorRadialVelocity = new List<double>();
        public List<double> SensorRadialAcc = new List<double>();
        public List<double> SensorTangentialAcc = new List<double>();
        public List<double> SensorAxialAcc = new List<double>();
        public List<double> SensorBendingMomentX = new List<double>();
        public List<double> SensorBendingMomentY = new List<double>();
        public List<double> Time = new List<double>();
        public List<double> SSI = new List<double>();

        // depth based values
        public List<double> Depth = new List<double>() { };
        public List<double> DepthAll = new List<double>() { };
        public List<double> SleevesDepth = new List<double>() { };
        public List<double> SideForce = new List<double>() { };
        public List<double> SideForceSoftString = new List<double>() { };
        public List<double> PipeAngularVelocity = new List<double>() { };
        public List<double> SleevesAngularVelocity = new List<double>() { };
        public List<double> RadialClearance = new List<double>() { };
        public List<double> LateralDisplacement = new List<double>() { };
        public List<double> LateralDisplacementAngle = new List<double>() { };
        public List<double> BendingMoment = new List<double>() { };
        public List<double> Torque = new List<double>() { };
        public List<double> Tension = new List<double>() { };
        public List<double> AxialVelocityD = new List<double>() { };

    }
}
