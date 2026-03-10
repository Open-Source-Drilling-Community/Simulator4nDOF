using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Configuration
    {
        public double TimeStep = 0.01;                      // TimeStep, [s] sampling frequency of outer loop
        public bool UseMudMotor = false;                    // true - simulate with PDM downhole motor, false - no downhole motor
        public bool UsePipeMovementReconstruction = true;   // true - compute accelerations and bending moments at a specified sensor location along the string using pipe movement reconstruction, false - no acceleration and bending moment calculations
        public bool MovingDrillstring = true;               // true - drillstring can move along wellbore, false - drillstring is fixed with the wellbore
        public int TopDriveController = 1;                  // 1 - stiff PI controller, 2 - tuned PI controller(SoftTorque/SoftSpeed), 3 - ZTorque
        public bool UseHeave = true;                       // true - simulate rig heave, false - no rig heave
        public bool UseBuoyancyFactor = true;               // true - use buoyancy factor in calcuation of buoyancy forces, false - use pressure integral method
        public double HeaveAmplitude = 0.07;                // [m] heave amplitude(assumed constant over time)
        public double HeavePeriod = 11;                     // [s] heave period(assumed constant over time)
        public double ConnectionTime = 60;                  // [s] time needed for making a connection
        public double TopdriveStartupTime = 5;              // [s] time for starting top drive after connection                                    
        public int SSIWindowSize = 20;                      // [s] downhole RPM window size for SSI calculation

        public double RockStrengthEpsilon = 100e6;          // [Pa] Rock strength parameter(used in both Detournay model), br.epsilon
        public double BitWearLength = 5e-3;                 // [m] Wear flat length(Detournay model), br.l
        public double BitRockFrictionCoeff = 0.9;           // [-] Bit - rock friction coefficient(used in both Detournay and MSE model), br.mu
        public double PdcBladeNo = 5;                       // [-] Number of blades of the PDC bit(Detournay model), br.N

        public double LengthBetweenLumpedElements = 5;      // [m] Length between two lumped elements, lc.dxL. 5 originally, may becompe unstable if increased to 30
        public double DrillPipeLumpedElementLength = 1.3;   // [m] Drill pipe lumped element length, lc.l_dp

        public required double SensorDistanceFromBit = 63;  // [m] axial distance(relative to bit depth) of an IMU measuring downhole RPM and accelerations
        public required Vector<double> SleeveDistancesFromBit; // [m] Sleeve distances from bit
        public double SleeveDamping = 2.0e2;                // [N.s/rad] Sleeve torsional damping coefficient, ds.k_ec

        public double TorsionalDamping = 1.0;               //[N.m.s/rad] Torsional damping, ds.kt_factor
        public double AxialDamping = 1.0;                   //[N.s/m] Axial damping, ds.ka_factor
        public double LateralDamping = 0.0;                 //[N.s/m] Lateral damping, ds.kl_factor

        public double TopDriveMomentOfInertia = 2900;       // [kg.m^2] Top drive mass moment of inertia
        public double FluidDamping = 3000.0;                // [N.s/m] Fluid damping coefficient for lateral dynamics
        public double FluidTemperature {get; set;} = 323;
        public double CoulombStaticFriction = 0.3;          // [-] Coulomb static friction coefficient, f.mu_s_factor
        public double CoulombKineticFriction = 0.2;         // [-] coulomb kinetic friction coefficient, f.mu_k_factor
        public double Stribeck = 5;                         // [s/rad] Parameter controlling smoothness of transition from static to kinetic friction (Stribeck model)


        public required double FluidDensity  = 1200;        // [kg/m3] Density of drilling mud, f.rhoMud
        public double SurfacePressure;

        // Top drive ZTorque controller parameters
        public double VFDFilterTimeconstantZTorque = 0.005;         // [s] VFD filter time constant
        public double EncoderTimeConstantZTorque = 0.002;           // [s] encoder time constant
        public double AccelerationFilterTimeConstantZTorque = 0.005; // [s] acceleration filter time constant
        public double TorqueHighPassFilterTimeConstantZTorque = 20;  // [s] torque high pass filter time constant
        public double TorqueLowPassFilterTimeConstantZTorque = 0.005;// [s] torque low pass filter time constant
        public double AdditionalTuningFactorZTorque = 0.25;          // additional ZTorque tuning factor between 0 - 1
        public double InertiaCorrectionFactorZTorque = 1.00;        // correction factor for estimated inertia

        // Top drive controller gains
        public double KpSoftTorqueSpeed = 4;                       // Soft torqee/speed
        public double KiSoftTorqueSpeed = 0; 
        public double TuningFrequenceySoftTorqueSpeed = 0.2;       // [Hz] tuning frequency
        public double KpStiffAndZTorque = 50;                               
        public double KiStiffAndZTorque = 5;

        public required double BitDepth;
        public required double HoleDepth;
        public required double TopOfStringPosition;
        public required double SurfaceRPM;
        public required double TopOfStringVelocity;
        public required ModelShared.Trajectory Trajectory;
        public required ModelShared.DrillString DrillString;
        public required ModelShared.DrillingFluidDescription DrillingFluidDescription;        
        public ModelShared.CasingSection CasingSection;
        public required double BitRadius;
        public BitRockModelEnum BitRockModelEnum { get; set; } = BitRockModelEnum.Detournay;
        public SolverType SolverType { get; set; } = SolverType.VerletMethod;
    }

}
