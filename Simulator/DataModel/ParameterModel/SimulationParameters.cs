using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;

using NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using System.Numerics;


namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulationParameters
    {
        //Simulation metadata
        public bool UseMudMotor;
        public bool MovingDrillstring {get; set;} = true;
        public Input Input;
        public IBitRock BitRock;
        public MudMotor MudMotor;
        public SimulatorTrajectory Trajectory;
        public LumpedCells LumpedCells;
        public DistributedCells DistributedCells;
        public SimulatorWellbore Wellbore;
        public SimulatorDrillString Drillstring;
        public SimulatorFlow Flow;
        public Friction Friction;
        public BitRockModelEnum BitRockModelEnum;
        public TopDriveDrawwork TopDriveDrawwork;
        //Integration properties
        public double OuterLoopTimeStep;
        public double InnerLoopTimeStep;
        public int InnerLoopIterations;
        public double dxl;
        public double dtl;
        public SolverType SolverType {get; set;}
        //Buyoancy
        public bool UseBuoyancyFactor {get; set;} = true;
        //Output properties
        public bool UsePipeMovementReconstruction {get; set;} = true;
        

        public SimulationParameters(DataModel.Configuration configuration)
        {
            Input = new Input()
            {
                InitialBitDepth = configuration.BitDepth,
                InitialHoleDepth = configuration.HoleDepth,
                InitialTopOfStringPosition = configuration.TopOfStringPosition,
                InitialTopOfStringVelocity = configuration.TopOfStringVelocity
            };
            LumpedCells = new LumpedCells(configuration.BitDepth, configuration.LengthBetweenLumpedElements);
            Drillstring = new SimulatorDrillString(LumpedCells, 
                                 configuration.DrillString,
                                 configuration.FluidDensity,                                  
                                 configuration.BitDepth, 
                                 configuration.BitRadius, 
                                 configuration.SensorDistanceFromBit, 
                                 configuration.SleeveDistancesFromBit,
                                 configuration.SleeveDamping,
                                 configuration.DrillPipeLumpedElementLength,
                                 configuration.TorsionalDamping,
                                 configuration.AxialDamping,
                                 configuration.LateralDamping);
            Wellbore = new SimulatorWellbore(Drillstring,
                             LumpedCells,
                             configuration.CasingSection
                             );
            Trajectory = new SimulatorTrajectory(LumpedCells, configuration.Trajectory);
            Flow = new SimulatorFlow(configuration, LumpedCells, Trajectory, Drillstring);

            MudMotor = new MudMotor();
            DistributedCells = new DistributedCells(Drillstring, configuration.SurfaceRPM, 2, configuration.LengthBetweenLumpedElements);        

            Friction = new Friction(LumpedCells, configuration.CoulombStaticFriction, configuration.CoulombKineticFriction, configuration.Stribeck);
        
            double dtTemp = DistributedCells.ElementLength / Math.Max(Drillstring.TorsionalWaveSpeed, Drillstring.AxialWaveSpeed) * .4;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
            dxl = 1.0 / DistributedCells.CellsInDepthOfCut;
            dtl = dxl / DistributedCells.OmegaMax;  // As per the CFL condition for the depth of cut PDE
            OuterLoopTimeStep = configuration.TimeStep; // time step of outer loop, which updates the distributed cells and calculates the bit forces
            InnerLoopIterations = (int)Math.Max(Math.Ceiling(configuration.TimeStep / dtTemp), Math.Ceiling(configuration.TimeStep / dtl)); // number of iterations in the inner loop
            InnerLoopTimeStep = configuration.TimeStep / InnerLoopIterations; // time step of inner loop      
            UseMudMotor = configuration.UseMudMotor;
            TopDriveDrawwork = new TopDriveDrawwork()
            {
                SurfaceRotation = configuration.SurfaceRPM,
                SurfaceAxialVelocity = configuration.TopOfStringVelocity,
                TopDriveControllerType = configuration.TopDriveController,
                HeaveAmplitude = configuration.HeaveAmplitude,
                HeavePeriod = configuration.HeavePeriod,
                TopDriveStartupTime = configuration.TopdriveStartupTime,
                ConnectionTime = configuration.ConnectionTime
            };
            MovingDrillstring = configuration.MovingDrillstring;
            UseBuoyancyFactor = configuration.UseBuoyancyFactor;
            UsePipeMovementReconstruction = configuration.UsePipeMovementReconstruction;
            SolverType = configuration.SolverType;
        }

        public void AddNewLumpedElement()
        {
            double Pro = Math.Min(Drillstring.OuterRadius[0], Drillstring.OuterRadius[1]);                          // [m] Drill pipe outer radius
            double Pri = Math.Max(Drillstring.InnerRadius[0], Drillstring.InnerRadius[1]);                          // [m] Drill pipe inner radius
            double Pro_tj = Math.Max(Drillstring.OuterRadius[0], Drillstring.OuterRadius[1]);                       // [m] Tool joint outer radius
            double Pri_tj = Math.Min(Drillstring.InnerRadius[0], Drillstring.InnerRadius[1]);                       // [m] Tool joint inner radius
            double Jp = Math.PI / 2.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string polar moment of inertia
            double Ap = Math.PI * (Math.Pow(Pro, 2) - Math.Pow(Pri, 2));        // [m ^ 2] Drill string cross sectional area
            double Ip = Math.PI / 4.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string moment of inertia
            double Atj = Math.PI * (Math.Pow(Pro_tj, 2) - Math.Pow(Pri_tj, 2)); // [m ^ 2] Tool joint cross sectional area
            double Ao = Math.PI * Math.Pow(Pro, 2);                             // [m ^ 2] Drill pipe outer surface area
            double Ai = Math.PI * Math.Pow(Pri, 2);                             // [m ^ 2] Drill pipe inner surface area
            double Atjo = Math.PI * Math.Pow(Pro_tj, 2);                        // [m ^ 2] Tool joint outer surface area
            double Atji = Math.PI * Math.Pow(Pri_tj, 2);                        // [m ^ 2] Tool joint inner surface area
            double ecc_percent = Math.Max(Drillstring.Eccentricity[0], Drillstring.Eccentricity[1]) / Math.Max(Drillstring.OuterRadius[0], Drillstring.OuterRadius[1]); // eccentricity percent relative to total radius
            double mass_imbalance_percent = Drillstring.EccentricMass[0] / Drillstring.LumpedElementMass[0];

            // if current top element is a drill pipe, add a tool joint, and vice - versa
            if (Drillstring.OuterRadius[0] == Pro)
            {
                // Copy the original array elements to the new array, starting at index 1
                Drillstring.OuterRadius = ExtendVectorStart(Pro_tj, Drillstring.OuterRadius);
                Drillstring.InnerRadius = ExtendVectorStart(Pri_tj, Drillstring.InnerRadius);
                Drillstring.Eccentricity = ExtendVectorStart(Pro_tj * ecc_percent, Drillstring.Eccentricity);
                Drillstring.LumpedElementMass = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Ap + Drillstring.SteelDensity * Drillstring.ToolJointLength * Atj, Drillstring.LumpedElementMass);
            }
            else
            {
                Drillstring.OuterRadius = ExtendVectorStart(Pro, Drillstring.OuterRadius);
                Drillstring.InnerRadius = ExtendVectorStart(Pri, Drillstring.InnerRadius);
                Drillstring.Eccentricity = ExtendVectorStart(0, Drillstring.Eccentricity);
                Drillstring.LumpedElementMass = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Ap, Drillstring.LumpedElementMass);
            }

            Drillstring.PipePolarMoment = ExtendVectorStart(Jp, Drillstring.PipePolarMoment);
            Drillstring.PipeArea = ExtendVectorStart(Ap, Drillstring.PipeArea);
            Drillstring.PipeInertia = ExtendVectorStart(Ip, Drillstring.PipeInertia);
            Drillstring.OuterArea = ExtendVectorStart(Ao, Drillstring.OuterArea);
            Drillstring.InnerArea = ExtendVectorStart(Ai, Drillstring.InnerArea);
            Drillstring.ToolJointOuterArea = ExtendVectorStart(Atjo, Drillstring.ToolJointOuterArea);
            Drillstring.ToolJointInnerArea = ExtendVectorStart(Atji, Drillstring.ToolJointInnerArea);
            Drillstring.YoungModuli = ExtendVectorStart(Drillstring.YoungModuli[0], Drillstring.YoungModuli);
            Drillstring.ShearModuli = ExtendVectorStart(Drillstring.ShearModuli[0], Drillstring.ShearModuli);
            Drillstring.WeightCorrectionFactor = ExtendVectorStart(Drillstring.WeightCorrectionFactor[0], Drillstring.WeightCorrectionFactor);
            Drillstring.LumpedElementMassMomentOfInertia = ExtendVectorStart(Drillstring.LumpedElementMassMomentOfInertia[0], Drillstring.LumpedElementMassMomentOfInertia);
            Drillstring.LumpedElementMomentOfInertia = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Jp, Drillstring.LumpedElementMomentOfInertia);
            Friction.StaticFrictionCoefficient = ExtendVectorStart(Friction.StaticFrictionCoefficient[0], Friction.StaticFrictionCoefficient);
            Friction.KinematicFrictionCoefficient = ExtendVectorStart(Friction.KinematicFrictionCoefficient[0], Friction.KinematicFrictionCoefficient);
            Drillstring.BendingStiffness = ExtendVectorStart(Drillstring.BendingStiffness[0], Drillstring.BendingStiffness);
            Drillstring.FluidAddedMass = ExtendVectorStart(Math.PI * Flow.FluidDensity * (Math.Pow(Drillstring.InnerRadius[0], 2) + Drillstring.AddedFluidMassCoefficient * Math.Pow(Drillstring.OuterRadius[0], 2)) * Drillstring.LumpedElementMassMomentOfInertia[0] / 2.0, Drillstring.FluidAddedMass);
            Drillstring.EccentricMass = ExtendVectorStart(mass_imbalance_percent * Drillstring.LumpedElementMass[0], Drillstring.EccentricMass);

            // Update spatial variables
            //int Pt_old = LumpedCells.DistributedToLumpedRatio * LumpedCells.NumberOfLumpedElements;
            //int NL_old = LumpedCells.NumberOfLumpedElements;

            // Generate the range from 0 to DistributedCells.x[0] with step size dx
            //List<double> range = new List<double>();
            //for (double value = 0; value <= DistributedCells.x[0]; value += DistributedCells.DistributedSectionAndLumpedLength)
            //{
            //    range.Add(value);
            //}
            //DistributedCells.x = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(range.Concat(DistributedCells.x).ToArray());
            LumpedCells.CumulativeElementLength = ExtendVectorStart(0, LumpedCells.CumulativeElementLength); // lumped section
            LumpedCells.NumberOfLumpedElements = LumpedCells.NumberOfLumpedElements + 1;
            if (Drillstring.SleeveIndexPosition.Count > 0)
            {
                // Increment each element in Drillstring.iS by 1
                for (int i = 0; i < Drillstring.SleeveIndexPosition.Count; i++)
                {
                    Drillstring.SleeveIndexPosition[i] += 1;
                }
            }
        }
    }
}