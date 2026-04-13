using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;


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
        // DrillString from the microservice. It is used as an input for model
        public DrillString DrillString;
        public Friction Friction;
        public BitRockModelEnum BitRockModelEnum;
        public TopDriveDrawwork TopDriveDrawwork;
        // Discretization properties
        public double NumberOfElements;
        public double ElementLength;
        public double DrillStringLength;
        
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
            LumpedCells = new LumpedCells(configuration.BitDepth, configuration.ElementLength);
            Drillstring = new SimulatorDrillString(LumpedCells, 
                                 configuration.DrillString,
                                 configuration.FluidDensity,                                  
                                 configuration.BitDepth, 
                                 configuration.BitRadius, 
                                 configuration.SensorDistanceFromBit, 
                                 configuration.SleeveDistancesFromBit,
                                 configuration.SleeveDamping,
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
            DistributedCells = new DistributedCells(Drillstring, configuration.SurfaceRPM, configuration.ElementLength);                    
            Friction = new Friction(LumpedCells, configuration.CoulombStaticFriction, configuration.CoulombKineticFriction, configuration.Stribeck);
            DrillStringLength = configuration.BitDepth - configuration.TopOfStringPosition;
            ElementLength = configuration.ElementLength;

            double dtTemp = DistributedCells.ElementLength / Math.Max(Drillstring.TorsionalWaveSpeed, Drillstring.AxialWaveSpeed) * 0.8;  // As per the CFL condition for the axial / torsional wave equations - change to 0.80 for better stability
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
            double Pro = Math.Min(Drillstring.ElementOuterRadius[0], Drillstring.ElementOuterRadius[1]);                          // [m] Drill pipe outer radius
            double Pri = Math.Max(Drillstring.ElementInnerRadius[0], Drillstring.ElementInnerRadius[1]);                          // [m] Drill pipe inner radius
            double Pro_tj = Math.Max(Drillstring.ElementOuterRadius[0], Drillstring.ElementOuterRadius[1]);                       // [m] Tool joint outer radius
            double Pri_tj = Math.Min(Drillstring.ElementInnerRadius[0], Drillstring.ElementInnerRadius[1]);                       // [m] Tool joint inner radius
            double Jp = Math.PI / 2.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string polar moment of inertia
            double Ap = Math.PI * (Math.Pow(Pro, 2) - Math.Pow(Pri, 2));        // [m ^ 2] Drill string cross sectional area
            double Ip = Math.PI / 4.0 * (Math.Pow(Pro, 4) - Math.Pow(Pri, 4));  // [m ^ 4] Drill string moment of inertia
            double Atj = Math.PI * (Math.Pow(Pro_tj, 2) - Math.Pow(Pri_tj, 2)); // [m ^ 2] Tool joint cross sectional area
            double Ao = Math.PI * Math.Pow(Pro, 2);                             // [m ^ 2] Drill pipe outer surface area
            double Ai = Math.PI * Math.Pow(Pri, 2);                             // [m ^ 2] Drill pipe inner surface area
            double Atjo = Math.PI * Math.Pow(Pro_tj, 2);                        // [m ^ 2] Tool joint outer surface area
            double Atji = Math.PI * Math.Pow(Pri_tj, 2);                        // [m ^ 2] Tool joint inner surface area
            double ecc_percent = Math.Max(Drillstring.ElementEccentricity[0], Drillstring.ElementEccentricity[1]) / Math.Max(Drillstring.ElementOuterRadius[0], Drillstring.ElementOuterRadius[1]); // eccentricity percent relative to total radius
            double mass_imbalance_percent = Drillstring.EccentricMass[0] / Drillstring.LumpedElementMass[0];

            // if current top element is a drill pipe, add a tool joint, and vice - versa
            if (Drillstring.ElementOuterRadius[0] == Pro)
            {
                // Copy the original array elements to the new array, starting at index 1
                Drillstring.ElementOuterRadius = ExtendVectorStart(Pro_tj, Drillstring.ElementOuterRadius);
                Drillstring.ElementInnerRadius = ExtendVectorStart(Pri_tj, Drillstring.ElementInnerRadius);
                Drillstring.ElementEccentricity = ExtendVectorStart(Pro_tj * ecc_percent, Drillstring.ElementEccentricity);
                Drillstring.LumpedElementMass = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Ap + Drillstring.SteelDensity * Drillstring.ToolJointLength * Atj, Drillstring.LumpedElementMass);
            }
            else
            {
                Drillstring.ElementOuterRadius = ExtendVectorStart(Pro, Drillstring.ElementOuterRadius);
                Drillstring.ElementInnerRadius = ExtendVectorStart(Pri, Drillstring.ElementInnerRadius);
                Drillstring.ElementEccentricity = ExtendVectorStart(0, Drillstring.ElementEccentricity);
                Drillstring.LumpedElementMass = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Ap, Drillstring.LumpedElementMass);
            }

            Drillstring.ElementPolarInertia = ExtendVectorStart(Jp, Drillstring.ElementPolarInertia);
            Drillstring.ElementArea = ExtendVectorStart(Ap, Drillstring.ElementArea);
            Drillstring.ElementInertia = ExtendVectorStart(Ip, Drillstring.ElementInertia);
            Drillstring.ElementOuterArea = ExtendVectorStart(Ao, Drillstring.ElementOuterArea);
            Drillstring.ElementInnerArea = ExtendVectorStart(Ai, Drillstring.ElementInnerArea);
            Drillstring.ToolJointOuterArea = ExtendVectorStart(Atjo, Drillstring.ToolJointOuterArea);
            Drillstring.ToolJointInnerArea = ExtendVectorStart(Atji, Drillstring.ToolJointInnerArea);
            Drillstring.ElementYoungModuli = ExtendVectorStart(Drillstring.ElementYoungModuli[0], Drillstring.ElementYoungModuli);
            Drillstring.ElementShearModuli = ExtendVectorStart(Drillstring.ElementShearModuli[0], Drillstring.ElementShearModuli);
            Drillstring.WeightCorrectionFactor = ExtendVectorStart(Drillstring.WeightCorrectionFactor[0], Drillstring.WeightCorrectionFactor);
            Drillstring.LumpedElementMassMomentOfInertia = ExtendVectorStart(Drillstring.LumpedElementMassMomentOfInertia[0], Drillstring.LumpedElementMassMomentOfInertia);
            Drillstring.LumpedElementMomentOfInertia = ExtendVectorStart(Drillstring.SteelDensity * Drillstring.LumpedElementMassMomentOfInertia[0] * Jp, Drillstring.LumpedElementMomentOfInertia);
            Friction.StaticFrictionCoefficient = ExtendVectorStart(Friction.StaticFrictionCoefficient[0], Friction.StaticFrictionCoefficient);
            Friction.KinematicFrictionCoefficient = ExtendVectorStart(Friction.KinematicFrictionCoefficient[0], Friction.KinematicFrictionCoefficient);
            Drillstring.BendingStiffness = ExtendVectorStart(Drillstring.BendingStiffness[0], Drillstring.BendingStiffness);
            Drillstring.FluidAddedMass = ExtendVectorStart(Math.PI * Flow.FluidDensity * (Math.Pow(Drillstring.ElementInnerRadius[0], 2) + Drillstring.AddedFluidMassCoefficient * Math.Pow(Drillstring.ElementOuterRadius[0], 2)) * Drillstring.LumpedElementMassMomentOfInertia[0] / 2.0, Drillstring.FluidAddedMass);
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