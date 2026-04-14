using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;
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
        public SimulatorWellbore Wellbore;
        public SimulatorDrillString Drillstring;
        public SimulatorFlow Flow;
        // DrillString from the microservice. It is used as an input for model
        public Friction Friction;
        public TopDriveDrawwork TopDriveDrawwork;
        // Discretization properties
        public int NumberOfElements;
        public double DrillStringLength;
        // 
        public int CellsInDepthOfCut = 5;

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
            Drillstring = new SimulatorDrillString(configuration);

            NumberOfElements = Drillstring.ElementLength.Count;
            Wellbore = new SimulatorWellbore(in Drillstring, in configuration.CasingSection);
            Trajectory = new SimulatorTrajectory(Drillstring, configuration.Trajectory);
            Flow = new SimulatorFlow(configuration, LumpedCells, Trajectory, Drillstring);
            MudMotor = new MudMotor();
            Friction = new Friction(LumpedCells, configuration.CoulombStaticFriction, configuration.CoulombKineticFriction, configuration.Stribeck);
            DrillStringLength = configuration.BitDepth - configuration.TopOfStringPosition;
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
           
            double dtTemp = 1E-4;  // Needs to be changed
            dxl = 1.0 / CellsInDepthOfCut;
            // That is the depth of cut minimum time-step
            dtl = dxl / (10.0 * TopDriveDrawwork.SurfaceRotation);  // The CFL condition here will use 10 times the top speed for the depth of cut PDE
            
            OuterLoopTimeStep = configuration.TimeStep; // time step of outer loop, which updates the distributed cells and calculates the bit forces
            InnerLoopIterations = (int)Math.Max(Math.Ceiling(configuration.TimeStep / dtTemp), Math.Ceiling(configuration.TimeStep / dtl)); // number of iterations in the inner loop
            InnerLoopTimeStep = configuration.TimeStep / InnerLoopIterations; // time step of inner loop      
            UseMudMotor = configuration.UseMudMotor;
            MovingDrillstring = configuration.MovingDrillstring;
            UseBuoyancyFactor = configuration.UseBuoyancyFactor;
            UsePipeMovementReconstruction = configuration.UsePipeMovementReconstruction;
            SolverType = configuration.SolverType;
        }

        public void AddNewLumpedElement()
        {
            // Insert first inactive element at the top of each active list
            Drillstring.ElementLength.Insert(0, Drillstring.InactiveElementLength[0]);
            Drillstring.ElementDensity.Insert(0, Drillstring.InactiveElementDensity[0]);
            Drillstring.ElementOuterRadius.Insert(0, Drillstring.InactiveElementOuterRadius[0]);
            Drillstring.ElementInnerRadius.Insert(0, Drillstring.InactiveElementInnerRadius[0]);
            Drillstring.ElementEccentricity.Insert(0, Drillstring.InactiveElementEccentricity[0]);
            Drillstring.ElementOuterArea.Insert(0, Drillstring.InactiveElementOuterArea[0]);
            Drillstring.ElementInnerArea.Insert(0, Drillstring.InactiveElementInnerArea[0]);
            Drillstring.ToolJointOuterArea.Insert(0, Drillstring.InactiveToolJointOuterArea[0]);
            Drillstring.ToolJointInnerArea.Insert(0, Drillstring.InactiveToolJointInnerArea[0]);
            Drillstring.ElementPolarInertia.Insert(0, Drillstring.InactiveElementPolarInertia[0]);
            Drillstring.ElementArea.Insert(0, Drillstring.InactiveElementArea[0]);
            Drillstring.ElementInertia.Insert(0, Drillstring.InactiveElementInertia[0]);
            Drillstring.WeightCorrectionFactor.Insert(0, Drillstring.InactiveWeightCorrectionFactor[0]);
            Drillstring.ElementYoungModuli.Insert(0, Drillstring.InactiveElementYoungModuli[0]);
            Drillstring.ElementShearModuli.Insert(0, Drillstring.InactiveElementShearModuli[0]);
            Drillstring.ElementFluidAddedMass.Insert(0, Drillstring.InactiveElementFluidAddedMass[0]);
            Drillstring.ElementEccentricMass.Insert(0, Drillstring.InactiveElementEccentricMass[0]);

            // Remove the first element from each inactive list, only if it has more than 1 element
            if (Drillstring.InactiveElementLength.Count > 1) Drillstring.InactiveElementLength.RemoveAt(0);
            if (Drillstring.InactiveElementDensity.Count > 1) Drillstring.InactiveElementDensity.RemoveAt(0);
            if (Drillstring.InactiveElementOuterRadius.Count > 1) Drillstring.InactiveElementOuterRadius.RemoveAt(0);
            if (Drillstring.InactiveElementInnerRadius.Count > 1) Drillstring.InactiveElementInnerRadius.RemoveAt(0);
            if (Drillstring.InactiveElementEccentricity.Count > 1) Drillstring.InactiveElementEccentricity.RemoveAt(0);
            if (Drillstring.InactiveElementOuterArea.Count > 1) Drillstring.InactiveElementOuterArea.RemoveAt(0);
            if (Drillstring.InactiveElementInnerArea.Count > 1) Drillstring.InactiveElementInnerArea.RemoveAt(0);
            if (Drillstring.InactiveToolJointOuterArea.Count > 1) Drillstring.InactiveToolJointOuterArea.RemoveAt(0);
            if (Drillstring.InactiveToolJointInnerArea.Count > 1) Drillstring.InactiveToolJointInnerArea.RemoveAt(0);
            if (Drillstring.InactiveElementPolarInertia.Count > 1) Drillstring.InactiveElementPolarInertia.RemoveAt(0);
            if (Drillstring.InactiveElementArea.Count > 1) Drillstring.InactiveElementArea.RemoveAt(0);
            if (Drillstring.InactiveElementInertia.Count > 1) Drillstring.InactiveElementInertia.RemoveAt(0);
            if (Drillstring.InactiveWeightCorrectionFactor.Count > 1) Drillstring.InactiveWeightCorrectionFactor.RemoveAt(0);
            if (Drillstring.InactiveElementYoungModuli.Count > 1) Drillstring.InactiveElementYoungModuli.RemoveAt(0);
            if (Drillstring.InactiveElementShearModuli.Count > 1) Drillstring.InactiveElementShearModuli.RemoveAt(0);
            if (Drillstring.InactiveElementFluidAddedMass.Count > 1) Drillstring.InactiveElementFluidAddedMass.RemoveAt(0);
            if (Drillstring.InactiveElementEccentricMass.Count > 1) Drillstring.InactiveElementEccentricMass.RemoveAt(0);
            

            NumberOfElements = Drillstring.ElementLength.Count;
            // Fields without Inactive counterparts
            Friction.StaticFrictionCoefficient = ExtendVectorStart(Friction.StaticFrictionCoefficient[0], Friction.StaticFrictionCoefficient);
            Friction.KinematicFrictionCoefficient = ExtendVectorStart(Friction.KinematicFrictionCoefficient[0], Friction.KinematicFrictionCoefficient);
            LumpedCells.CumulativeElementLength = ExtendVectorStart(0, LumpedCells.CumulativeElementLength);
            LumpedCells.NumberOfLumpedElements = LumpedCells.NumberOfLumpedElements + 1;
            if (Drillstring.SleeveIndexPosition.Count > 0)
            {
                for (int i = 0; i < Drillstring.SleeveIndexPosition.Count; i++)
                {
                    Drillstring.SleeveIndexPosition[i] += 1;
                }
            }
        }
    }
}