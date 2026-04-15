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
        public SimulatorWellbore Wellbore;
        public SimulatorDrillString Drillstring;
        public SimulatorFlow Flow;
        // DrillString from the microservice. It is used as an input for model
        public Friction Friction;
        public TopDriveDrawwork TopDriveDrawwork;
        // Discretization properties
        public int NumberOfElements;
        public int NumberOfNodes; 
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
            Drillstring = new SimulatorDrillString(configuration);

            NumberOfElements = Drillstring.ElementLength.Count;
            NumberOfNodes = Drillstring.ElementLength.Count + 1;
            Wellbore = new SimulatorWellbore(in Drillstring, in configuration.CasingSection);
            Trajectory = new SimulatorTrajectory(Drillstring, configuration.Trajectory);
            Flow = new SimulatorFlow(configuration, Trajectory, Drillstring);
            MudMotor = new MudMotor();
            Friction = new Friction(NumberOfNodes, configuration.CoulombStaticFriction, configuration.CoulombKineticFriction, configuration.Stribeck);
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

        public void AddNewElement()
        {
            // Insert first inactive element at the top of each active list
            Drillstring.ActivateElements();
           

            NumberOfElements = Drillstring.ElementLength.Count;
            // Fields without Inactive counterparts
            Friction.StaticFrictionCoefficient = ExtendVectorStart(Friction.StaticFrictionCoefficient[0], Friction.StaticFrictionCoefficient);
            Friction.KinematicFrictionCoefficient = ExtendVectorStart(Friction.KinematicFrictionCoefficient[0], Friction.KinematicFrictionCoefficient);
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