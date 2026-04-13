using MathNet.Numerics.LinearAlgebra; 
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class State
    {
        public bool SimulationDiverged { get; set; } = false;

        public Vector<double> SleeveAngularVelocity;          // Sleeve angular velocity
        public Vector<double> DepthOfCut;                     // Depth of cut

        // State variables
        public Vector<double> SleeveAngularDisplacement;      // Sleeve angular displacement
        public Vector<double> SleeveAngularAcceleration;      // Sleeve angular acceleration
        public Vector<double> XDisplacement;                  // Lumped element lateral displacement in x-direction
        public Vector<double> XVelocity;                      // Lumped element lateral velocity in x-direction
        public Vector<double> XAcceleration;                  // Lumped element lateral acceleration in x-direction
        public Vector<double> YDisplacement;                  // Lumped element lateral displacement in y-direction
        public Vector<double> YVelocity;                      // Lumped element lateral velocity in y-direction
        public Vector<double> YAcceleration;                  // Lumped element lateral acceleration in y-direction
        public Vector<double> ZDisplacement;                  // Lumped element lateral acceleration in x-direction        
        public Vector<double> ZVelocity;                        // Lumped element axial velocity
        public Vector<double> ZAcceleration;                    // Lumped element axial acceleration
        public double TopOfStringRelativeAxialPosition;               // Top of string relative position
        public double TopOfStringRelativeRotation;               // Top of string relative rotation
        

        public Vector<double> AngularDisplacement;            // Lumped element angular displacement        
        public Vector<double> AngularVelocity;                // Lumped element angular velocity
        public Vector<double> AngularAcceleration;            // Lumped element angular acceleration
        // Auxiliar state variables
        public Vector<double> WhirlAngle;                     // Lumped element whirl angle (angular displacement)
        public Vector<double> WhirlVelocity;                  // Lumped element whirl velocity (angular velocity)        
        public Vector<double> RadialDisplacement;             // Lumped element radial displacement
        public Vector<double> RadialVelocity;                 // Lumped element radial velocity
        //Torsional Axial variables
        //public Vector<double> DownwardTorsionalWave; // Downward traveling wave, torsional
        //public Vector<double> UpwardTorsionalWave; // Upward traveling wave, torsional
        //public Vector<double> DownwardAxialWave; // Downward traveling wave, axial
        //public Vector<double> UpwardAxialWave; // Upward traveling wave, axial
        //public Vector<double> DownwardTorsionalWaveLeftBoundary;
        //public Vector<double> UpwardTorsionalWaveRightBoundary;
        //public Vector<double> DownwardAxialWaveLeftBoundary;
        //public Vector<double> UpwardAxialWaveRightBoundary;
        // Bit interaction generalized forces        
        public double WeightOnBit;
        public double TorqueOnBit;
        // Top Drive state variables
        public TopDriveAndDrawworkState TopDrive;
        //public double TopDriveAngularVelocity;                // Top drive angular velocity        
        //public double TopDriveMotorTorque;                           // Top drive motor torque
        //public double TopDriveRPMSetPoint;
        public Vector<double> SleeveForces;                   // Sleeve forces
        public List<int> SleeveToLumpedIndex;                // Mapping from sleeve indices to lumped element indices
        public Vector<double> SlipCondition;                 // Slip condition evaluated at each lumped element

   
        // Mud motor stator and rotor angular velocities
        public double MudStatorAngularVelocity;               // Mud motor stator angular velocity
        public double MudRotorAngularVelocity;                // Mud motor rotor angular velocity

        // Depths and positions
        public double BitDepth;                               // Bit depth
        public double HoleDepth;                              // Hole depth
        public double TopOfStringPosition;                    // Top of string position
        public double BitVelocity;                            // Bit velocity
        // State outputs for reconstruction
        public Vector<double> BendingMomentX;
        public Vector<double> BendingMomentY;
        public Vector<double> NormalCollisionForce;        
        public Vector<double> SoftStringNormalForce;
        public Vector<double> Tension;
        public Vector<double> Torque;
        public double MudTorque;
        public double PhiDdotNoSlipSensor;
        public double ThetaDotNoSlipSensor;

        // Simulation flags and indices
        public double PreviousCalculatedBitDepth;             // Previous calculated bit depth (used for trajectory update)
        public bool MakeConnection;                          // Flag indicating if making a connection
        public bool PullOutBeforeConnectionBool;                   // Flag for POOH before connection
        public double OnBottomStart;                      // Start index of on-bottom in results array
        public bool BitOnBotton;                                 // Flag indicating if on bottom
        public int Step;                                      // Simulation step counter



        // Timestamps
        public double ConnectionStartTime;                     // Connection start time [s]
        public double TopDriveStartupTime;                     // Top drive startup time [s]

        public State(in SimulationParameters simulationParameters)
        {
            
            

            // Initialize lumped element whirl angle
            WhirlAngle = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element radial displacement
            RadialDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element radial velocity
            RadialVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element whirl velocity
            WhirlVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize top drive angular velocity
            TopDrive = new TopDriveAndDrawworkState()
            {
                TopDriveAngularVelocity = simulationParameters.TopDriveDrawwork.SurfaceRotation,
                TopDriveMotorTorque = simulationParameters.TopDriveDrawwork.TopDriveMotorTorque,
                MaximumTopDriveTorque = simulationParameters.TopDriveDrawwork.MaximumTopDriveTorque,
                TopDriveRPMSetPoint = simulationParameters.TopDriveDrawwork.TopDriveRPMSetPoint
            };
            // Initialize lumped element angular displacement
            AngularDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element angular velocity
            AngularVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element angular acceleration
            AngularAcceleration = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element axial displacement
            ZDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            for (int i = 0; i < simulationParameters.LumpedCells.NumberOfLumpedElements; i++)
            {
                ZDisplacement[i] = simulationParameters.LumpedCells.CumulativeElementLength[i];
            }
            // Initialize lumped element axial velocity
            ZVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element axial acceleration
            ZAcceleration = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral displacement in x-direction
            XDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral velocity in x-direction
            XVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral acceleration in x-direction
            XAcceleration = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral displacement in y-direction
            YDisplacement = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral velocity in y-direction
            YVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize lumped element lateral acceleration in y-direction
            YAcceleration = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            // Initialize sleeve angular displacement
            SleeveAngularDisplacement = Vector<double>.Build.Dense(simulationParameters.Drillstring.TotalSleeveNumber);
            // Initialize sleeve angular velocity
            SleeveAngularVelocity = Vector<double>.Build.Dense(simulationParameters.Drillstring.TotalSleeveNumber);
            // Initialize sleeve angular acceleration
            SleeveAngularAcceleration = Vector<double>.Build.Dense(simulationParameters.Drillstring.TotalSleeveNumber);
            // Initialize depth of cut
            DepthOfCut = Vector<double>.Build.Dense(simulationParameters.DistributedCells.CellsInDepthOfCut);
            // Initialize sleeve forces
            SleeveForces = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            TopOfStringRelativeAxialPosition = 0;
            TopOfStringRelativeRotation = 0;

            // Initialize slip condition
            SlipCondition = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            BendingMomentX = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            BendingMomentY = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            NormalCollisionForce = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements + 1);        
            SoftStringNormalForce = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements + 1);
            Tension = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements + 1);
            Torque = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements + 1);
            MudTorque = 0;
          
            // Initialize sleeve to lumped index mapping
            SleeveToLumpedIndex = new List<int>();
            for (int i = 0; i < simulationParameters.LumpedCells.NumberOfLumpedElements; i++)
            {
                SleeveToLumpedIndex.Add(simulationParameters.Drillstring.SleeveIndexPosition.Contains(i) ? i : -1);
            }
            // Add 1 to every positive number in SleeveToLumpedIndex
            for (int i = 0; i < SleeveToLumpedIndex.Count; i++)
            {
                if (SleeveToLumpedIndex[i] > 0)
                {
                    SleeveToLumpedIndex[i] += 1;
                }
            }

            MudStatorAngularVelocity = 0;
            MudRotorAngularVelocity = 0;

            this.BitDepth = simulationParameters.Input.InitialBitDepth;                               // Initial values set in SimulationParameters - to be moved
            PreviousCalculatedBitDepth = simulationParameters.Input.InitialBitDepth;
            this.HoleDepth = simulationParameters.Input.InitialHoleDepth;
            this.TopOfStringPosition = simulationParameters.Input.InitialTopOfStringPosition;
            this. ZVelocity[0] = simulationParameters.Input.InitialTopOfStringVelocity;
            BitOnBotton = false;

            MakeConnection = false;
            PullOutBeforeConnectionBool = false;
            ConnectionStartTime = 0;                                 // [s] connection start time
            TopDriveStartupTime = 0;
            OnBottomStart = -1;
          }
        public void AddNewLumpedElement()
        {
        
            // Extend AngularDisplacement vector by prepending the first element
            AngularDisplacement = ExtendVectorStart(AngularDisplacement[0], AngularDisplacement);

            // Extend AngularVelocity vector by prepending the first element
            AngularVelocity = ExtendVectorStart(AngularVelocity[0], AngularVelocity);

            // Extend AngularAcceleration vector by prepending the first element
            AngularAcceleration = ExtendVectorStart(AngularAcceleration[0], AngularAcceleration);

            // Extend AxialVelocity vector by prepending the first element
            ZVelocity = ExtendVectorStart(ZVelocity[0], ZVelocity);

            // Extend AxialAcceleration vector by prepending the first element
            ZAcceleration = ExtendVectorStart(ZAcceleration[0], ZAcceleration);

            // Extend XDisplacement vector by prepending the first element
            XDisplacement = ExtendVectorStart(XDisplacement[0], XDisplacement);

            // Extend XVelocity vector by prepending the first element
            XVelocity = ExtendVectorStart(XVelocity[0], XVelocity);

            // Extend XAcceleration vector by prepending the first element
            XAcceleration = ExtendVectorStart(XAcceleration[0], XAcceleration);

            // Extend YDisplacement vector by prepending the first element
            YDisplacement = ExtendVectorStart(YDisplacement[0], YDisplacement);

            // Extend YVelocity vector by prepending the first element
            YVelocity = ExtendVectorStart(YVelocity[0], YVelocity);

            // Extend YAcceleration vector by prepending the first element
            YAcceleration = ExtendVectorStart(YAcceleration[0], YAcceleration);

            // Extend WhirlAngle vector by prepending the first element
            WhirlAngle = ExtendVectorStart(WhirlAngle[0], WhirlAngle);

            // Extend WhirlVelocity vector by prepending the first element
            WhirlVelocity = ExtendVectorStart(WhirlVelocity[0], WhirlVelocity);

            // Extend RadialDisplacement vector by prepending the first element
            RadialDisplacement = ExtendVectorStart(RadialDisplacement[0], RadialDisplacement);

            // Extend RadialVelocity vector by prepending the first element
            RadialVelocity = ExtendVectorStart(RadialVelocity[0], RadialVelocity);

            // Extend slip_condition vector by prepending the first element
            SlipCondition = ExtendVectorStart(SlipCondition[0], SlipCondition);

            // Extend SleeveForce vector by prepending the first element            
            SleeveForces = ExtendVectorStart(SleeveForces[0], SleeveForces);
            // Insert -1 at the beginning of SleeveToLumpedIndex to account for the new element
            SleeveToLumpedIndex.Insert(0, -1);


            // Add 1 to every positive number in SleeveToLumpedIndex to update indices after insertion
            for (int i = 0; i < SleeveToLumpedIndex.Count; i++)
            {
                if (SleeveToLumpedIndex[i] > 0)
                {
                    SleeveToLumpedIndex[i] += 1;
                }
            }
        }
    }

}
