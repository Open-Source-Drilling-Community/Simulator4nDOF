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
        // Axial-torsional state variables, defined at the axial-torsional nodes
        public Vector<double> ZDisplacement;                  // Axial-torsional node axial displacement
        public Vector<double> ZVelocity;                        // Axial-torsional node axial velocity
        public Vector<double> ZAcceleration;                    // Axial-torsional node axial acceleration
        public Vector<double> AngularDisplacement;            // Axial-torsional node angular displacement        
        public Vector<double> AngularVelocity;                // Axial-torsional node angular velocity
        public Vector<double> AngularAcceleration;            // Axial-torsional node angular acceleration
        // Axial-torsional state variables linearly interpolated to the lateral nodes
        public Vector<double> AxialDisplacementAtLateralNodes;                  // Axial displacement at the lateral nodes
        public Vector<double> AxialVelocityAtLateralNodes;                      // Axial velocity at the lateral nodes
        public Vector<double> AxialAccelerationAtLateralNodes;                  // Axial acceleration at the lateral nodes
        public Vector<double> AngularDisplacementAtLateralNodes;                // Angular displacement at the lateral nodes
        public Vector<double> AngularVelocityAtLateralNodes;                    // Angular velocity at the lateral nodes
        public Vector<double> AngularAccelerationAtLateralNodes;                // Angular acceleration at the lateral nodes
        // Auxiliar state variables
        public Vector<double> WhirlAngle;                     // Lumped element whirl angle (angular displacement)
        public Vector<double> WhirlVelocity;                  // Lumped element whirl velocity (angular velocity)        
        public Vector<double> RadialDisplacement;             // Lumped element radial displacement
        public Vector<double> RadialVelocity;                 // Lumped element radial velocity
        // Bit interaction generalized forces        
        public double WeightOnBit;
        public double TorqueOnBit;
        // Top Drive state variables
        public TopDriveAndDrawworkState TopDrive;
        public Vector<double> SleeveForces;                   // Sleeve forces
        public List<int> SleeveToLumpedIndex;                // Mapping from sleeve indices to lumped element indices
        public Vector<double> SlipCondition;                 // Slip condition evaluated at each lumped element

   
        // Mud motor stator and rotor angular velocities
        public double MudStatorAngularVelocity;               // Mud motor stator angular velocity
        public double MudRotorAngularVelocity;                // Mud motor rotor angular velocity

        // Depths and positions
        public double BitDepth;                               // Bit depth
        public double HoleDepth;                              // Hole depth
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
        private int numberOfNodes;
        private int numberOfElements;
        private int numberOfAxialNodes;
        public State(in SimulationParameters parameters)
        {
            numberOfElements = parameters.NumberOfElements;      
            numberOfNodes = parameters.NumberOfNodes;      
            numberOfAxialNodes = parameters.NumberOfAxialNodes;
            // Initialize lumped element whirl angle
            WhirlAngle = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element radial displacement
            RadialDisplacement = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element radial velocity
            RadialVelocity = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element whirl velocity
            WhirlVelocity = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize top drive angular velocity
            TopDrive = new TopDriveAndDrawworkState()
            {
                AngularVelocity = parameters.TopDriveDrawwork.SurfaceRotation,
                TopDriveMotorTorque = parameters.TopDriveDrawwork.TopDriveMotorTorque,
                MaximumTopDriveTorque = parameters.TopDriveDrawwork.MaximumTopDriveTorque,
                TopDriveRPMSetPoint = parameters.TopDriveDrawwork.TopDriveRPMSetPoint,
                AngularDisplacement = 0,
                RelativeAxialPosition = 0,
                AxialPosition = parameters.Input.InitialTopOfStringPosition,
                AxialVelocity = parameters.Input.InitialTopOfStringVelocity
            };
            // Initialize lumped element angular displacement
            AngularDisplacement = Vector<double>.Build.Dense(numberOfAxialNodes);
            AngularDisplacementAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element angular velocity
            AngularVelocity = Vector<double>.Build.Dense(numberOfAxialNodes);
            AngularVelocityAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element angular acceleration
            AngularAcceleration = Vector<double>.Build.Dense(numberOfAxialNodes);
            AngularAccelerationAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element axial displacement
            ZDisplacement = Vector<double>.Build.Dense(numberOfAxialNodes);
            AxialDisplacementAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            //for (int i = 0; i < numberOfElements; i++)
            //{
            //    ZDisplacement[i] = parameters.LumpedCells.CumulativeElementLength[i];
            //}
            // Initialize lumped element axial velocity
            ZVelocity = Vector<double>.Build.Dense(numberOfAxialNodes);
            AxialVelocityAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element axial acceleration
            ZAcceleration = Vector<double>.Build.Dense(numberOfAxialNodes);
            AxialAccelerationAtLateralNodes = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral displacement in x-direction
            XDisplacement = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral velocity in x-direction
            XVelocity = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral acceleration in x-direction
            XAcceleration = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral displacement in y-direction
            YDisplacement = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral velocity in y-direction
            YVelocity = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize lumped element lateral acceleration in y-direction
            YAcceleration = Vector<double>.Build.Dense(numberOfNodes);
            // Initialize sleeve angular displacement
            SleeveAngularDisplacement = Vector<double>.Build.Dense(parameters.Drillstring.TotalSleeveNumber);
            // Initialize sleeve angular velocity
            SleeveAngularVelocity = Vector<double>.Build.Dense(parameters.Drillstring.TotalSleeveNumber);
            // Initialize sleeve angular acceleration
            SleeveAngularAcceleration = Vector<double>.Build.Dense(parameters.Drillstring.TotalSleeveNumber);
            // Initialize depth of cut
            DepthOfCut = Vector<double>.Build.Dense(parameters.CellsInDepthOfCut);
            // Initialize sleeve forces
            SleeveForces = Vector<double>.Build.Dense(numberOfNodes);            
            // Initialize slip condition
            SlipCondition = Vector<double>.Build.Dense(numberOfNodes);
            BendingMomentX = Vector<double>.Build.Dense(numberOfNodes);
            BendingMomentY = Vector<double>.Build.Dense(numberOfNodes);
            NormalCollisionForce = Vector<double>.Build.Dense(numberOfNodes + 1);        
            SoftStringNormalForce = Vector<double>.Build.Dense(numberOfNodes + 1);
            Tension = Vector<double>.Build.Dense(numberOfNodes + 1);
            Torque = Vector<double>.Build.Dense(numberOfNodes + 1);
            MudTorque = 0;
          
            // Initialize sleeve to lumped index mapping
            SleeveToLumpedIndex = new List<int>();
            for (int i = 0; i < numberOfNodes; i++)
            {
                SleeveToLumpedIndex.Add(parameters.Drillstring.SleeveIndexPosition.Contains(i) ? i : -1);
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

            this.BitDepth = parameters.Input.InitialBitDepth;                               // Initial values set in SimulationParameters - to be moved
            PreviousCalculatedBitDepth = parameters.Input.InitialBitDepth;
            this.HoleDepth = parameters.Input.InitialHoleDepth;
            //this. ZVelocity[0] = parameters.Input.InitialTopOfStringVelocity;
            BitOnBotton = false;

            MakeConnection = false;
            PullOutBeforeConnectionBool = false;
            ConnectionStartTime = 0;                                 // [s] connection start time
            TopDriveStartupTime = 0;
            OnBottomStart = -1;
          }
        /// <summary>
        /// Adds one axial-torsional node and <paramref name="lateralNodesAdded"/> lateral nodes at the top of the string,
        /// repeating the values of the current top node. The new element shifts the reference position of every node
        /// down by <paramref name="addedLength"/>, so the axial displacements are reduced by the same amount to keep
        /// the physical positions.
        /// </summary>
        public void AddNewLumpedElement(int lateralNodesAdded, double addedLength)
        {
            numberOfElements += lateralNodesAdded;
            numberOfNodes += lateralNodesAdded;
            numberOfAxialNodes += 1;
            // Axial-torsional nodes
            AngularDisplacement = ExtendVectorStart(AngularDisplacement[0], AngularDisplacement);
            AngularVelocity = ExtendVectorStart(AngularVelocity[0], AngularVelocity);
            AngularAcceleration = ExtendVectorStart(AngularAcceleration[0], AngularAcceleration);
            ZDisplacement = ExtendVectorStart(ZDisplacement[0], ZDisplacement);
            ZVelocity = ExtendVectorStart(ZVelocity[0], ZVelocity);
            ZAcceleration = ExtendVectorStart(ZAcceleration[0], ZAcceleration);
            for (int i = 0; i < ZDisplacement.Count; i++)
            {
                ZDisplacement[i] -= addedLength;
            }
            TopDrive.RelativeAxialPosition -= addedLength;
            // Lateral nodes
            AxialDisplacementAtLateralNodes = ExtendVectorStart(AxialDisplacementAtLateralNodes[0], AxialDisplacementAtLateralNodes, lateralNodesAdded);
            AxialVelocityAtLateralNodes = ExtendVectorStart(AxialVelocityAtLateralNodes[0], AxialVelocityAtLateralNodes, lateralNodesAdded);
            AxialAccelerationAtLateralNodes = ExtendVectorStart(AxialAccelerationAtLateralNodes[0], AxialAccelerationAtLateralNodes, lateralNodesAdded);
            AngularDisplacementAtLateralNodes = ExtendVectorStart(AngularDisplacementAtLateralNodes[0], AngularDisplacementAtLateralNodes, lateralNodesAdded);
            AngularVelocityAtLateralNodes = ExtendVectorStart(AngularVelocityAtLateralNodes[0], AngularVelocityAtLateralNodes, lateralNodesAdded);
            AngularAccelerationAtLateralNodes = ExtendVectorStart(AngularAccelerationAtLateralNodes[0], AngularAccelerationAtLateralNodes, lateralNodesAdded);
            XDisplacement = ExtendVectorStart(XDisplacement[0], XDisplacement, lateralNodesAdded);
            XVelocity = ExtendVectorStart(XVelocity[0], XVelocity, lateralNodesAdded);
            XAcceleration = ExtendVectorStart(XAcceleration[0], XAcceleration, lateralNodesAdded);
            YDisplacement = ExtendVectorStart(YDisplacement[0], YDisplacement, lateralNodesAdded);
            YVelocity = ExtendVectorStart(YVelocity[0], YVelocity, lateralNodesAdded);
            YAcceleration = ExtendVectorStart(YAcceleration[0], YAcceleration, lateralNodesAdded);
            WhirlAngle = ExtendVectorStart(WhirlAngle[0], WhirlAngle, lateralNodesAdded);
            WhirlVelocity = ExtendVectorStart(WhirlVelocity[0], WhirlVelocity, lateralNodesAdded);
            RadialDisplacement = ExtendVectorStart(RadialDisplacement[0], RadialDisplacement, lateralNodesAdded);
            RadialVelocity = ExtendVectorStart(RadialVelocity[0], RadialVelocity, lateralNodesAdded);
            SlipCondition = ExtendVectorStart(SlipCondition[0], SlipCondition, lateralNodesAdded);
            SleeveForces = ExtendVectorStart(SleeveForces[0], SleeveForces, lateralNodesAdded);
            BendingMomentX = ExtendVectorStart(BendingMomentX[0], BendingMomentX, lateralNodesAdded);
            BendingMomentY = ExtendVectorStart(BendingMomentY[0], BendingMomentY, lateralNodesAdded);
            NormalCollisionForce = ExtendVectorStart(NormalCollisionForce[0], NormalCollisionForce, lateralNodesAdded);
            SoftStringNormalForce = ExtendVectorStart(SoftStringNormalForce[0], SoftStringNormalForce, lateralNodesAdded);
            Tension = ExtendVectorStart(Tension[0], Tension, lateralNodesAdded);
            Torque = ExtendVectorStart(Torque[0], Torque, lateralNodesAdded);
            // Insert -1 at the beginning of SleeveToLumpedIndex to account for the new elements
            for (int i = 0; i < lateralNodesAdded; i++)
            {
                SleeveToLumpedIndex.Insert(0, -1);
            }
            // Shift every positive number in SleeveToLumpedIndex to update indices after insertion
            for (int i = 0; i < SleeveToLumpedIndex.Count; i++)
            {
                if (SleeveToLumpedIndex[i] > 0)
                {
                    SleeveToLumpedIndex[i] += lateralNodesAdded;
                }
            }
        }
    }

}
