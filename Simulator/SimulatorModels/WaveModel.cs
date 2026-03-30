using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class WaveModel : IModel<WaveModel>
    {             
        // Wave propagation variables
        public Vector<double> DownwardWave;
        public Vector<double> UpwardWave;
        public Vector<double> DownwardWaveBoundary;
        public Vector<double> UpwardWaveBoundary;

        public Vector<double> DiffDownwardWave;
        public Vector<double> DiffUpwardWave;
        public double TopBoundary;
        public double BottomBoundary;                
        public double ElementLength;
        public int NumberOfElements;
        public int LateralModelToWaveRatio;
        public int NumberOfLateralElements;

        public double WaveSpeed;
        public Vector<double> Strain;
        public Vector<double> Velocity;
         
        //Interpolation variables
        private double currentPosition;
        private int lowerWaveIndex;
        private int upperWaveIndex;            
        private double lowerPosition;
        private double deltaStrain;
        private double zeroStrain; 
        private double deltaVelocity;
        private double zeroVelocity;
        public double TopBoundaryVelocity;
        public double BottomBoundaryVelocity;
        public double TopBoundaryStrain;
        public double BottomBoundaryStrain;
        public Vector<double> InterpolatedStrain;
        public Vector<double> InterpolatedVelocity;
        private List<int> DownwardBoundaryIndex = new();
        private List<int> UpwardBoundaryIndex = new();

        public Vector<double> StrainDifference;

        public WaveModel(in SimulationParameters simulationParameters)
        {
            //Needs to be update after testing stage
            ElementLength = simulationParameters.DistributedCells.ElementLength;   
            LateralModelToWaveRatio = simulationParameters.DistributedCells.LateralModelToWaveRatio;
            NumberOfLateralElements = simulationParameters.LumpedCells.NumberOfLumpedElements;
            // Calculate the number of elements based on the total length and element length, ensuring it's an integer
            NumberOfElements = simulationParameters.DistributedCells.NumberOfElements;
            DownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            UpwardWave = Vector<double>.Build.Dense(NumberOfElements);
            DownwardWaveBoundary = Vector<double>.Build.Dense(NumberOfElements);
            UpwardWaveBoundary = Vector<double>.Build.Dense(NumberOfElements);
            // Made matching the lateral-torsional model
            Strain = Vector<double>.Build.Dense(NumberOfElements);
            Velocity = Vector<double>.Build.Dense(NumberOfElements);
            DiffDownwardWave = Vector<double>.Build.Dense(NumberOfElements);
            DiffUpwardWave = Vector<double>.Build.Dense(NumberOfElements);  
            InterpolatedStrain = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            InterpolatedVelocity = Vector<double>.Build.Dense(simulationParameters.LumpedCells.NumberOfLumpedElements);
            WaveSpeed = simulationParameters.Drillstring.AxialWaveSpeed;
            StrainDifference = Vector<double>.Build.Dense(NumberOfElements);
            for (int i = 0; i < NumberOfLateralElements; i++)
            {
                DownwardBoundaryIndex.Add(i * LateralModelToWaveRatio);
                UpwardBoundaryIndex.Add((i+1) * LateralModelToWaveRatio - 1);
                
            }
            //UpdateBoundaryConditions(state, simulationParameters);           
        }

        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
            for (int i = 0; i < NumberOfElements-1; i++)          
            {
                DownwardWave[i] = Velocity[i] + WaveSpeed * Strain[i];
                UpwardWave[i]   = Velocity[i] - WaveSpeed * Strain[i];
            }                     
        }
        public void UpdateDifferential(in Vector<double> velocityVector, in double initialVelocity)
        {
            // Allocate relevant variables
            int nodeIndex;
            #region Downward wave
            double downwardBoundary;            
            // Loop through downward wave
            for (int i = 0; i < NumberOfElements; i ++)
            {
                // get the lateral model element number
                nodeIndex = i / LateralModelToWaveRatio;
                //Special treatment for the top-drive
                if (i == 0)
                {
                    downwardBoundary = - UpwardWave[i] + 2 * initialVelocity;                                   
                }
                //Assume a boundary condition for other nodes
                else if (DownwardBoundaryIndex.Contains(i))
                {
                    // Calculate unkown wave properties by setting vel = (upwward_wave + downward_wave) / 2
                    downwardBoundary = - UpwardWave[i] + 2 * velocityVector[nodeIndex - 1];
                    // Correct element adjacent element by assuming constant derivative in the neighborhood                                 
                    //DownwardWave[i + 1] = DownwardWave[i] + (DownwardWave[i - 1] - DownwardWave[i - 2]);                                                                       
                }
                else
                {
                    //Else use the previous element for the upwind stage.
                    downwardBoundary = DownwardWave[i - 1];
                }
                //Update differential vector
                DiffDownwardWave[i] = DownwardWave[i] - downwardBoundary;
            }
            #endregion
            #region Upward wave
            double upwardBoundary;            
            // Reverse loop through upward wave to avoid overwritting
            for (int i = NumberOfElements-1; i >= 0; i--)
            {
                nodeIndex = i  / LateralModelToWaveRatio;
                if (i == NumberOfElements-1)
                {
                    // Use the last element
                    upwardBoundary = - DownwardWave[i] + 2 * velocityVector[velocityVector.Count - 1];                    
                }
                else if (UpwardBoundaryIndex.Contains(i))
                {                
                    //Note that the last element is used twice.
                    // Calculate unkown wave properties by setting vel = (upwward_wave + downward_wave) / 2
                    upwardBoundary = - DownwardWave[i] + 2 * velocityVector[nodeIndex];
                    // Correct element adjacent element by assuming constant derivative in the neighborhood                         
                    //UpwardWave[i - 1] = UpwardWave[i] - (UpwardWave[i + 2] - UpwardWave[i + 1]);                                                                  
                }
                else
                {
                    upwardBoundary = UpwardWave[i + 1];
                }
                DiffUpwardWave[i] = upwardBoundary - UpwardWave[i];                 
            }
            #endregion                          
        }
        public virtual void CalculateAccelerations(State state, in SimulationParameters parameters)
        {
            
        }
        public virtual void UpdateState(State state, in SimulationParameters parameters)
        {
            // Compute states from Riemann invariants                              
            for (int i = 0; i < NumberOfElements; i ++)
            {                       
                // Compute states from Riemann invariants              
                Strain[i] = (DownwardWave[i] - UpwardWave[i]) / (2 * WaveSpeed);
                Velocity[i] = 0.5 * (DownwardWave[i] + UpwardWave[i]);   
                StrainDifference[i] = i > 0 ? Strain[i] - Strain[i-1] : 0;                                            
            }      
            InterpolateStateFromWave(this, parameters);      
        }
        public void InterpolateStateFromWave( 
            in WaveModel model, 
            in SimulationParameters simulationParameters)
        {                       
            for (int i = 0; i < NumberOfLateralElements; i++)
            {
                currentPosition = i * simulationParameters.Drillstring.TotalLength / NumberOfLateralElements;
                //Get the position index
                lowerWaveIndex = (int) Math.Floor(currentPosition / model.ElementLength);
                upperWaveIndex = lowerWaveIndex + 1;
                lowerPosition = lowerWaveIndex * model.ElementLength;
                //Interpolate strain
                deltaStrain = (model.Strain[upperWaveIndex] - model.Strain[lowerWaveIndex]) / model.ElementLength;//(upperPosition - lowerPosition);
                zeroStrain = model.Strain[lowerWaveIndex] - deltaStrain * lowerPosition;
                InterpolatedStrain[i] = deltaStrain * currentPosition + zeroStrain;                
            }
        }                
    }
}