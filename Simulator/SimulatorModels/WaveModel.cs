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
            //UpdateBoundaryConditions(state, simulationParameters);           
        }

        public void PrepareModel(in State state, in SimulationParameters parameters)
        {
            for (int i = 0; i < NumberOfElements-1; i++)          
            {
                DownwardWave[i] = Strain[i] + WaveSpeed * Strain[i];
                UpwardWave[i]   = Strain[i] - WaveSpeed * Strain[i];
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
                nodeIndex = i / NumberOfLateralElements;
                if (i == nodeIndex *  NumberOfLateralElements)
                {
                    // Calculate unkown wave properties by setting vel = (upwward_wave + downward_wave) / 2
                    double velocity = (i == 0) ? initialVelocity : velocityVector[nodeIndex - 1];
                    downwardBoundary = - UpwardWave[i] + 2 * velocity;
                    // Correct element adjacent element by assuming constant derivative in the neighborhood
                    if (i > 1)
                    {
                        DownwardWave[i + 1] = DownwardWave[i] + (DownwardWave[i - 1] - DownwardWave[i - 2]);
                    }
                }
                else
                {
                    downwardBoundary = DownwardWave[i - 1];
                }
                //Update differential vector
                DiffDownwardWave[i] = DownwardWave[i] - downwardBoundary;
            }
            #endregion
            #region Upward wave
            double upwardBoundary;            
            // Reverse loop through upward wave to avoid overwritting
            for (int i = NumberOfElements-2; i >= 0; i--)
            {
                nodeIndex = i / NumberOfLateralElements;
                if (i == nodeIndex *  NumberOfLateralElements - 1)
                {
                    // Calculate unkown wave properties by setting vel = (upwward_wave + downward_wave) / 2
                    upwardBoundary = - DownwardWave[i] + 2 * velocityVector[nodeIndex];
                    // Correct element adjacent element by assuming constant derivative in the neighborhood
                    if (i < NumberOfElements - 2)
                    {
                        UpwardWave[i - 1] = UpwardWave[i] - (UpwardWave[i + 2] - UpwardWave[i + 1]); 
                    }
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
        public virtual void UpdateState(State state)
        {
            // Compute states from Riemann invariants                              
            for (int i = 0; i < NumberOfElements; i ++)
            {                       
                // Compute states from Riemann invariants              
                Strain[i] = (DownwardWave[i] - UpwardWave[i]) / (2 * WaveSpeed);
                Velocity[i] = 0.5 * (DownwardWave[i] + UpwardWave[i]);                                               
            }      
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
                //Interpolate velocity
                deltaVelocity = (model.Velocity[upperWaveIndex] - model.Velocity[lowerWaveIndex]) / model.ElementLength;// (upperPosition - lowerPosition);
                zeroVelocity = model.Velocity[lowerWaveIndex] - deltaVelocity * lowerPosition;
                InterpolatedVelocity[i] = deltaVelocity * currentPosition + zeroVelocity;                
            }
        }                
    }
}