using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Diagnostics;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class AxialTorsionalModel : IModel<AxialTorsionalModel>
    {     
        private double topDriveTorque;
        private double torsionalVelocityLeft;
        private double axialVelocityLeft;
        private double[] bitForces;
        public AxialTorsionalModel(State state, in SimulationParameters simulationParameters)
        {
            bitForces = new double[2]{0,0};
            UpdateBoundaryConditions(state, simulationParameters);           
        }
        public void UpdateBoundaryConditions(State state, in SimulationParameters parameters)
        {                                               
            int N = state.AxialVelocity.Count;
            int idx = parameters.LumpedCells.DistributedToLumpedRatio - 1;
            for (int i = 0; i < N; i++)
            {
                torsionalVelocityLeft = (i == 0) ? state.TopDrive.TopDriveAngularVelocity : state.AngularVelocity[i - 1];
                axialVelocityLeft = (i == 0) ? state.TopDrive.CalculateSurfaceAxialVelocity : state.AxialVelocity[i - 1];                
                //Left boundaries
                state.DownwardTorsionalWaveLeftBoundary[i] = - state.UpwardTorsionalWave[0, i] + 2 * torsionalVelocityLeft;
                state.DownwardAxialWaveLeftBoundary[i]    = - state.UpwardAxialWave[0, i] + 2 * axialVelocityLeft;
                //Right boundaries
                state.UpwardTorsionalWaveRightBoundary[i] = - state.DownwardTorsionalWave[idx, i] + 2 * state.AngularVelocity[i];
                state.UpwardAxialWaveRightBoundary[i]    = - state.DownwardAxialWave[idx, i] + 2 * state.AxialVelocity[i];
            }         
        }
        public void IntegrateTopDriveSpeed(State state, SimulationParameters parameters)
        {
            topDriveTorque = 0.5 * parameters.Drillstring.PipePolarMoment[0] * parameters.Drillstring.ShearModuli[0] / parameters.Drillstring.TorsionalWaveSpeed *
                (   
                    state.DownwardTorsionalWave[0, 0] 
                    - state.UpwardTorsionalWave[0, 0]
                );

            state.TopDrive.TopDriveAngularVelocity = state.TopDrive.TopDriveAngularVelocity + parameters.InnerLoopTimeStep * (state.TopDrive.TopDriveMotorTorque - topDriveTorque) / parameters.Wellbore.TopDriveInertia;                
        }

        public void PrepareModel(AxialTorsionalModel model, State state, SimulationParameters parameters)
        {
            
            for (int i = 0; i < parameters.LumpedCells.DistributedToLumpedRatio; i++)          
            {
                for (int j = 0; j < parameters.LumpedCells.NumberOfLumpedElements; j++)          
                {
                    // --- Update Torsional waves                   
                    //Downward torsional wave
                    state.DownwardTorsionalWave[i, j] = state.PipeAngularVelocity[i, j] 
                        + parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain[i, j];
                    //Upward torsional wave
                    state.UpwardTorsionalWave[i, j]   = state.PipeAngularVelocity[i, j] 
                        - parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain[i, j]; 
                    // --- Update Axial waves
                    // Downward axial wave
                    state.DownwardAxialWave[i, j] = state.PipeAxialVelocity[i, j] 
                        + parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain[i, j]; 
                    //Upward axial wave
                    state.UpwardAxialWave[i, j] = state.PipeAxialVelocity[i, j] 
                        - parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain[i, j];                        
                }
            }
        }
        public void CalculateAccelerations(State state, in SimulationParameters parameters)
        {                                        
            this.UpdateBoundaryConditions(state, parameters);               
            state.BitVelocity = 0.5 * 
                (
                    state.DownwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, state.DownwardAxialWave.ColumnCount - 1] 
                    + state.UpwardAxialWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, state.UpwardAxialWave.ColumnCount - 1]
                );
           
            double angularVelocityBottom;
            if (!parameters.UseMudMotor)
                angularVelocityBottom = 0.5 * 
                    (
                        state.DownwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, state.DownwardTorsionalWave.ColumnCount - 1] 
                        + state.UpwardTorsionalWave[parameters.LumpedCells.DistributedToLumpedRatio - 1, state.UpwardTorsionalWave.ColumnCount - 1]
                    );
            else
                angularVelocityBottom = state.MudRotorAngularVelocity;

            bitForces = parameters.BitRock.CalculateInteractionForce(state, angularVelocityBottom, parameters);
            state.TorqueOnBit = bitForces[0];
            state.WeightOnBit = bitForces[1];
           
            // manage the bit sticking off bottom condition
            if (!state.BitOnBotton)
            {
                double omega_ = state.AngularVelocity[state.AngularVelocity.Count - 1];
                if (parameters.Input.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;             
                    double torsionalAcceleration = state.UpwardTorsionalWave[state.UpwardTorsionalWave.RowCount - 1, state.UpwardTorsionalWave.ColumnCount - 1];
                    double axialAcceleration = state.UpwardAxialWave[state.UpwardAxialWave.RowCount - 1, state.UpwardAxialWave.ColumnCount - 1];                                            
                    state.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] / parameters.Drillstring.TorsionalWaveSpeed * torsionalAcceleration;
                    state.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] / parameters.Drillstring.AxialWaveSpeed * axialAcceleration;
                }
                else
                {
                    double normalForce_ = parameters.Input.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.AxialVelocity[state.AxialVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            state.TorqueOnBit = 0;
                            state.WeightOnBit = 0;
                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            state.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            state.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        state.TorqueOnBit = 0;
                        state.WeightOnBit = 0;
                    }
                }
            }   
       }                
    }
}