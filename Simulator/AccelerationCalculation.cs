using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using System.Security.Cryptography.X509Certificates;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public static class AccelerationCalculation
    {
        public static void PrepareAxialTorsional(AxialTorsionalModel model, State state, SimulationParameters parameters)
        {
            //Update waves            
            model.DownwardAxialWave = state.PipeAngularVelocity + parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain;
            model.UpwardTorsionalWave = state.PipeAngularVelocity - parameters.Drillstring.TorsionalWaveSpeed * state.PipeShearStrain; // Upward traveling wave, torsional
            model.DownwardAxialWave = state.PipeAxialVelocity + parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Downward traveling wave, axial
            model.UpwardAxialWave = state.PipeAxialVelocity - parameters.Drillstring.AxialWaveSpeed * state.PipeAxialStrain; // Upward traveling wave, axial
        }
        public static void AxialTorsionalSystem(AxialTorsionalModel model, Input simulationInput, Configuration configuration, State state, SimulationParameters parameters)
        {
            
            //Create velocity vectors            
            model.OL_vec = ExtendVectorStart(state.TopDriveAngularVelocity, state.LumpedElementAngularVelocity);
            model.VL_vec = ExtendVectorStart(simulationInput.CalculateSurfaceAxialVelocity, state.LumpedElementAxialVelocity);
            // Left boundaries
            model.DownwardTorsionalWaveLeftBoundary = -model.UpwardTorsionalWave.Row(0) + 2 * model.OL_vec.SubVector(0, model.OL_vec.Count - 1);
            model.UpwardTorsionalWaveLeftBoundary = -model.UpwardAxialWave.Row(0) + 2 * model.VL_vec.SubVector(0, model.VL_vec.Count - 1);
            // Right boundaries
            model.DownwardAxialWaveRightBoundary = -model.DownwardTorsionalWave.Row(parameters.LumpedCells.PL - 1) + 2 * model.OL_vec.SubVector(1, model.OL_vec.Count - 1);
            model.UpwardAxialWaveRightBoundary = -model.DownwardAxialWave.Row(parameters.LumpedCells.PL - 1) + 2 * model.VL_vec.SubVector(1, model.VL_vec.Count - 1);

            state.BitVelocity = 0.5 * (model.DownwardAxialWave[parameters.LumpedCells.PL - 1, model.DownwardAxialWave.ColumnCount - 1] + model.UpwardAxialWave[parameters.LumpedCells.PL - 1, model.UpwardAxialWave.ColumnCount - 1]);
            
            
            double angularVelocityBottom;
            if (!configuration.UseMudMotor)
                angularVelocityBottom = 0.5 * (model.DownwardTorsionalWave[parameters.LumpedCells.PL - 1, model.DownwardTorsionalWave.ColumnCount - 1] + model.UpwardTorsionalWave[parameters.LumpedCells.PL - 1, model.UpwardTorsionalWave.ColumnCount - 1]);
            else
                angularVelocityBottom = state.MudRotorAngularVelocity;

            double[] bitForces = parameters.BitRock.
                CalculateInteractionForce(state, angularVelocityBottom, model.DownwardTorsionalWave, parameters);
            model.TorqueOnBit = bitForces[0];
            model.WeightOnBit = bitForces[1];    
            // manage the bit sticking off bottom condition
            if (!state.onBottom)
            {
                double omega_ = state.LumpedElementAngularVelocity[state.LumpedElementAngularVelocity.Count - 1];
                if (simulationInput.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;
                    Vector<double> TorsionalAcceleration = model.UpwardTorsionalWave.Row(parameters.LumpedCells.PL - 1);
                    Vector<double> AxialAcceleration = model.UpwardAxialWave.Row(parameters.LumpedCells.PL - 1);
                    model.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] / parameters.Drillstring.TorsionalWaveSpeed * TorsionalAcceleration[TorsionalAcceleration.Count - 1];
                    model.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] / parameters.Drillstring.AxialWaveSpeed * AxialAcceleration[TorsionalAcceleration.Count - 1];
                }
                else
                {
                    double normalForce_ = simulationInput.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.LumpedElementAxialVelocity[state.LumpedElementAxialVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            model.TorqueOnBit = 0;
                            model.WeightOnBit = 0;

                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            model.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            model.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        model.TorqueOnBit = 0;
                        model.WeightOnBit = 0;
                    }
                }
            }   
            model.DownwardTorsionalWaveStackedWithLeftBoundary = model.DownwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(model.DownwardTorsionalWave);
            model.UpwardTorsionalWaveStackedWithLeftBoundary = model.UpwardTorsionalWave.Stack(model.DownwardAxialWaveRightBoundary.ToRowMatrix());               
            model.DownwardAxialWaveStackedWithRightBoundary = model.UpwardTorsionalWaveLeftBoundary.ToRowMatrix().Stack(model.DownwardAxialWave);
            model.UpwardAxialWaveStackedWithRightBoundary = model.UpwardAxialWave.Stack(model.UpwardAxialWaveRightBoundary.ToRowMatrix());          
        }        

    }
}