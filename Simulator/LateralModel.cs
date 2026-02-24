using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.Common;
using System;
using System.Reflection;
using System.Reflection.Metadata;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator
{
    public class LateralModel
    {
       
        public Vector<double> NormalCollisionForce;        
        public Vector<double> SoftStringNormalForce;
        public Vector<double> Tension;
        public Vector<double> Torque;
        public Vector<double> BendingStiffness;
        public Vector<double> PolarMomentTimesShearModuli;
        public Vector<double> PreStressNormalForce;
        public Vector<double> PreStressBinormalForce;
                        
        public Matrix<double> ScalingMatrix;
        public Vector<double> ToolFaceAngle;

        public double MudTorque;        
    
        public Vector<double> BendingMomentX;
        public Vector<double> BendingMomentY;

        public double PhiDdotNoSlipSensor;
        public double ThetaDotNoSlipSensor;
        
        public LateralModel(SimulationParameters simulationParameters, State state)
        {

            NormalCollisionForce = Vector<double>.Build.Dense(state.XDisplacement.Count);;        
            SoftStringNormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            Tension = Vector<double>.Build.Dense(state.XDisplacement.Count + 1);
            Torque = Vector<double>.Build.Dense(state.XDisplacement.Count + 1);
            BendingStiffness = Vector<double>.Build.Dense(state.XDisplacement.Count+1);
            PolarMomentTimesShearModuli = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressNormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            PreStressBinormalForce = Vector<double>.Build.Dense(state.XDisplacement.Count);
            ScalingMatrix = Vector<double>.Build.Dense(simulationParameters.LumpedCells.DistributedToLumpedRatio, 1).ToColumnMatrix();
            ToolFaceAngle = Vector<double>.Build.Dense(state.XDisplacement.Count);
            MudTorque = 0;        
            BendingMomentX = Vector<double>.Build.Dense(state.XDisplacement.Count);
            BendingMomentY = Vector<double>.Build.Dense(state.YDisplacement.Count);
            PhiDdotNoSlipSensor = 0;
            ThetaDotNoSlipSensor = 0;
        }
        public void UpdateBendingMoments(State state, SimulationParameters simulationParameters)
        {
            double XiMinus1, YiMinus1, XiPlus1, YiPlus1;
            double invElementLengthSquared = 1.0 / (simulationParameters.LumpedCells.ElementLength * simulationParameters.LumpedCells.ElementLength);
            double momentX, momentY;
            
            for (int i = 1; i < state.XDisplacement.Count - 1; i++)
            {
                XiMinus1 = (i == 0) ? 0.0 : state.XDisplacement[i - 1];
                YiMinus1 = (i == 0) ? 0.0 : state.YDisplacement[i - 1];
                XiPlus1 = (i == state.XDisplacement.Count - 1) ? 0.0 : state.XDisplacement[i + 1];
                YiPlus1 = (i == state.YDisplacement.Count - 1) ? 0.0 : state.YDisplacement[i + 1];
                //Calcualte the bending moments using the central difference scheme
                momentX = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (XiPlus1 - 2 * state.XDisplacement[i] + XiMinus1) * invElementLengthSquared; // Bending moment x-component
                momentY = simulationParameters.Drillstring.YoungModuli[i] * simulationParameters.Drillstring.PipeInertia[i] *
                    (YiPlus1 - 2 * state.YDisplacement[i] + YiMinus1) * invElementLengthSquared; //
                 BendingMomentX[i] =  momentX;
                 BendingMomentY[i] =  momentY;
                 
            
            }
        }
    }
}