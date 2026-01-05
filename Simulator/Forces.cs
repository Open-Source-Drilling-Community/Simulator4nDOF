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
    public class Forces
    {
        private State state;
        private SimulationParameters p;
        //Forces
        public Vector<double> TangentialCoulombFriction;
        public Vector<double> AxialCoulombFriction;
        public Vector<double> NormalCollisionForce;
        public Vector<double> StaticTangentialCoulombFriction;
        public Vector<double> StaticAxialCoulombFriction;
        public Vector<double> BendingMomentsX;
        public Vector<double> BendingMomentsY;

        public double TorqueOnBit;
        public double WeightOnBit;
        

        Forces()
        {
            this.TangentialCoulombFriction = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.AxialCoulombFriction = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.NormalCollisionForce = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.StaticTangentialCoulombFriction = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.StaticAxialCoulombFriction = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.BendingMomentsX = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.BendingMomentsY = Vector<double>.Build.Dense(p.LumpedCells.NumberOfLumpedElements);
            this.TorqueOnBit = 0.0;
            this.WeightOnBit = 0.0;
        }
        
    }
}