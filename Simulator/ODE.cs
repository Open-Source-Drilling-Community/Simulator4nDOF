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
    public class ODE
    {
        public Vector<double> LateralModelDisplacementX;
        public Vector<double> LateralModelDisplacementY;
        public Vector<double> LateralModelVelocityX;
        public Vector<double> LateralModelVelocityY;
        public Vector<double> LateralModelVelocityZ;        
        public Vector<double> LateralAccelerationX;
        public Vector<double> LateralAccelerationY;
        
        public Vector<double> LateralWhirlAngle;
        public Vector<double> LateralWhirlVelocity;        
        public Vector<double> LateralRadialDisplacement;
        public Vector<double> LateralRadialVelocity;
        
        
        public Vector<double> OL_dot;//?
        public Vector<double> OS_dot;//?
        


    }
}