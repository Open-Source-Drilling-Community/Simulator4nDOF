using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using NORCE.Drilling.Simulator4nDOF.Model;
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
    public class NumericalIntegrationScheme
    {
        public Vector<double> Xi;
        public Vector<double> Yi;
        public Vector<double> DxDti;
        public Vector<double> DyDti;
        
        public Vector<double> XiPlusOne;
        public Vector<double> YiPlusOne;
        
        
        public NumericalIntegrationScheme(SimulationParameters simulationParameters)
        {
            
        }
        public virtual void IntegrateTimeStep()
        {
            
        }
    }
}