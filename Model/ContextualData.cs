using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class ContextualData
    {
        public ModelShared.Trajectory? Trajectory { get; set; }
        public double FluidDensity { get; set; }
        public double BitRadius { get; set; }
        public ModelShared.DrillString DrillString { get; set; }
        public ModelShared.DrillingFluidDescription DrillingFluidDescription {get; set;}
        public ModelShared.CasingSection CasingSection {get; set;}
        public double Temperature {get; set;} = 293;
        public double SurfacePipePressure {get; set;} = 100_000;
        public SolverType SolverType { get; set; } = SolverType.VerletMethod;
        

        // ID 066505f3-bbeb-47e1-a0c9-b48b16a1f3aa
        //HttpHostName  https://dev.digiwells.no/
        //HttpHostBasePath DrillString/api/
        //HttpEndPoint DrillString/
    }

}
