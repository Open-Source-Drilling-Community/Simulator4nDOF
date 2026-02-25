using Model;
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
        public List<BoreHoleSize> BoreHoleSizeList { get; set; } = new List<BoreHoleSize>();
        public double BitRadius { get; set; }
        public ModelShared.DrillString DrillString { get; set; }
        public string AnnulusPressureFile { get; set; }
        public string DrillstringPressureFile { get; set; }
        

        // ID 066505f3-bbeb-47e1-a0c9-b48b16a1f3aa
        //HttpHostName  https://dev.digiwells.no/
        //HttpHostBasePath DrillString/api/
        //HttpEndPoint DrillString/
    }

    public class BoreHoleSize()
    {
        public double Depth { get; set; }
        public double ID { get; set; }
    }
}
