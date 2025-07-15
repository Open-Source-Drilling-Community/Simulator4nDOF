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
        public string TrajectoryFile { get; set; }
        public string DrillstringFile { get; set; }
        public string DrillstringPressureFile { get; set; }
        public double FluidDensity { get; set; }
        public double CasingShoeDepth { get; set; }
        public double LinerShoeDepth { get; set; }
        public double CasingID { get; set; }
        public double LinerID { get; set; }
        public double WellheadDepth { get; set; }
        public double RiserID { get; set; }
        public double BitRadius { get; set; }

        public Guid DrillStringID { get; set; }
        public Guid DrillStringOpenLabID { get; set; }
        public string AnnulusPressureFile { get; set; }

        public DrillStringSourceType DrillStringSource { get; set; } = DrillStringSourceType.DrillStringMS;


        // ID 066505f3-bbeb-47e1-a0c9-b48b16a1f3aa
        //HttpHostName  https://dev.digiwells.no/
        //HttpHostBasePath DrillString/api/
        //HttpEndPoint DrillString/
    }

    
}
