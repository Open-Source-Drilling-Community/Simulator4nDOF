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
        public string AnnulusPressureFile { get; set; }
        public string DrillstringPressureFile { get; set; }
        public double FluidDensity { get; set; }
        public double CasingShoeDepth { get; set; }
        public double LinerShoeDepth { get; set; }
        public double CasingID { get; set; }
        public double LinerID { get; set; }
        public double WellheadDepth { get; set; }
        public double RiserID { get; set; }
        public double BitRadius { get; set; }
    }
}
