using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Results
    {

        public double Time { get; set; }
        public double WOB { get; set; }
        public double TOB { get; set; }
        public double TopDriveTorque { get; set; }
        public double BitRPM { get; set; }
        public double TopDriveRPM { get; set; }
        public double BitDepth { get; set; }
        public double HoleDepth { get; set; }
        public double TopOfStringPosition { get; set; }
        public double BitVelocity { get; set; }
        public double SSI { get; set; }



        //public double Tag { get; set; } // Or value for each eg WOB

        //public double Value { get; set; } // Skip depth based for now

        //public List<DepthProfilePointModel> Values { get; set; }
    }
}
