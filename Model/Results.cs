using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Results
    {
        public double AvgCumulativeSSI { get; set; } = 0.0;

        // Time based values
        public Scalars Scalars { get; set; } 

        // depth based values
        public List<Profiles> Profiles { get; set; } = new List<Profiles>() { };
    }
}
