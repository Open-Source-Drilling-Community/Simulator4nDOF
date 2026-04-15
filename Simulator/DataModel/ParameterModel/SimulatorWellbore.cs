using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulatorWellbore
    {
        // Move topdrive
        public readonly double WallStiffness = 5.0e7;                 // [N.s/m] coefficient of damping at wellbore wall
        public readonly double WallDamping = 5.0e4;                   // percent of mass imbalance compared to total mass

        // Geometry to be configured
        private List<SimulatorBoreHole> BoreHoleSizes {get; set;}
        public Vector<double> DrillStringClearance;            // [m] drillstring radial clearance to the borehole wall

        private Vector<double> boreholeRadius;                   // [m] Wellbore radius calculation

        public SimulatorWellbore( in SimulatorDrillString drillString, in CasingSection casingSection )
        {
            
            List<SimulatorBoreHole> boreHoleSizes = new();
            double depth = casingSection.TopDepth.GaussianValue.Mean ?? 0;
            foreach (BoreHoleSize boreHoleSize in casingSection.CasingSectionSizeTable)
            {
                boreHoleSizes.Add(
                    new SimulatorBoreHole
                    {
                        Depth = (double ) depth,
                        Diameter = (double) boreHoleSize.HoleSize.GaussianValue.Mean!,
                        Length  = (double) boreHoleSize.Length.GaussianValue.Mean!  
                    }
                );
                depth += boreHoleSize.Length.GaussianValue.Mean ?? 0.0;
            }
            if (casingSection.OpenHoleSection != null)
            {
                foreach (BoreHoleSize boreHoleSize in casingSection.OpenHoleSection.HoleSizes)
                {
                    boreHoleSizes.Add(
                        new SimulatorBoreHole
                        {
                            Depth = (double ) depth,
                            Diameter = (double) boreHoleSize.HoleSize.GaussianValue.Mean!,
                            Length  = (double) boreHoleSize.Length.GaussianValue.Mean!  
                        }
                    );                    
                } 
            }
            BoreHoleSizes = boreHoleSizes;
            DrillStringClearance = Vector<double>.Build.Dense(drillString.RelativeNodeDepth.Count);

            UpdateWellbore(drillString);
        }
        
        public void UpdateWellbore(in SimulatorDrillString drillString)
        {
            // Wellbore radius calculation at each node
            boreholeRadius = Vector<double>.Build.Dense(drillString.RelativeNodeDepth.Count);
            int index = 0;
            double localRadius;
            for (int i = 0; i < drillString.RelativeNodeDepth.Count; i++)
            {

                // If the node depth is greater than the borehole, go to the next one
                if (index < BoreHoleSizes.Count)
                    index += (drillString.RelativeNodeDepth[i] > BoreHoleSizes[index].Depth) ? 1 : 0; 
                // Switch between borehole radius and bit radius
                localRadius = index < BoreHoleSizes.Count ? 0.5 * BoreHoleSizes[index].Diameter : drillString.BitRadius;
                //Update radius list
                boreholeRadius[i] = localRadius;            
            }
            for (int i = 0; i < drillString.RelativeNodeDepth.Count; i++)
            {
                double outerRadius = drillString.SleeveIndexPosition.Contains(i) ? drillString.SleeveOuterRadius : drillString.NodeOuterRadius[i];
                DrillStringClearance[i] = boreholeRadius[i] - outerRadius;                
            }        
        }
    }
}
 