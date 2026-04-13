using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using OSDC.DotnetLibraries.General.Math;
using SharpYaml.Serialization.Logging;
using System.Globalization;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class SimulatorDrillString
    {
        //Default values
        public double BitRadius = .2159 / 2;                   // [m] Bit radius(used in both Detournay and MSE model)
        public readonly double SteelDensity = 7850;              // [kg / m3] Density of steel
        public readonly double PoissonRatio = 0.28;               // Poisson ratio of steel
        public double ToolJointLength = 0.5;                       // [m] Default tool joint length
        
        public double SleeveInnerRadius = 0.1651 / 2;                // [m] Sleeve inner radius
        public double SleeveOuterRadius = 0.1905 / 2 + 0.005;        // [m] Sleeve outer radius
        public double SleeveLength = 1.0;                        // [m] Sleeve length
        public double SleeveTorsionalDampingCoefficient = 2.0e2;              // [N.s/rad] Sleeve torsional damping coefficient
        public double AxialFrictionReduction = 0.99;    // Axial friction reduction factor due to spur wheels
        public readonly double AddedFluidMassCoefficient = 1.7;              // Added fluid mass coefficient
        public readonly double MassImbalancePercentage = .05; //percent (fraction?) of mass imbalance compared to total mass -  todo rename?

        public List<double> ElementLength;                       
        public List<double> ElementDensity;                       
        public List<double> ElementOuterRadius;                       // [m] Outer radius vector
        public List<double> ElementInnerRadius;                       // [m] Inner radius vector
        public List<double> ElementEccentricity;                        // [m] Eccentricity vector
        public List<double> ElementOuterArea;                       // [m^2] Drillpipe outer area vector
        public List<double> ElementInnerArea;                       // [m^2] Drillpipe inner area vector
        public List<double> ToolJointOuterArea;                     // [m^2] Tool joint outer area vector
        public List<double> ToolJointInnerArea;                     // [m^2] Tool joint inner area vector

        // Inertial peroperties                         
        public List<double> ElementPolarInertia;                        // [m^4] Drillpipe polar moment of inertia vector
        public List<double> ElementArea;                        // [m^2] Drillpipe cross sectional area vector
        public List<double> ElementInertia;                        // [m^4] Drillpipe moment of inertia vector

        public List<double> WeightCorrectionFactor;               // [-] Linear weight correction factor vector
        public List<double> ElementYoungModuli;                        // [Pa] Young's modulus vector
        public List<double> ElementShearModuli;                        // [Pa] Shear modulus vector
        public List<double> ElementFluidAddedMass;                       // [kg] Added fluid mass
        public List<double> ElementEccentricMass;                      // [kg] Eccentric mass

        public List<double> InactiveElementLength;                       // [m] Outer radius vector
        public List<double> InactiveElementDensity;                       // [m] Outer radius vector
        
        public List<double> InactiveElementOuterRadius;                       // [m] Outer radius vector
        public List<double> InactiveElementInnerRadius;                       // [m] Inner radius vector
        public List<double> InactiveElementEccentricity;                        // [m] Eccentricity vector
        public List<double> InactiveElementOuterArea;                       // [m^2] Drillpipe outer area vector
        public List<double> InactiveElementInnerArea;                       // [m^2] Drillpipe inner area vector
        public List<double> InactiveToolJointOuterArea;                     // [m^2] Tool joint outer area vector
        public List<double> InactiveToolJointInnerArea;                     // [m^2] Tool joint inner area vector

        // Inertial peroperties                         
        public List<double> InactiveElementPolarInertia;                        // [m^4] Drillpipe polar moment of inertia vector
        public List<double> InactiveElementArea;                        // [m^2] Drillpipe cross sectional area vector
        public List<double> InactiveElementInertia;                        // [m^4] Drillpipe moment of inertia vector

        public List<double> InactiveWeightCorrectionFactor;               // [-] Linear weight correction factor vector
        public List<double> InactiveElementYoungModuli;                        // [Pa] Young's modulus vector
        public List<double> InactiveElementShearModuli;                        // [Pa] Shear modulus vector
        public List<double> InactiveElementFluidAddedMass;                       // [kg] Added fluid mass
        public List<double> InactiveElementEccentricMass;                      // [kg] Eccentric mass

        //public Vector<double> LumpedElementMomentOfInertia;                      // [kg.m^2] Lumped element mass moment of inertia
        //public Vector<double> LumpedElementMass;                      // [kg] Lumped element mass
        //public Vector<double> LumpedElementMassMomentOfInertia;                      // [kg.m^2] Lumped element mass moment of inertia
                                                        

        // Sleeves - to be configured
        public Vector<double> SleeveDistancesFromBit;  // Example data


        // Sleeves - calculated
        public int TotalSleeveNumber;                                  // Total number of sleeves
        public Vector<double> SleeveIndexPosition;                       // Index of sleeves in lumped nodes
        public double SleeveMassMomentOfInertia;                              // [kg.m^2] Sleeve mass moment of inertia
        public Vector<double> SleeveTorsionalDamping;                     // [N.s/rad] Sleeve torsional damping coefficient

        public double TorsionalDampingFactor = 1.0;                  //[N.m.s/rad] Torsional damping 
        public double AxialDampingFactor = 1.0;                  //[N.s/m] Axial damping
        public double LateralDampingFactor = 0.0;                  //[N.s/m] Lateral damping

        // Structureal damping - calculated
        public double CalculatedTorsionalDamping;
        public double CalculatedAxialDamping;
        public double CalculateLateralDamping;
        // 
        public double TotalLength;
        public Vector<double> BendingStiffness;                       // [N/m] Bending stiffness
        
        public double CharacteristicDrillPipeImpedance;                           // Characteristic drill pipe impedance

        //IMU sensors -  configure
        public double SensorDistanceFromBit = 63;       // [m] axial distance(relative to bit depth) of an IMU measuring downhole RPM and accelerations
        public readonly double SensorMisalignmentPolarAngle = 0 * Math.PI / 180; // polar angle of the accelerometer misalignment[rad]
        public readonly double SensorMisalignmentAzimuthAngle = 0 * Math.PI / 180; // azimuth angle of the accelerometer misalignment[rad]
        public readonly Vector<double> SensorDisplacementsInLocalFrame = Vector<double>.Build.Dense(3, 0); // radial, tangential and axial displacements in the local frame of the accelerometer

        //IMU sensors -  to be calculated
        public int IndexSensor;
        public double SensorRadialDistance; // [m] radial distance from centerline of the tool to the accelerometer


        private List<double> mergedComponentLength = new List<double> { 0.0 };            
        private List<double> mergedComponentOuterRadius = new List<double> { 0.0 };
        private List<double> mergedComponentArea = new List<double> { 0.0 };
        private List<double> mergedComponentInnerArea = new List<double> { 0.0 };
        private List<double> mergedComponentOuterArea = new List<double> { 0.0 };
        private List<double> mergedComponentInnerRadius = new List<double> { 0.0 };
        private List<double> mergedComponentInertia = new List<double> { 0.0 };
        private List<double> mergedComponentYoungsModulus = new List<double> { 0.0 };
        private List<double> mergedComponentShearModulus = new List<double> { 0.0 };     
        private List<double> mergedComponentDensity = new List<double> { 0.0 };    

        public SimulatorDrillString(LumpedCells lumpedCells, 
                           DrillString drillString,
                           double fluidDensity,
                           double MD, 
                           double Rb, 
                           double sensorDistanceFromBit, 
                           Vector<double> sleeveDistancesFromBit, 
                           double sleepDampingFactor, 
                           double torsionalDampingFactor,
                           double axialDampingFactor,
                           double lateralDampingFactor)
        {
            this.SensorDistanceFromBit = sensorDistanceFromBit;
            this.SleeveDistancesFromBit = sleeveDistancesFromBit;
            this.SleeveTorsionalDampingCoefficient = sleepDampingFactor;
            this.TorsionalDampingFactor = torsionalDampingFactor;
            this.AxialDampingFactor = axialDampingFactor;
            this.LateralDampingFactor = lateralDampingFactor;
            this.BitRadius = Rb;
    #region Drill-String simplification
            // Simplify geometry by merging similar components 
            List<DrillStringComponentTypes> componentTypeList = new List<DrillStringComponentTypes>
                                                {
                                                    drillString.
                                                    DrillStringSectionList.ElementAt(0).
                                                    SectionComponentList.ElementAt(0).
                                                    Type
                                                };
            // Loop through all sections
            foreach (DrillStringSection section in drillString.DrillStringSectionList)
            {
                // Number of times the section is repeated
                int sectionRepetitions = section.Count;
                foreach (DrillStringComponent component in section.SectionComponentList)
                {
                    foreach (DrillStringComponentPart part in component.PartList)
                    {
                        //  Check if the last element is the same type as the previous.
                        // If so, merge them.
                        if (componentTypeList[componentTypeList.Count] == component.Type)
                        {
                            mergedComponentLength[mergedComponentLength.Count - 1] += sectionRepetitions * part.TotalLength;
                            //      Elements in here are to be averaged based on the length. 
                            // They will be divided by the mergedComponent length later on
                            mergedComponentYoungsModulus[mergedComponentYoungsModulus.Count - 1] += sectionRepetitions * part.TotalLength * part.YoungModulus;
                            mergedComponentShearModulus[mergedComponentShearModulus.Count - 1] += sectionRepetitions * part.TotalLength
                                    * part.YoungModulus / ( 2.0* ( 1.0 + part.PoissonRatio ) ); 
                            mergedComponentInertia[mergedComponentInertia.Count - 1] += sectionRepetitions * part.TotalLength * part.FirstCrossSectionTorsionalInertia;
                            mergedComponentOuterRadius[mergedComponentOuterRadius.Count - 1] += 0.5 * sectionRepetitions * part.TotalLength * part.OuterDiameter;
                            mergedComponentArea[mergedComponentArea.Count - 1] += sectionRepetitions * part.TotalLength * part.CrossSectionArea;
                            mergedComponentDensity[mergedComponentDensity.Count - 1] += sectionRepetitions * part.TotalLength * part.MaterialDensity;
                            double innerArea = 0.25 * part.InnerDiameter * part.InnerDiameter * Math.PI;
                            double outerArea = part.CrossSectionArea - innerArea;
                            mergedComponentInnerArea[mergedComponentInnerArea.Count - 1] += sectionRepetitions * part.TotalLength * innerArea;
                            mergedComponentOuterArea[mergedComponentOuterArea.Count - 1] += sectionRepetitions * part.TotalLength * outerArea;
                            mergedComponentInnerRadius[mergedComponentInnerRadius.Count - 1] += 0.5 * sectionRepetitions * part.TotalLength * part.InnerDiameter;
                       
                        }
                        else
                        {
                            //      If the element type is different, 
                            // then add one item and start anew.
                            componentTypeList.Add(component.Type);
                            mergedComponentLength.Add(sectionRepetitions * part.TotalLength);
                            //      Elements in here are to be averaged based on the length. 
                            // They will be divided by the mergedComponent length later on
                            mergedComponentYoungsModulus.Add(sectionRepetitions * part.TotalLength * part.YoungModulus);
                            mergedComponentShearModulus.Add( sectionRepetitions * part.TotalLength
                                    * part.YoungModulus / ( 2.0* ( 1.0 + part.PoissonRatio ) ) ); 
                            mergedComponentInertia.Add(sectionRepetitions * part.TotalLength * part.FirstCrossSectionTorsionalInertia);
                            mergedComponentOuterRadius.Add(0.5 * sectionRepetitions * part.TotalLength * part.OuterDiameter);
                            mergedComponentArea.Add(sectionRepetitions * part.TotalLength * part.CrossSectionArea);       
                            mergedComponentDensity.Add(sectionRepetitions * part.TotalLength * part.MaterialDensity);           
                            double innerArea = 0.25 * part.InnerDiameter * part.InnerDiameter * Math.PI;
                            double outerArea = part.CrossSectionArea - innerArea;
                            mergedComponentInnerArea.Add(sectionRepetitions * part.TotalLength * innerArea);
                            mergedComponentOuterArea.Add(sectionRepetitions * part.TotalLength * outerArea);
                            mergedComponentInnerRadius.Add(0.5 * sectionRepetitions * part.TotalLength * part.InnerDiameter);
                             
                        }
                    }
                }
            }            
            //  Those properties that are to be averaged are called here and divided 
            // by the length calulated for the merged component
            for (int i = 0; i < mergedComponentLength.Count; i++)
            {
                mergedComponentYoungsModulus[i] /= mergedComponentLength[i];
                mergedComponentShearModulus[i] /= mergedComponentLength[i];
                mergedComponentInertia[i] /= mergedComponentLength[i];
                mergedComponentOuterRadius[i] /= mergedComponentLength[i];
                mergedComponentArea[i] /= mergedComponentLength[i];
                mergedComponentDensity[i] /= mergedComponentLength[i];
                mergedComponentInnerArea[i] /= mergedComponentInnerArea[i];
                mergedComponentOuterArea[i] /= mergedComponentOuterArea[i];
                mergedComponentInnerRadius[i] /= mergedComponentInnerRadius[i];
            }
            //  If the drill-string starts with drill-pipes, 
            // then it must be reverted to a Bit/BHA-first order
            if (
                componentTypeList[0] == DrillStringComponentTypes.DrillPipe ||
                componentTypeList[0] == DrillStringComponentTypes.HeavyWeightDrillPipe
            )
            {
                componentTypeList.Reverse();
                mergedComponentLength.Reverse();
                mergedComponentOuterRadius.Reverse();
                mergedComponentArea.Reverse();
                mergedComponentInertia.Reverse();
                mergedComponentYoungsModulus.Reverse();
                mergedComponentShearModulus.Reverse();
                mergedComponentDensity.Reverse();
            }
            #endregion
            #region Lumped Element Discretization
            //  In here the lumped elements are discretized. There are 
            // two expected circumstances:
            //  I - the merged component length > 2 * Expected Element Length
            //       - divide the section in equally spaced elements
            // II - the merged component length < 2 * Expected Element Length
            //       - Use a single element
            double expectedElementLength = lumpedCells.ElementLength;
            //   The drill-string used for the simulation case might 
            // be smaller than the one provided through the microservice.
            // In this case, there will most likely be unused drill-pipe sections.
            // Those  
            
            //  Current position of the last elemebt in relation to the drill-string
            // length (from the bit)
            double lastElementPosition = 0;
            // Loop through all drill-string length                        
            for (int i = 0; i < mergedComponentLength.Count; i++)
            {
                // Using this integer notation, if the 
                //  max(  floor(length/expectedLength) - 1, 0 ) + 1
                //   will return 1 for all length/expectedLength < 2
                //   will round up the number of elements for all length/expectedLength > 2  
                int numberOfElementsInSection = Math.Max((int) Math.Floor(mergedComponentLength[i] / expectedElementLength - 1), 0) + 1;
                for (int j = 0; j < numberOfElementsInSection; j++ )
                {
                    if (lastElementPosition <= lumpedCells.CumulativeElementLength[lumpedCells.CumulativeElementLength.Count - 1])
                    {  
                        // Divide by the number of elements
                        ElementLength.Add( mergedComponentLength[i] / (double) numberOfElementsInSection );
                        //  Those properties do not need to be divided, 
                        // as they have been averaged by the length beforehand:
                        ElementDensity.Add( mergedComponentDensity[i] );                        
                        ElementOuterRadius.Add( mergedComponentOuterRadius[i] );
                        ElementYoungModuli.Add( mergedComponentYoungsModulus[i] );
                        ElementShearModuli.Add( mergedComponentShearModulus[i] );
                        ElementInertia.Add( mergedComponentInertia[i] );
                        ElementArea.Add( mergedComponentArea[i] );
                        
                        ElementInnerRadius.Add( mergedComponentInnerRadius[i] );
                        ElementInnerArea.Add( mergedComponentInnerArea[i] );
                        ElementOuterArea.Add( mergedComponentOuterArea[i] );
                        double mass = mergedComponentDensity[i] * mergedComponentArea[i] * mergedComponentLength[i] / (double) numberOfElementsInSection;
                        ElementEccentricMass.Add( mass );
                        ElementEccentricity.Add( MassImbalancePercentage );
                    }
                    else 
                    {
                        //  If the elements are not part of the initial state, they are nonetheless calculated.
                        // if there drill-string displace downwards enough, a new element will be needed and 
                        // thus the initial configuration shall be preserved.
                        // Divide by the number of elements
                        InactiveElementLength.Add( mergedComponentLength[i] / (double) numberOfElementsInSection );
                        //  Those properties do not need to be divided, 
                        // as they have been averaged by the length beforehand:
                        InactiveElementDensity.Add( mergedComponentDensity[i] );                                              
                        InactiveElementOuterRadius.Add( mergedComponentOuterRadius[i] );
                        InactiveElementYoungModuli.Add( mergedComponentYoungsModulus[i] );
                        InactiveElementShearModuli.Add( mergedComponentShearModulus[i] );
                        InactiveElementInertia.Add( mergedComponentInertia[i] );
                        InactiveElementArea.Add( mergedComponentArea[i] );  
                        InactiveElementInnerRadius.Add( mergedComponentInnerRadius[i] );
                        InactiveElementInnerArea.Add( mergedComponentInnerArea[i] );
                        InactiveElementOuterArea.Add( mergedComponentOuterArea[i] );
                        double mass = mergedComponentDensity[i] * mergedComponentArea[i] * mergedComponentLength[i] / (double) numberOfElementsInSection;
                        InactiveElementEccentricMass.Add( mass );
                        InactiveElementEccentricity.Add( MassImbalancePercentage );                          
                    }
                    lastElementPosition += mergedComponentLength[i] / (double) numberOfElementsInSection;
                }     
            }            
            #endregion
           
        }
    }
}
