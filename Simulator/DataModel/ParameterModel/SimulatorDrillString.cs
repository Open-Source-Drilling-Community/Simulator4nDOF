using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using SharpYaml.Events;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    /// <summary>
    /// SimulatorDrillString is used to convert a DrillString object from the DrillString Microservice 
    /// onto a simulation ready format. This 
    /// implies that each similar consecutive part of the drill-string will have an 
    /// averaged behaviour with all of the material properties. This is implemented as
    /// a separete class as it can be imported in the future for other models. 
    /// 
    ///     Element description must not contain method-related variables (e.g.: stiffness values).
    /// Those are expected to be calculated at the model's constructor level. 
    /// </summary>
    public class SimulatorDrillString
    {

        //Default values
        /// <summary>
        /// [m] Bit radius (used in both Detournay and MSE model)
        /// </summary>
        public double BitRadius = .2159 / 2;
        /// <summary>
        /// [kg / m3] Density of steel
        /// </summary>
        public readonly double SteelDensity = 7850;
        /// <summary>
        /// Poisson ratio of steel
        /// </summary>
        public readonly double PoissonRatio = 0.28;
        /// <summary>
        /// [m] Default tool joint length
        /// </summary>
        public double ToolJointLength = 0.5;

        /// <summary>
        /// [m] Sleeve inner radius
        /// </summary>
        public double SleeveInnerRadius = 0.1651 / 2;
        /// <summary>
        /// [m] Sleeve outer radius
        /// </summary>
        public double SleeveOuterRadius = 0.1905 / 2 + 0.005;
        /// <summary>
        /// [m] Sleeve length
        /// </summary>
        public double SleeveLength = 1.0;
        /// <summary>
        /// [N.s/rad] Sleeve torsional damping coefficient
        /// </summary>
        public double SleeveTorsionalDampingCoefficient = 2.0e2;
        /// <summary>
        /// Axial friction reduction factor due to spur wheels
        /// </summary>
        public double AxialFrictionReduction = 0.99;
        /// <summary>
        /// Added fluid mass coefficient
        /// </summary>
        public readonly double AddedFluidMassCoefficient = 1.7;
        /// <summary>
        /// Percent (fraction?) of mass imbalance compared to total mass - todo rename?
        /// </summary>
        public readonly double MassImbalancePercentage = .05;

        public List<double> RelativeNodeDepth = new();
        public List<double> ElementDepth = new();
        public List<double> ElementLength = new();
        public List<double> ElementDensity = new();
        /// <summary>
        /// [m] Outer radius vector
        /// </summary>
        public List<double> ElementOuterRadius = new();
        /// <summary>
        /// [m] Inner radius vector
        /// </summary>
        public List<double> ElementInnerRadius = new();
        /// <summary>
        /// [m] Eccentricity vector
        /// </summary>
        public List<double> ElementEccentricity = new();
        /// <summary>
        /// [m^2] Drillpipe outer area vector
        /// </summary>
        public List<double> ElementOuterArea = new();
        /// <summary>
        /// [m^2] Drillpipe inner area vector
        /// </summary>
        public List<double> ElementInnerArea = new();
        /// <summary>
        /// [m^2] Tool joint cross-sectional area vector, computed from the maximum OuterDiameter
        /// and minimum InnerDiameter within each merged component
        /// </summary>
        public List<double> ElementToolJointOuterArea = new();
        /// <summary>
        /// [m^2] Tool joint inner area vector, computed from the minimum InnerDiameter
        /// within each merged component
        /// </summary>
        public List<double> ElementToolJointInnerArea = new();

        // Inertial properties
        /// <summary>
        /// [m^4] Drillpipe polar moment of inertia vector
        /// </summary>
        public List<double> ElementPolarInertia = new();
        /// <summary>
        /// [m^2] Drillpipe cross sectional area vector
        /// </summary>
        public List<double> ElementArea = new();
        /// <summary>
        /// [m^4] Drillpipe moment of inertia vector
        /// </summary>
        public List<double> ElementInertia = new();

        /// <summary>
        /// [-] Linear weight correction factor vector
        /// </summary>
        public List<double> ElementWeightCorrectionFactor = new();
        /// <summary>
        /// [Pa] Young's modulus vector
        /// </summary>
        public List<double> ElementYoungModuli = new();
        /// <summary>
        /// [Pa] Shear modulus vector
        /// </summary>
        public List<double> ElementShearModuli = new();
        /// <summary>
        /// [kg] Added fluid mass
        /// </summary>
        public List<double> ElementFluidAddedMass = new();
        /// <summary>
        /// [kg] Eccentric mass
        /// </summary>
        public List<double> ElementEccentricMass = new();

        /// <summary>
        /// [m] Inactive element length vector
        /// </summary>
        public List<double> InactiveElementLength = new();
        /// <summary>
        /// [kg/m^3] Inactive element density vector
        /// </summary>
        public List<double> InactiveElementDensity = new();

        /// <summary>
        /// [m] Outer radius vector
        /// </summary>
        public List<double> InactiveElementOuterRadius = new();
        /// <summary>
        /// [m] Inner radius vector
        /// </summary>
        public List<double> InactiveElementInnerRadius = new();
        /// <summary>
        /// [m] Eccentricity vector
        /// </summary>
        public List<double> InactiveElementEccentricity = new();
        /// <summary>
        /// [m^2] Drillpipe outer area vector
        /// </summary>
        public List<double> InactiveElementOuterArea = new();
        /// <summary>
        /// [m^2] Drillpipe inner area vector
        /// </summary>
        public List<double> InactiveElementInnerArea = new();

        /// <summary>
        /// [m^2] Tool joint cross-sectional area vector, computed from the maximum OuterDiameter
        /// and minimum InnerDiameter within each merged component
        /// </summary>
        public List<double> InactiveElementToolJointOuterArea = new();
        /// <summary>
        /// [m^2] Tool joint inner area vector, computed from the minimum InnerDiameter
        /// within each merged component
        /// </summary>
        public List<double> InactiveElementToolJointInnerArea = new();

        // Inertial properties
        /// <summary>
        /// [m^4] Drillpipe polar moment of inertia vector
        /// </summary>
        public List<double> InactiveElementPolarInertia = new();
        /// <summary>
        /// [m^2] Drillpipe cross sectional area vector
        /// </summary>
        public List<double> InactiveElementArea = new();
        /// <summary>
        /// [m^4] Drillpipe moment of inertia vector
        /// </summary>
        public List<double> InactiveElementInertia = new();

        /// <summary>
        /// [-] Linear weight correction factor vector
        /// </summary>
        public List<double> InactiveElementWeightCorrectionFactor = new();
        /// <summary>
        /// [Pa] Young's modulus vector
        /// </summary>
        public List<double> InactiveElementYoungModuli = new();
        /// <summary>
        /// [Pa] Shear modulus vector
        /// </summary>
        public List<double> InactiveElementShearModuli = new();
        /// <summary>
        /// [kg] Added fluid mass
        /// </summary>
        public List<double> InactiveElementFluidAddedMass = new();
        /// <summary>
        /// [kg] Eccentric mass
        /// </summary>
        public List<double> InactiveElementEccentricMass = new();

                                                       

        // Sleeves - to be configured
        /// <summary>
        /// [m] Distances from bit to each sleeve
        /// </summary>
        public Vector<double> SleeveDistancesFromBit;

        // Sleeves - calculated
        /// <summary>
        /// Total number of sleeves
        /// </summary>
        public int TotalSleeveNumber;
        /// <summary>
        /// Index of sleeves in lumped nodes
        /// </summary>
        public List<int> SleeveIndexPosition;
        /// <summary>
        /// [kg.m^2] Sleeve mass moment of inertia
        /// </summary>
        public double SleeveMassMomentOfInertia;
        /// <summary>
        /// [N.s/rad] Sleeve torsional damping coefficient
        /// </summary>
        public Vector<double> SleeveTorsionalDamping;

        /// <summary>
        /// [N.m.s/rad] Torsional damping
        /// </summary>
        public double TorsionalDampingFactor = 1.0;
        /// <summary>
        /// [N.s/m] Axial damping
        /// </summary>
        public double AxialDampingFactor = 1.0;
        /// <summary>
        /// [N.s/m] Lateral damping
        /// </summary>
        public double LateralDampingFactor = 0.0;

        // Structural damping - calculated
        public double CalculatedTorsionalDamping;
        public double CalculatedAxialDamping;
        public double CalculateLateralDamping;
        public double TotalLength;

        /// <summary>
        /// Characteristic drill pipe impedance
        /// </summary>
        public double CharacteristicDrillPipeImpedance;

        //IMU sensors - configure
        /// <summary>
        /// [m] Axial distance (relative to bit depth) of an IMU measuring downhole RPM and accelerations
        /// </summary>
        public double SensorDistanceFromBit = 63;
        /// <summary>
        /// [rad] Polar angle of the accelerometer misalignment
        /// </summary>
        public readonly double SensorMisalignmentPolarAngle = 0 * Math.PI / 180;
        /// <summary>
        /// [rad] Azimuth angle of the accelerometer misalignment
        /// </summary>
        public readonly double SensorMisalignmentAzimuthAngle = 0 * Math.PI / 180;
        /// <summary>
        /// Radial, tangential and axial displacements in the local frame of the accelerometer
        /// </summary>
        public readonly Vector<double> SensorDisplacementsInLocalFrame = Vector<double>.Build.Dense(3, 0);

        //IMU sensors - to be calculated
        public int IndexSensor;
        /// <summary>
        /// [m] Radial distance from centerline of the tool to the accelerometer
        /// </summary>
        public double SensorRadialDistance;


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
        private List<double> mergedComponentToolJointMaxOuterDiameter = new List<double> { 0.0 };
        private List<double> mergedComponentToolJointMinInnerDiameter = new List<double> { double.MaxValue };
        private double topOfString;
        
        public SimulatorDrillString(Configuration configuration)
        {
            double expectedElementLength = configuration.ElementLength;
            DrillString drillString = configuration.DrillString;
            double fluidDensity = configuration.FluidDensity;                               
            double bitDepth = configuration.BitDepth;
            topOfString = configuration.TopOfStringPosition;
            BitRadius = configuration.BitRadius;
            SensorDistanceFromBit = configuration.SensorDistanceFromBit; 
            SleeveDistancesFromBit = configuration.SleeveDistancesFromBit;
            SleeveTorsionalDampingCoefficient = configuration.SleeveDamping;
            TorsionalDampingFactor = configuration.TorsionalDamping;
            AxialDampingFactor = configuration.AxialDamping;
            LateralDampingFactor = configuration.LateralDamping;
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
                        if (componentTypeList[componentTypeList.Count - 1] == component.Type)
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
                            mergedComponentToolJointMaxOuterDiameter[mergedComponentToolJointMaxOuterDiameter.Count - 1] = Math.Max(mergedComponentToolJointMaxOuterDiameter[mergedComponentToolJointMaxOuterDiameter.Count - 1], part.OuterDiameter);
                            mergedComponentToolJointMinInnerDiameter[mergedComponentToolJointMinInnerDiameter.Count - 1] = Math.Min(mergedComponentToolJointMinInnerDiameter[mergedComponentToolJointMinInnerDiameter.Count - 1], part.InnerDiameter);
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
                            mergedComponentToolJointMaxOuterDiameter.Add(part.OuterDiameter);
                            mergedComponentToolJointMinInnerDiameter.Add(part.InnerDiameter);
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
            bool drillPipesFirst = componentTypeList[0] == DrillStringComponentTypes.DrillPipe ||componentTypeList[0] == DrillStringComponentTypes.HeavyWeightDrillPipe;            
            if (drillPipesFirst)
            {
                componentTypeList.Reverse();
                mergedComponentLength.Reverse();
                mergedComponentOuterRadius.Reverse();
                mergedComponentArea.Reverse();
                mergedComponentInertia.Reverse();
                mergedComponentYoungsModulus.Reverse();
                mergedComponentShearModulus.Reverse();
                mergedComponentDensity.Reverse();
                mergedComponentInnerArea.Reverse();
                mergedComponentOuterArea.Reverse();
                mergedComponentInnerRadius.Reverse();
                mergedComponentToolJointMaxOuterDiameter.Reverse();
                mergedComponentToolJointMinInnerDiameter.Reverse();

            }
            #endregion
            #region Element Discretization
            //  In here the lumped elements are discretized. There are 
            // two expected circumstances:
            //  I - the merged component length > 2 * Expected Element Length
            //       - divide the section in equally spaced elements
            // II - the merged component length < 2 * Expected Element Length
            //       - Use a single element
            //   The drill-string used for the simulation case might 
            // be smaller than the one provided through the microservice.
            // In this case, there will most likely be unused drill-pipe sections.
            // Those  
            
            //  Current position of the last elemebt in relation to the drill-string
            // length (from the bit)
            double lastElementPosition = 0;
            RelativeNodeDepth.Add(bitDepth);
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
                    // Check if the last node is still within the expected range
                    if (RelativeNodeDepth[RelativeNodeDepth.Count-1] >= topOfString)
                    {                          
                        // Divide by the number of elements
                        ElementLength.Add( mergedComponentLength[i] / (double) numberOfElementsInSection );
                        //The Element depth is always in-between nodes
                        ElementDepth.Add(RelativeNodeDepth[RelativeNodeDepth.Count-1] - 0.5 * ElementLength[ElementLength.Count-1]);
                        // The node depth
                        RelativeNodeDepth.Add(
                                RelativeNodeDepth[ElementDepth.Count-1] - ElementLength[ElementLength.Count-1]
                            );
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
                        ElementToolJointOuterArea.Add( Math.PI / 4.0 * mergedComponentToolJointMaxOuterDiameter[i] * mergedComponentToolJointMaxOuterDiameter[i] );
                        ElementToolJointInnerArea.Add( Math.PI / 4.0 * mergedComponentToolJointMinInnerDiameter[i] * mergedComponentToolJointMinInnerDiameter[i] );
                        double volume = mergedComponentArea[i] * mergedComponentLength[i] / (double) numberOfElementsInSection;
                        ElementFluidAddedMass.Add( fluidDensity * volume );
                        ElementEccentricMass.Add( mergedComponentDensity[i] * volume );
                        ElementEccentricity.Add( MassImbalancePercentage * mergedComponentOuterRadius[i]);
                        ElementWeightCorrectionFactor.Add(1.0);
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
                        InactiveElementToolJointOuterArea.Add( Math.PI / 4.0 * (mergedComponentToolJointMaxOuterDiameter[i] * mergedComponentToolJointMaxOuterDiameter[i]
                            - mergedComponentToolJointMinInnerDiameter[i] * mergedComponentToolJointMinInnerDiameter[i]) );
                        InactiveElementToolJointInnerArea.Add( Math.PI / 4.0 * mergedComponentToolJointMinInnerDiameter[i] * mergedComponentToolJointMinInnerDiameter[i] );
                        double volume = mergedComponentArea[i] * mergedComponentLength[i] / (double) numberOfElementsInSection;
                        InactiveElementFluidAddedMass.Add( fluidDensity * volume );
                        InactiveElementEccentricMass.Add( mergedComponentDensity[i] * volume );
                        InactiveElementEccentricity.Add( MassImbalancePercentage * mergedComponentOuterRadius[i] );        
                        InactiveElementWeightCorrectionFactor.Add(1.0);
                  
                    }
                    lastElementPosition += mergedComponentLength[i] / (double) numberOfElementsInSection;
                }     
            }
            #endregion

            // The lists always starts from the bit. However, the rest of the simulator uses from top-first
            ElementLength.Reverse();
            ElementDensity.Reverse();
            ElementOuterRadius.Reverse();
            ElementYoungModuli.Reverse();
            ElementShearModuli.Reverse();
            ElementInertia.Reverse();
            ElementArea.Reverse();
            ElementInnerRadius.Reverse();
            ElementInnerArea.Reverse();
            ElementOuterArea.Reverse();
            ElementToolJointOuterArea.Reverse();
            ElementToolJointInnerArea.Reverse();
            ElementEccentricMass.Reverse();
            ElementEccentricity.Reverse();
            ElementDepth.Reverse();
            ElementWeightCorrectionFactor.Reverse();
            RelativeNodeDepth.Reverse();

            InactiveElementLength.Reverse();
            InactiveElementDensity.Reverse();
            InactiveElementOuterRadius.Reverse();
            InactiveElementYoungModuli.Reverse();
            InactiveElementShearModuli.Reverse();
            InactiveElementInertia.Reverse();
            InactiveElementArea.Reverse();
            InactiveElementInnerRadius.Reverse();
            InactiveElementInnerArea.Reverse();
            InactiveElementOuterArea.Reverse();
            InactiveElementToolJointOuterArea.Reverse();
            InactiveElementToolJointInnerArea.Reverse();
            InactiveElementEccentricMass.Reverse();
            InactiveElementEccentricity.Reverse();
            InactiveElementWeightCorrectionFactor.Reverse();

            IndexSensor = Enumerable.Range(0, RelativeNodeDepth.Count)
                .MinBy(i => Math.Abs(RelativeNodeDepth[i] - SensorDistanceFromBit));

            SleeveIndexPosition = SleeveDistancesFromBit
                .Select(sleeveDepth => Enumerable.Range(0, ElementDepth.Count)
                    .MinBy(i => Math.Abs(RelativeNodeDepth[i] - sleeveDepth)))
                .ToList();
            SleeveMassMomentOfInertia = Math.PI / 2.0 * SteelDensity * SleeveLength
                * (Math.Pow(SleeveOuterRadius, 4) - Math.Pow(SleeveInnerRadius, 4)); // [kg.m^2]
        }
        public void ActivateElements()
        {
             // Insert first inactive element at the top of each active list
            ElementLength.Insert(0, InactiveElementLength[0]);
            ElementDensity.Insert(0, InactiveElementDensity[0]);
            ElementOuterRadius.Insert(0, InactiveElementOuterRadius[0]);
            ElementInnerRadius.Insert(0, InactiveElementInnerRadius[0]);
            ElementEccentricity.Insert(0, InactiveElementEccentricity[0]);
            ElementOuterArea.Insert(0, InactiveElementOuterArea[0]);
            ElementInnerArea.Insert(0, InactiveElementInnerArea[0]);
            ElementToolJointOuterArea.Insert(0, InactiveElementToolJointOuterArea[0]);
            ElementToolJointInnerArea.Insert(0, InactiveElementToolJointInnerArea[0]);
            ElementPolarInertia.Insert(0, InactiveElementPolarInertia[0]);
            ElementArea.Insert(0, InactiveElementArea[0]);
            ElementInertia.Insert(0, InactiveElementInertia[0]);
            ElementWeightCorrectionFactor.Insert(0, InactiveElementWeightCorrectionFactor[0]);
            ElementYoungModuli.Insert(0, InactiveElementYoungModuli[0]);
            ElementShearModuli.Insert(0, InactiveElementShearModuli[0]);
            ElementFluidAddedMass.Insert(0, InactiveElementFluidAddedMass[0]);
            ElementEccentricMass.Insert(0, InactiveElementEccentricMass[0]);
            RelativeNodeDepth.Insert(0, topOfString);
            // Add the length of the new element to all the depth nodes
            for (int i = 1; i < RelativeNodeDepth.Count; i++)
            {
                RelativeNodeDepth[i] += ElementLength[0];
            }

            // Remove the first element from each inactive list, only if it has more than 1 element
            if (InactiveElementLength.Count > 1) InactiveElementLength.RemoveAt(0);
            if (InactiveElementDensity.Count > 1) InactiveElementDensity.RemoveAt(0);
            if (InactiveElementOuterRadius.Count > 1) InactiveElementOuterRadius.RemoveAt(0);
            if (InactiveElementInnerRadius.Count > 1) InactiveElementInnerRadius.RemoveAt(0);
            if (InactiveElementEccentricity.Count > 1) InactiveElementEccentricity.RemoveAt(0);
            if (InactiveElementOuterArea.Count > 1) InactiveElementOuterArea.RemoveAt(0);
            if (InactiveElementInnerArea.Count > 1) InactiveElementInnerArea.RemoveAt(0);
            if (InactiveElementToolJointOuterArea.Count > 1) InactiveElementToolJointOuterArea.RemoveAt(0);
            if (InactiveElementToolJointInnerArea.Count > 1) InactiveElementToolJointInnerArea.RemoveAt(0);
            if (InactiveElementPolarInertia.Count > 1) InactiveElementPolarInertia.RemoveAt(0);
            if (InactiveElementArea.Count > 1) InactiveElementArea.RemoveAt(0);
            if (InactiveElementInertia.Count > 1) InactiveElementInertia.RemoveAt(0);
            if (InactiveElementWeightCorrectionFactor.Count > 1) InactiveElementWeightCorrectionFactor.RemoveAt(0);
            if (InactiveElementYoungModuli.Count > 1) InactiveElementYoungModuli.RemoveAt(0);
            if (InactiveElementShearModuli.Count > 1) InactiveElementShearModuli.RemoveAt(0);
            if (InactiveElementFluidAddedMass.Count > 1) InactiveElementFluidAddedMass.RemoveAt(0);
            if (InactiveElementEccentricMass.Count > 1) InactiveElementEccentricMass.RemoveAt(0);
            



        }
    }
}
