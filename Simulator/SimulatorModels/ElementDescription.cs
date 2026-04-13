using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{   
    /// <summary>
    ///     ElementDescription is used to transform from a DrillString object extracted
    /// from the DrillString microservice and create a simulation ready format. This 
    /// implies that each similar consecutive part of the drill-string will have an 
    /// averaged behaviour with all of the material properties. This is implemented as
    /// a separete class as it can be imported in the future for other models. 
    /// 
    ///     Element description must not contain method-related variables (e.g.: stiffness values).
    /// Those are expected to be calculated at the model's constructor level. 
    /// </summary>
    public class ElementDescription
    {
        public List<double> ElementYoungModuli = new();
        public List<double> ElementShearModuli = new();                        
        public List<double> ElementArea = new();
        public List<double> ElementInertia = new();
        public List<double> ElementDensity = new();
        public List<double> ElementOuterRadius = new();
        public List<double> ElementLength = new();            
        //inactive drill-string elements
        public List<double> InactiveElementYoungModuli = new();
        public List<double> InactiveElementShearModuli = new();                        
        public List<double> InactiveElementArea = new();
        public List<double> InactiveElementInertia = new();
        public List<double> InactiveElementDensity = new();
        public List<double> InactiveElementOuterRadius = new();
        public List<double> InactiveElementLength = new();        
        private List<double> mergedComponentLength = new List<double> { 0.0 };            
        private List<double> mergedComponentOuterRadius = new List<double> { 0.0 };
        private List<double> mergedComponentArea = new List<double> { 0.0 };
        private List<double> mergedComponentInertia = new List<double> { 0.0 };
        private List<double> mergedComponentYoungsModulus = new List<double> { 0.0 };
        private List<double> mergedComponentShearModulus = new List<double> { 0.0 };     
        private List<double> mergedComponentDensity = new List<double> { 0.0 };           
        public ElementDescription(in SimulationParameters simulationParameters, 
            in DrillString drillString)
        {
            // Expected behaviour:
            // Simplify geometry by merging similar components 
            //  We try to keep the element size as constant as possible. 
            // If there is a very small component, it uses it's own element 
            // (MergedComponentLength < 2 * ExpectedElementLength)                         
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
            double expectedElementLength = simulationParameters.ElementLength;
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
                    if (lastElementPosition <= simulationParameters.DrillStringLength)
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
                    }
                    lastElementPosition += mergedComponentLength[i] / (double) numberOfElementsInSection;
                }     
            }            
            #endregion
        }
    }
}
