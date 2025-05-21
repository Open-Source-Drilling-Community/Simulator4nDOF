using OSDC.DotnetLibraries.Drilling.DrillingProperties;
namespace NORCE.Drilling.Simulator4nDOF.Model
{
    /// <summary>
    /// a class deriving from the ToBeRemoved base class
    /// ASSUMPTION: for each ToBeRemoved instance, one, and only one, of the derived data should be instanciated, while others remain null
    /// </summary>
    public class DerivedData1
    {
        /// <summary>
        /// a parameter for the derived data defined as a Gaussian distribution 
        /// </summary>
        public ScalarDrillingProperty? DerivedData1Param { get; set; }
    }
}
