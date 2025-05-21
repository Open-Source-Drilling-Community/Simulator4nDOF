using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Simulation
    {
        public MetaInfo MetaInfo { get; set; }
        public string? Name { get; set; }
        public string? Description { get; set; }
        public DateTimeOffset? CreationDate { get; set; }
        public DateTimeOffset? LastModificationDate { get; set; }
        public ContextualData ContextualData { get; set; }
        public double CurrentTime { get; set; }
        public InitialValues InitialValues { get; set; }
        public double? Progress { get; set; }
        public int? TerminationState { get; set; }
        public List<SetPoints>? SetPointsList { get; set; }
        public List<Results>? Results { get; set; }


        /// <summary>
        /// an input list of SetPoints
        /// </summary>

        //

        /// <summary>
        /// an output parameter, result of the Calculate() method
        /// </summary>
        //public double? OutputParam { get; set; }

        /// <summary>
        /// main calculation method of the Simulator
        /// </summary>
        /// <returns></returns>
        public bool Calculate()
        {
            bool success = false;
            //OutputParam = null;
            // if (SetPointsList != null)
            // {
            // foreach (var setPoints in SetPointsList)
            // {
            //     OutputParam = (OutputParam ?? 0) +
            //         (setPoints?.SetPointsParam?.DiracDistributionValue?.Value ?? 0) +
            //         (setPoints?.DerivedData1?.DerivedData1Param?.DiracDistributionValue?.Value ?? 0) +
            //         (setPoints?.DerivedData2?.DerivedData2Param ?? 0);
            // }
            success = true;
            // }
            return success;
        }
    }
}
