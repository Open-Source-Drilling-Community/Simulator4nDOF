using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class Simulation
    {
        public MetaInfo? MetaInfo { get; set; } = null;
        public string? Name { get; set; } = null;
        public string? Description { get; set; } = null;
        public DateTimeOffset? CreationDate { get; set; } = null;
        public DateTimeOffset? LastModificationDate { get; set; } = null;
        public ContextualData? ContextualData { get; set; } = null;
        public Guid? WellBoreID { get; set; } = null;
        public Config? Config { get; set; } = null;
        public InitialValues? InitialValues { get; set; } = null;
        public List<SetPoints>? SetPointsList { get; set; } = null;
        public double CurrentTime { get; set; } 
        public double? Progress { get; set; } = null;
        public int? TerminationState { get; set; } = null;
        public Results? Results { get; set; } = null;

    }
}
