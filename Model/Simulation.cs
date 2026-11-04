using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using OSDC.DotnetLibraries.Drilling.SemanticCatalogue;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    [Semantic(Concepts.CalculationCase)]
    public class Simulation
    {
        public MetaInfo? MetaInfo { get; set; } = null;
        public string? Name { get; set; } = null;
        public string? Description { get; set; } = null;
        public DateTimeOffset? CreationDate { get; set; } = null;
        public DateTimeOffset? LastModificationDate { get; set; } = null;
        public ContextualData? ContextualData { get; set; } = null;
        public Guid? WellBoreID { get; set; } = null;
        [Semantic(Concepts.CalculationSpecification, Role = Concepts.CalculationInput)]
        public Config? Config { get; set; } = null;
        public InitialValues? InitialValues { get; set; } = null;
        public List<SetPoints>? SetPointsList { get; set; } = null;
        public double CurrentTime { get; set; } 
        [Semantic(Concepts.CalculationProgress)]
        public double? Progress { get; set; } = null;
        [Semantic(Concepts.CalculationState)]
        public int? TerminationState { get; set; } = null;
        [Semantic(Concepts.CalculationResult, Role = Concepts.ServerDerivedCalculationResult)]
        public Results? Results { get; set; } = null;

    }
}
