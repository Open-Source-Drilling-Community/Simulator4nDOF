using OSDC.DotnetLibraries.General.DataManagement;
using System;
using OSDC.DotnetLibraries.Drilling.SemanticCatalogue;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    /// <summary>
    /// Light weight version of a Simulation
    /// Used to avoid loading the complete Simulation (heavy weight data) each time we only need contextual info on the data
    /// Typically used for listing, sorting and filtering purposes
    /// </summary>
    [Semantic(Concepts.CalculationCase, Role = Concepts.CalculationStatusProjection)]
    public class SimulationLight
    {
        public SimulationLight(MetaInfo? metaInfo, string? name, string? description, DateTimeOffset? creationDate, DateTimeOffset? lastModificationDate, Guid wellBoreID, double progress, int terminationState)
        {
            this.MetaInfo = metaInfo;
            this.Name = name;
            this.Description = description;
            this.CreationDate = creationDate;
            this.LastModificationDate = lastModificationDate;
            this.Progress = progress;
            this.TerminationState = terminationState;
            this.WellBoreID = wellBoreID;
        }

        public MetaInfo? MetaInfo { get; set; } = null;
        public string? Name { get; set; } = null;
        public string? Description { get; set; } = null;
        public DateTimeOffset? CreationDate { get; set; } = null;
        public DateTimeOffset? LastModificationDate { get; set; } = null;
        public Guid? WellBoreID { get; set; } = null;
        [Semantic(Concepts.CalculationProgress)]
        public double Progress { get; set; }
        [Semantic(Concepts.CalculationState)]
        public int TerminationState { get; set; }

    }
}
