using System;

namespace NORCE.Drilling.Simulator4nDOF.Model
{
    public class ContextualData
    {
        public Guid? DrillStringID { get; set; } = null;
        public Guid? DrillingFluidDescriptionID { get; set; } = null;
        public Guid? WellBoreArchitectureID { get; set; } = null;
        public Guid? TrajectoryID { get; set; } = null;
        public Guid? RigID { get; set; } = null;
        public int? CasingID { get; set; } = null;
        public double Temperature { get; set; } = 293;
        public double SurfacePipePressure { get; set; } = 100_000;
        public SolverType SolverType { get; set; }
    }
}
