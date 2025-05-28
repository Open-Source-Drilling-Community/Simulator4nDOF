using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator;
using NORCE.Drilling.Simulator.DataModel;
using NORCE.Drilling.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.DataManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;

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
        public Results? Results { get; set; }

    }
}
