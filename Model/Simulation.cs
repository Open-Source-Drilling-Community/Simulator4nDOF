using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator;
using NORCE.Drilling.Simulator.DataModel;
using NORCE.Drilling.Simulator.DataModel.ParametersModel;
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

        private Solver solver;

        private double outerTimeStep;

        public bool Initialize()
        {
            var config = new Configuration()
            {
                AnnulusPressureFile = ContextualData.AnnulusPressureFile,
                TrajectoryFile = ContextualData.TrajectoryFile,
                DrillstringFile = ContextualData.DrillstringFile,
                StringPressureFile = ContextualData.DrillstringPressureFile,
                BitDepth = InitialValues.BitDepth,                            // [m]
                HoleDepth = InitialValues.HoleDepth,                           // [m]
                TopOfStringPosition = InitialValues.TopOfStringPosition,                   // [m]
                SurfaceRPM = SetPointsList?.Count > 0 ? SetPointsList[0].SurfaceRPM : 0,            // [rad/s]
                TopOfStringVelocity = SetPointsList?.Count > 0 ? SetPointsList[0].TopOfStringVelocity : 0,         // [m/s] 
                CasingShoeDepth = ContextualData.CasingShoeDepth,                     // [m] Casing shoe depth
                LinerShoeDepth = ContextualData.LinerShoeDepth,                         // [m] Liner shoe depth, set to 0 if there is no liner
                CasingID = ContextualData.CasingID,                        // [m] Casing inner diameter
                LinerID = ContextualData.LinerID,                         // [m] Casing outer diameter
                WellheadDepth = ContextualData.WellheadDepth,                        // [m] Well head depth
                RiserID = ContextualData.RiserID,                        // [m]
                BitRadius = ContextualData.BitRadius,                     // [m]
                //SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700 }),
                SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { }),
                SensorDistanceFromBit = 63,
                FluidDensity = 1200,                       // [kg/m3] Density of drilling mud
                LengthBetweenLumpedElements = 30,
            };

            outerTimeStep = config.TimeStep;

            var parameters = new Parameters(c: config);
            
            solver = new Solver(parameters, in config);

            var success = true;
            return success;
        }
 
        public bool Calculate()
        {
            bool success = false;
            Results = new List<Results>();
            if (SetPointsList != null)
            {
                double totalDuration = 0;
                foreach (var setPoints in SetPointsList)
                {
                    totalDuration = totalDuration + setPoints.TimeDuration;
                }

                foreach (var setPoints in SetPointsList)
                {
                    double duration = setPoints.TimeDuration;
                    double steps = duration / outerTimeStep;

                    for (int i = 0; i < steps; i++)
                    {
                        (var state, var output, var u) = solver.OuterStep(setPoints.SurfaceRPM, setPoints.TopOfStringVelocity);
                        Results.Add(new Model.Results()
                        {
                            Time = state.step * outerTimeStep,
                            BitDepth = state.BitDepth,
                            BitVelocity = output.vb,
                            WOB = output.wob,
                            TOB = output.tob,
                            BitRPM = output.omega_b,
                            TopDriveRPM = output.omega_td,
                            HoleDepth = state.HoleDepth,
                            TopOfStringPosition = state.TopOfStringPosition,
                        });
                        Progress = state.step * outerTimeStep / totalDuration;
                    }
                }
                TerminationState = 1;
                success = true;
            }
            return success;
        }
    }
}
