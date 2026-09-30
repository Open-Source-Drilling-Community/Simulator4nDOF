using System.Runtime.CompilerServices;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.SimulatorTest
{
    public class SimulatorTrajectoryTests
    {
        private static SimulatorDrillString CreateDrillString(int elementCount, double elementLength)
        {
            // Only the node depths and element lengths are used by SimulatorTrajectory
            var drillString = (SimulatorDrillString)RuntimeHelpers.GetUninitializedObject(typeof(SimulatorDrillString));
            drillString.ElementLength = Enumerable.Repeat(elementLength, elementCount).ToList();
            drillString.RelativeNodeDepth = Enumerable.Range(0, elementCount + 1).Select(i => i * elementLength).ToList();
            return drillString;
        }

        private static List<SurveyStation> CreateStations()
        {
            return new List<SurveyStation>
            {
                new SurveyStation { MD = 0, Inclination = 0, Azimuth = 0, TVD = 0 },
                new SurveyStation { MD = 300, Inclination = 10, Azimuth = 45, TVD = 298 },
                new SurveyStation { MD = 600, Inclination = 30, Azimuth = 60, TVD = 570 },
                new SurveyStation { MD = 1000, Inclination = 45, Azimuth = 70, TVD = 870 },
            };
        }

        [Test]
        public void SurveyRunAndTrajectoryWithSameStationsGiveSameResult()
        {
            var drillString = CreateDrillString(50, 20);
            var fromTrajectory = new SimulatorTrajectory(drillString, new Trajectory { SurveyStationList = CreateStations() });
            var fromSurveyRun = new SimulatorTrajectory(drillString, new SurveyRun { SurveyStationList = CreateStations() });

            Assert.That(fromSurveyRun.InterpolatedThetaAtNode.ToArray(), Is.EqualTo(fromTrajectory.InterpolatedThetaAtNode.ToArray()).Within(1e-12));
            Assert.That(fromSurveyRun.InterpolatedPhiAtNode.ToArray(), Is.EqualTo(fromTrajectory.InterpolatedPhiAtNode.ToArray()).Within(1e-12));
            Assert.That(fromSurveyRun.Curvature.ToArray(), Is.EqualTo(fromTrajectory.Curvature.ToArray()).Within(1e-12));
        }

        [Test]
        public void StationsAreSortedByMeasuredDepth()
        {
            var drillString = CreateDrillString(50, 20);
            var shuffled = CreateStations();
            shuffled.Reverse();
            var sorted = new SimulatorTrajectory(drillString, new Trajectory { SurveyStationList = CreateStations() });
            var unsorted = new SimulatorTrajectory(drillString, new Trajectory { SurveyStationList = shuffled });

            Assert.That(unsorted.InterpolatedThetaAtNode.ToArray(), Is.EqualTo(sorted.InterpolatedThetaAtNode.ToArray()).Within(1e-12));
        }

        [Test]
        public void SurveyRunTieInPointAboveFirstStationIsPrepended()
        {
            var drillString = CreateDrillString(50, 20);
            var stationsBelowTieIn = CreateStations().Skip(1).ToList();
            var surveyRun = new SurveyRun
            {
                TieInPoint = new SurveyStation { MD = 0, Inclination = 0, Azimuth = 0, TVD = 0 },
                SurveyStationList = stationsBelowTieIn
            };
            var withTieIn = new SimulatorTrajectory(drillString, surveyRun);
            var fullTrajectory = new SimulatorTrajectory(drillString, new Trajectory { SurveyStationList = CreateStations() });

            Assert.That(withTieIn.InterpolatedThetaAtNode.ToArray(), Is.EqualTo(fullTrajectory.InterpolatedThetaAtNode.ToArray()).Within(1e-12));
        }

        [Test]
        public void MissingOrTooFewStationsThrow()
        {
            var drillString = CreateDrillString(10, 20);
            Assert.Throws<ArgumentException>(() => new SimulatorTrajectory(drillString, new SurveyRun { SurveyStationList = null }));
            Assert.Throws<ArgumentException>(() => new SimulatorTrajectory(drillString, new Trajectory { SurveyStationList = CreateStations().Take(1).ToList() }));
        }
    }
}
