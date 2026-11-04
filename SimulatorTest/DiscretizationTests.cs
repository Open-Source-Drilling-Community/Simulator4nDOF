using System.Runtime.CompilerServices;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;

namespace NORCE.Drilling.Simulator4nDOF.SimulatorTest
{
    public class DiscretizationTests
    {
        /// <summary>
        /// Creates a drill-string with the given lateral element lengths, grouped into
        /// axial-torsional elements with <paramref name="lateralCounts"/> lateral elements each.
        /// </summary>
        private static SimulatorDrillString CreateDrillString(List<double> elementLengths, List<int> lateralCounts)
        {
            var drillString = (SimulatorDrillString)RuntimeHelpers.GetUninitializedObject(typeof(SimulatorDrillString));
            int n = elementLengths.Count;
            drillString.ElementLength = elementLengths;
            drillString.ElementDensity = Enumerable.Repeat(7850.0, n).ToList();
            drillString.ElementArea = Enumerable.Repeat(3.0e-3, n).ToList();
            drillString.ElementPolarInertia = Enumerable.Repeat(1.0e-5, n).ToList();
            drillString.ElementYoungModuli = Enumerable.Repeat(2.1e11, n).ToList();
            drillString.ElementShearModuli = Enumerable.Repeat(8.0e10, n).ToList();
            drillString.AxialElementLateralCount = lateralCounts;
            drillString.AxialElementLength = new();
            drillString.AxialElementDensity = new();
            drillString.AxialElementArea = new();
            drillString.AxialElementPolarInertia = new();
            drillString.AxialElementYoungModuli = new();
            drillString.AxialElementShearModuli = new();
            drillString.AxialToLateralNode = new();
            drillString.LateralToAxialElement = new();
            drillString.LateralNaturalCoordinate = new();
            drillString.BuildAxialGrid();
            return drillString;
        }

        private static readonly List<double> UnevenLengths = new() { 5, 5, 5, 4, 4, 4, 6, 6, 6, 6 };
        private static readonly List<int> UnevenCounts = new() { 3, 3, 4 };

        private static double[] LateralPositions(SimulatorDrillString drillString)
        {
            double[] positions = new double[drillString.ElementLength.Count + 1];
            for (int k = 1; k < positions.Length; k++)
                positions[k] = positions[k - 1] + drillString.ElementLength[k - 1];
            return positions;
        }

        private static double[] AxialPositions(SimulatorDrillString drillString)
        {
            double[] positions = new double[drillString.AxialElementLength.Count + 1];
            for (int j = 1; j < positions.Length; j++)
                positions[j] = positions[j - 1] + drillString.AxialElementLength[j - 1];
            return positions;
        }

        [Test]
        public void AxialNodesCoincideWithLateralNodes()
        {
            var drillString = CreateDrillString(UnevenLengths, UnevenCounts);
            double[] lateral = LateralPositions(drillString);
            double[] axial = AxialPositions(drillString);

            Assert.That(drillString.AxialToLateralNode, Is.EqualTo(new List<int> { 0, 3, 6, 10 }));
            for (int j = 0; j < axial.Length; j++)
            {
                Assert.That(lateral[drillString.AxialToLateralNode[j]], Is.EqualTo(axial[j]).Within(1e-12));
            }
            Assert.That(drillString.LateralNaturalCoordinate[0], Is.EqualTo(0.0));
            Assert.That(drillString.LateralNaturalCoordinate[^1], Is.EqualTo(1.0));
            Assert.That(drillString.LateralToAxialElement.Count, Is.EqualTo(lateral.Length));
        }

        [Test]
        public void LinearFieldIsReproducedAtTheLateralNodes()
        {
            // Patch test: a linear field on the axial-torsional nodes is exact at every lateral node
            var drillString = CreateDrillString(UnevenLengths, UnevenCounts);
            double[] lateral = LateralPositions(drillString);
            List<double> axialField = AxialPositions(drillString).Select(s => 2.0 - 0.3 * s).ToList();

            double[] interpolated = new double[lateral.Length];
            drillString.InterpolateToLateral(axialField, interpolated);

            Assert.That(interpolated, Is.EqualTo(lateral.Select(s => 2.0 - 0.3 * s).ToArray()).Within(1e-12));
        }

        [Test]
        public void ConsistentNodalLoadPreservesForceAndMoment()
        {
            var drillString = CreateDrillString(UnevenLengths, UnevenCounts);
            double[] lateral = LateralPositions(drillString);
            double[] axial = AxialPositions(drillString);
            double[] pointLoads = lateral.Select((s, k) => Math.Sin(k + 1.0) * 100.0).ToArray();

            double[] nodalLoads = new double[axial.Length];
            drillString.ConsistentNodalLoad(pointLoads, nodalLoads);

            Assert.That(nodalLoads.Sum(), Is.EqualTo(pointLoads.Sum()).Within(1e-9));
            double lateralMoment = pointLoads.Select((f, k) => f * lateral[k]).Sum();
            double axialMoment = nodalLoads.Select((f, j) => f * axial[j]).Sum();
            Assert.That(axialMoment, Is.EqualTo(lateralMoment).Within(1e-9));
        }

        [Test]
        public void SameDiscretizationGivesIdentityMapping()
        {
            var drillString = CreateDrillString(UnevenLengths, Enumerable.Repeat(1, UnevenLengths.Count).ToList());
            double[] values = Enumerable.Range(0, UnevenLengths.Count + 1).Select(k => k * 1.5 - 3.0).ToArray();

            double[] interpolated = new double[values.Length];
            drillString.InterpolateToLateral(values, interpolated);
            double[] nodalLoads = new double[values.Length];
            drillString.ConsistentNodalLoad(values, nodalLoads);

            Assert.That(interpolated, Is.EqualTo(values).Within(1e-12));
            Assert.That(nodalLoads, Is.EqualTo(values).Within(1e-12));
            Assert.That(drillString.AxialElementLength, Is.EqualTo(UnevenLengths).Within(1e-12));
        }

        [Test]
        public void ShortAxialElementsAreMergedUntilMinimumLength()
        {
            // Bit-first: bit 0.3 m, sub 1 m, stabilizer 2 m, collars 3 x 9 m, short remainder 0.5 m
            List<double> lengths = new() { 0.3, 1.0, 2.0, 9.0, 9.0, 9.0, 0.5 };
            List<int> counts = Enumerable.Repeat(1, lengths.Count).ToList();

            List<int> merged = SimulatorDrillString.MergeShortAxialElements(counts, lengths, 10.0);

            // 0.3 + 1 + 2 + 9 = 12.3 m, then 9 + 9 = 18 m, and the 0.5 m remainder joins the last element
            Assert.That(merged, Is.EqualTo(new List<int> { 4, 3 }));
            Assert.That(merged.Sum(), Is.EqualTo(lengths.Count));
            Assert.That(SimulatorDrillString.MergeShortAxialElements(counts, lengths, 0.0), Is.EqualTo(counts));
        }

        [Test]
        public void MergedElementKeepsMassInertiaAndStiffness()
        {
            List<double> lengths = new() { 0.5, 2.0, 7.5 };
            var drillString = CreateDrillString(lengths, new List<int> { 3 });
            drillString.ElementDensity = new() { 7850, 8000, 7850 };
            drillString.ElementArea = new() { 1.0e-2, 4.0e-3, 3.0e-3 };
            drillString.ElementPolarInertia = new() { 4.0e-5, 1.5e-5, 1.0e-5 };
            drillString.ElementYoungModuli = new() { 2.1e11, 1.9e11, 2.1e11 };
            drillString.ElementShearModuli = new() { 8.0e10, 7.3e10, 8.0e10 };
            drillString.BuildAxialGrid();

            double length = drillString.AxialElementLength[0];
            double density = drillString.AxialElementDensity[0];
            double area = drillString.AxialElementArea[0];
            double polarInertia = drillString.AxialElementPolarInertia[0];
            double expectedMass = 0, expectedInertia = 0, axialFlexibility = 0, torsionalFlexibility = 0;
            for (int k = 0; k < lengths.Count; k++)
            {
                expectedMass += drillString.ElementDensity[k] * drillString.ElementArea[k] * lengths[k];
                expectedInertia += drillString.ElementDensity[k] * drillString.ElementPolarInertia[k] * lengths[k];
                axialFlexibility += lengths[k] / (drillString.ElementYoungModuli[k] * drillString.ElementArea[k]);
                torsionalFlexibility += lengths[k] / (drillString.ElementShearModuli[k] * drillString.ElementPolarInertia[k]);
            }

            Assert.That(length, Is.EqualTo(10.0).Within(1e-12));
            Assert.That(density * area * length, Is.EqualTo(expectedMass).Within(1e-9 * expectedMass));
            Assert.That(density * polarInertia * length, Is.EqualTo(expectedInertia).Within(1e-9 * expectedInertia));
            Assert.That(drillString.AxialElementYoungModuli[0] * area / length, Is.EqualTo(1.0 / axialFlexibility).Within(1e-9 / axialFlexibility));
            Assert.That(drillString.AxialElementShearModuli[0] * polarInertia / length, Is.EqualTo(1.0 / torsionalFlexibility).Within(1e-9 / torsionalFlexibility));
        }
    }
}
