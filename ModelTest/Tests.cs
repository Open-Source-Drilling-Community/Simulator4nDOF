using OSDC.DotnetLibraries.Drilling.DrillingProperties;
using OSDC.DotnetLibraries.General.DataManagement;
using OSDC.DotnetLibraries.General.Statistics;
using NORCE.Drilling.Simulator4nDOF.Model;

namespace NORCE.Drilling.Simulator4nDOF.ModelTest
{
    public class Tests
    {
        [OneTimeSetUp]
        public void OneTimeSetUp()
        {
        }

        [Test]
        public void Test_Calculus()
        {
            Guid guid = Guid.NewGuid();
            MetaInfo metaInfo = new() { ID = guid };
            DateTimeOffset creationDate = DateTimeOffset.UtcNow;

            Guid guid2 = Guid.NewGuid();
            MetaInfo metaInfo2 = new() { ID = guid2 };
            DateTimeOffset creationDate2 = DateTimeOffset.UtcNow;
            ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } };
            Model.DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
            ToBeRemoved toBeRemoved = new()
            {
                MetaInfo = metaInfo2,
                Name = "My test ToBeRemoved name",
                Description = "My test ToBeRemoved for POST",
                CreationDate = creationDate,
                LastModificationDate = creationDate2,
                ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } },
                DerivedData1 = derivedData1,
                Type = ToBeRemovedType.DerivedData1
            };
            Model.Simulation simulation = new()
            {
                MetaInfo = metaInfo,
                Name = "My test Simulation",
                Description = "My test Simulation",
                CreationDate = creationDate,
                LastModificationDate = creationDate,
                ToBeRemovedList = [toBeRemoved],
            };

            Assert.That(simulation.OutputParam, Is.Null);
            simulation.Calculate();
            Assert.That(simulation.OutputParam, Is.EqualTo(3));
        }

        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
        }
    }
}