using System.Text.Json.Nodes;

namespace NORCE.Drilling.Simulator4nDOF.ServiceTest;

public class CalculationCaseDeletionSemanticTests
{
    [Test]
    public void Delete_operation_publishes_reviewed_calculation_case_deletion_semantics()
    {
        string schemaPath = Path.GetFullPath(Path.Combine(
            TestContext.CurrentContext.TestDirectory,
            "..", "..", "..", "..", "Service", "wwwroot", "json-schema", "Simulator4nDOFMergedModel.json"));
        JsonObject document = JsonNode.Parse(File.ReadAllText(schemaPath))!.AsObject();
        JsonObject operation = document["paths"]!.AsObject()
            .SelectMany(path => path.Value!.AsObject().Select(method => method.Value!.AsObject()))
            .Single(candidate => candidate["operationId"]?.GetValue<string>() == "DeleteSimulationById");
        JsonObject semantic = operation["x-osdc-semantic"]!.AsObject();

        Assert.Multiple(() =>
        {
            Assert.That(semantic["catalogueVersion"]!.GetValue<string>(), Is.EqualTo("0.16.0"));
            Assert.That(semantic["concept"]!.GetValue<string>(), Is.EqualTo("urn:osdc:semantic:calculation-case"));
            Assert.That(semantic["role"]!.GetValue<string>(), Is.EqualTo("urn:osdc:semantic:calculation-case-deletion"));
            Assert.That(semantic["curationStatus"]!.GetValue<string>(), Is.EqualTo("Reviewed"));
        });
    }
}

