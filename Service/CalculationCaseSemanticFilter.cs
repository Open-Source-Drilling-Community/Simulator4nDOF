using Microsoft.OpenApi.Any;
using Microsoft.OpenApi.Models;
using OSDC.DotnetLibraries.Drilling.SemanticCatalogue;
using Swashbuckle.AspNetCore.SwaggerGen;

namespace NORCE.Drilling.Simulator4nDOF.Service;

public sealed class CalculationCaseSemanticFilter : ISchemaFilter, IOperationFilter
{
    public void Apply(OpenApiSchema schema, SchemaFilterContext context)
    {
        if (SemanticMetadata.For(context.Type) is { } typeMetadata)
            schema.Extensions[SemanticMetadata.ExtensionName] = OpenApiAnyFactory.CreateFromJson(typeMetadata.ToJsonString());
        foreach (var property in context.Type.GetProperties())
            if (schema.Properties.TryGetValue(property.Name, out var target) && SemanticMetadata.For(property) is { } metadata)
                Attach(target, OpenApiAnyFactory.CreateFromJson(metadata.ToJsonString()));
    }

    public void Apply(OpenApiOperation operation, OperationFilterContext context)
    {
        if (SemanticMetadata.For(context.MethodInfo) is { } metadata)
            operation.Extensions[SemanticMetadata.ExtensionName] = OpenApiAnyFactory.CreateFromJson(metadata.ToJsonString());
    }

    private static void Attach(OpenApiSchema target, IOpenApiAny metadata)
    {
        // OpenAPI ignores siblings of $ref. Preserve the reference through allOf so
        // the property-level semantic extension remains part of the wire contract.
        if (target.Reference is { } reference)
        {
            target.Reference = null;
            target.AllOf.Add(new OpenApiSchema { Reference = reference });
        }
        target.Extensions[SemanticMetadata.ExtensionName] = metadata;
    }
}
