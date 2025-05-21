using Microsoft.OpenApi.Models;
using Microsoft.OpenApi.Any;
using Microsoft.OpenApi.Interfaces;

public class OpenApiSchemaReferenceUpdater
{
    private readonly Dictionary<string, string> _renamedSchemas = new();

    public void MergeSchemasAndUpdateRefs(OpenApiDocument target, OpenApiDocument source, Func<string, string> keyTransformer)
    {
        foreach (var kv in source.Components.Schemas)
        {
            string oldKey = kv.Key;
            OpenApiSchema schema = kv.Value;

            string newKey = keyTransformer(oldKey);

            if (oldKey != newKey)
                _renamedSchemas[oldKey] = newKey;

            var clonedSchema = CloneSchema(schema);

            if (clonedSchema.Reference != null)
            {
                clonedSchema.Reference = new OpenApiReference
                {
                    Id = newKey,
                    Type = ReferenceType.Schema
                };
            }

            target.Components.Schemas[newKey] = clonedSchema;
        }

        UpdateAllReferences(target);
    }

    private void UpdateAllReferences(OpenApiDocument doc)
    {
        // fix references in Paths
        foreach (var path in doc.Paths.Values)
        {
            foreach (var operation in path.Operations.Values)
            {
                foreach (var param in operation.Parameters)
                {
                    if (param.Schema != null)
                        UpdateSchemaRef(param.Schema);
                }

                if (operation.RequestBody != null)
                {
                    foreach (var content in operation.RequestBody.Content.Values)
                        UpdateSchemaRef(content.Schema);
                }

                foreach (var response in operation.Responses.Values)
                {
                    foreach (var content in response.Content.Values)
                        UpdateSchemaRef(content.Schema);
                }
            }
        }

        // fix references in Components
        foreach (var param in doc.Components.Parameters.Values)
        {
            if (param.Schema != null)
                UpdateSchemaRef(param.Schema);
        }

        foreach (var requestBody in doc.Components.RequestBodies.Values)
        {
            foreach (var content in requestBody.Content.Values)
                UpdateSchemaRef(content.Schema);
        }

        foreach (var response in doc.Components.Responses.Values)
        {
            foreach (var content in response.Content.Values)
                UpdateSchemaRef(content.Schema);
        }

        // nested references in Components.Schemas
        foreach (var schema in doc.Components.Schemas.Values)
        {
            UpdateSchemaRef(schema);
        }
    }

    private void UpdateSchemaRef(OpenApiSchema schema)
    {
        if (schema == null)
            return;

        if (schema.Reference != null &&
            schema.Reference.Type == ReferenceType.Schema &&
            _renamedSchemas.TryGetValue(schema.Reference.Id, out var newId))
        {
            schema.Reference = new OpenApiReference
            {
                Type = ReferenceType.Schema,
                Id = newId
            };
        }

        foreach (var property in schema.Properties.Values)
            UpdateSchemaRef(property);

        if (schema.Items != null)
            UpdateSchemaRef(schema.Items);

        if (schema.AdditionalProperties is OpenApiSchema aps)
            UpdateSchemaRef(aps);

        if (schema.AllOf != null)
            foreach (var sub in schema.AllOf)
                UpdateSchemaRef(sub);

        if (schema.AnyOf != null)
            foreach (var sub in schema.AnyOf)
                UpdateSchemaRef(sub);

        if (schema.OneOf != null)
            foreach (var sub in schema.OneOf)
                UpdateSchemaRef(sub);
    }

    /// <summary>
    /// manual deep cloning of OpenApi schemas (all other options, more automated, failed cloning Properties)
    /// </summary>
    /// <param name="source"></param>
    /// <returns></returns>
    private OpenApiSchema CloneSchema(OpenApiSchema source)
    {
        if (source == null) return null;

        var clone = new OpenApiSchema
        {
            Title = source.Title,
            Type = source.Type,
            Format = source.Format,
            Description = source.Description,
            Nullable = source.Nullable,
            Deprecated = source.Deprecated,
            ReadOnly = source.ReadOnly,
            WriteOnly = source.WriteOnly,
            ExternalDocs = source.ExternalDocs,
            Example = source.Example,
            Default = source.Default,
            Enum = new List<IOpenApiAny>(source.Enum),
            Reference = source.Reference != null
                ? new OpenApiReference
                {
                    Id = source.Reference.Id,
                    Type = source.Reference.Type
                }
                : null,
            Required = new HashSet<string>(source.Required),
            Discriminator = source.Discriminator,
            MaxItems = source.MaxItems,
            MinItems = source.MinItems,
            MaxLength = source.MaxLength,
            MinLength = source.MinLength,
            Maximum = source.Maximum,
            Minimum = source.Minimum,
            Pattern = source.Pattern,
            UniqueItems = source.UniqueItems,
            MultipleOf = source.MultipleOf,
            Extensions = new Dictionary<string, IOpenApiExtension>(source.Extensions)
        };

        // Deep clone properties
        foreach (var kvp in source.Properties)
        {
            clone.Properties.Add(kvp.Key, CloneSchema(kvp.Value));
        }

        if (source.Items != null)
            clone.Items = CloneSchema(source.Items);

        if (source.AdditionalProperties != null)
        {
            clone.AdditionalProperties = source.AdditionalProperties switch
            {
                OpenApiSchema schema => CloneSchema(schema),
                _ => source.AdditionalProperties
            };
        }

        if (source.AllOf != null)
        {
            clone.AllOf = new List<OpenApiSchema>();
            foreach (var s in source.AllOf)
                clone.AllOf.Add(CloneSchema(s));
        }

        if (source.AnyOf != null)
        {
            clone.AnyOf = new List<OpenApiSchema>();
            foreach (var s in source.AnyOf)
                clone.AnyOf.Add(CloneSchema(s));
        }

        if (source.OneOf != null)
        {
            clone.OneOf = new List<OpenApiSchema>();
            foreach (var s in source.OneOf)
                clone.OneOf.Add(CloneSchema(s));
        }

        return clone;
    }

}