using Microsoft.OpenApi.Models;
using Swashbuckle.AspNetCore.SwaggerGen;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace NORCE.Drilling.Simulator4nDOF.Service
{
    public static class JsonSettings
    {
        public static readonly JsonSerializerOptions Options = Create();

        private static JsonSerializerOptions Create()
        {
            var options = new JsonSerializerOptions
            {
                PropertyNamingPolicy = null                             // Preserve C# properties naming conventions (no forced lower case applied)
            };

            options.Converters.Add(new JsonStringEnumConverter());      // Serialize enums as their string names instead of numeric values
            return options;
        }

        public static void ApplyTo(JsonSerializerOptions target)
        {
            target.PropertyNamingPolicy = Options.PropertyNamingPolicy;

            foreach (var converter in Options.Converters)
            {
                target.Converters.Add(converter);
            }
        }
    }
}
