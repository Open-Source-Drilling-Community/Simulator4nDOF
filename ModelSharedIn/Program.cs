using Microsoft.OpenApi.Models;
using Microsoft.OpenApi.Readers;
using Microsoft.OpenApi;
using NJsonSchema;
using NJsonSchema.CodeGeneration.CSharp;
using NSwag.CodeGeneration.CSharp;
using System.Text;
using Microsoft.OpenApi.Extensions;

/// <summary>
/// // ### BEGIN CODE SPECIFIC TO ModelSharedIn 1/3 ###
/// This console program aims at automatically generating the C# OpenAPI client and model of dependencies the current microservice depends on
/// // ### BEGIN CODE SPECIFIC TO ModelSharedIn 1/3 ###
/// 
///     - all model dependencies of the current microservice data model are stored in folder json-schemas/ as OpenApi documents (.json)
///     - the content of this folder is parsed automatically and documents are merged together in to a single OpenApi document (JSON_BUNDLE)
///     - merging process is composed of two steps
///             Schemas of each document is analyzed and schema Id are stripped to their short name (namespace removed)
///             Paths and Schemas are then parsed recursively to replace every schema Id and references to them by their short name
///             
/// ### BEGIN CODE SPECIFIC TO ModelSharedIn 2/3 ###
///     - from this merged OpenApi document
///             a C# data model CSHARP_MODEL is generated in ModelSharedIn so that it can be referenced by the Model project
///             a json formatted OpenApi document JSON_BUNDLE is generated in ModelSharedIn only for verification purpose
/// ### END CODE SPECIFIC TO ModelSharedIn 2/3 ###
/// 
/// It capitalizes on the concept of distributed shared model in a microservice architecture
///     - which means that a microservice handles the external classes it needs by itself, using the OpenAPI schema of its dependencies as a source of truth
///     - default option in this program expects the user to manually collect these dependency schemas
///             found here (NORCE-generated custom schema registration):    https://someServer:somePort/someMicroserviceDependency/api/swagger/merged/swagger.json
///             or here (default SwaggerUI schema registration):            https://someServer:somePort/someMicroserviceDependency/api/swagger/v1/swagger.json
///     - option 2 discovers these dependencies online each time the current program executes, the risk being that modifications brought to the dependencies by another team go unaware
///     - more info: https://github.com/NORCE-DrillingAndWells/DrillingAndWells/wiki/MS-Development#distributed-shared-data-model
/// </summary>
class Program
{
    private static bool finished_ = false;
    private static readonly object lock_ = new object();

    private static readonly string JSON_BUNDLE = "MergedModel.json";
    private static readonly string CSHARP_MODEL = "MergedModel.cs";

    // ### BEGIN CODE SPECIFIC TO ModelSharedIn 3/3 ###
    private static readonly string NAMESPACE = "NORCE.Drilling.Simulator4nDOF.ModelShared"; // should be the same as for ModelSharedIn to avoid type name collision
    private static readonly string SHARED_FOLDER = "ModelSharedIn";
    // ### END CODE SPECIFIC TO ModelSharedIn 3/3 ###

    private static readonly string PRETTY_STRING = SHARED_FOLDER;

    // option 2 variables
    // As an illustration, it is assumed that Model depends on Field
    //private static readonly List<string> MS_DEPENDENCIES = new List<string> { "UnitConversion" };
    //private static readonly string HOST = "https://dev.digiwells.no/";
    //private static readonly string ENDPOINT = "swagger/merged/swagger.json";

    static async Task Main()
    {
        bool finished = false;
        Console.Clear();
        Console.BackgroundColor = ConsoleColor.Black;
        do
        {
            Thread.Sleep(100);

            if (await GenerateModelFromDependencies())
                Console.BackgroundColor = ConsoleColor.Red;

            lock (lock_)
            {
                finished = finished_;
            }
        } while (!finished);

        return;
    }

    static async Task<bool> GenerateModelFromDependencies()
    {
        PrettyPrint(PRETTY_STRING, "Starting generation process (OpenApi document bundle and shared C# model)...");

        bool error = true;
        var directory = new DirectoryInfo(Directory.GetCurrentDirectory());
        while (directory != null && !directory.GetFiles("*.sln").Any())
        {
            directory = directory.Parent;
        }
        if (directory != null)
        {
            string modelSharedDir = directory.FullName + Path.DirectorySeparatorChar + SHARED_FOLDER;
            string jsonDirectory = modelSharedDir + Path.DirectorySeparatorChar + "json-schemas" + Path.DirectorySeparatorChar;
            string? res = "Y";
            if (File.Exists(modelSharedDir + Path.DirectorySeparatorChar + JSON_BUNDLE) || File.Exists(modelSharedDir + Path.DirectorySeparatorChar + CSHARP_MODEL))
            {
                PrettyPrint(PRETTY_STRING, $"Shared model files already exist!\n" +
                            "\tAre you sure you want to overwrite the following existing files?\n" +
                            $"\t\t- C# model (.{Path.DirectorySeparatorChar}{SHARED_FOLDER}{Path.DirectorySeparatorChar}{CSHARP_MODEL})\n" +
                            $"\t\t- OpenApi bundle (.{Path.DirectorySeparatorChar}{SHARED_FOLDER}{Path.DirectorySeparatorChar}{JSON_BUNDLE})\n" +
                            //// option 2: activate the following line in case of online dependency discovery
                            //$"\t\t- backups of the individual OpenApi documents (.\\json-schemas\\microserviceDependency.json)\n" +
                            "\tType Y for YES, or any other key for NO");
                res = Console.ReadLine();
            }
            if (res == "Y")
            {
                try
                {
                    if (Directory.Exists(jsonDirectory))
                    {
                        //// Option 2: Online discovery of service dependencies - and local storage
                        //foreach (string msi in MS_DEPENDENCIES)
                        //{
                        //    PrettyPrint(msi, "Processing Open Api doc into bundle...");
                        //    HttpClient httpClient = new HttpClient { BaseAddress = new Uri(HOST + msi + "/api/") };
                        //    using var stream = await httpClient.GetStreamAsync(ENDPOINT);
                        //    var doc = new OpenApiStreamReader().Read(stream, out var diagnostic);
                        //    var oString = doc.Serialize(OpenApiSpecVersion.OpenApi3_0, OpenApiFormat.Json);
                        //    // forcing backup of swagger.json in json-schemas/
                        //    PrettyPrint(msi, "Backup OpenApi doc (" + jsonDirectory + ")");
                        //    using (StreamWriter outputFile = new StreamWriter(jsonDirectory + Path.DirectorySeparatorChar + msi + ".json"))
                        //    {
                        //        outputFile.Write(oString);
                        //    }
                        //}
                        //Thread.Sleep(1000); // make sure files are written

                        // a bundle OpenApi schema (json format) is created to combine OpenApi schema dependencies (Microsoft.OpenApi.Models.OpenApiDocument is used rather than NSwag)
                        OpenApiDocument document = new OpenApiDocument
                        {
                            Info = new OpenApiInfo
                            {
                                Title = "ModelShared (Service)",
                                Description = "Data model containing model dependencies of the clients of the current microservice (WebApp, ServiceTest)",
                                Version = "1.0"
                            },
                            Components = new OpenApiComponents
                            {
                                Schemas = new Dictionary<string, OpenApiSchema> { }
                            },
                            Paths = new OpenApiPaths()
                        };

                        // Reading locally stored dependencies
                        IEnumerable<string> files = Directory.EnumerateFiles(jsonDirectory, "*.json");
                        foreach (string file in files)
                        {
                            PrettyPrint(file, "Processing Open Api doc into API client...");
                            var stream = File.OpenRead(file);
                            var doc = new OpenApiStreamReader().Read(stream, out var diagnostic);

                            // Merge paths
                            foreach (var p in doc.Paths)
                            {
                                document.Paths.TryAdd(p.Key, p.Value);
                            }

                            // Merge and normalize schemas (centralized in updater)
                            var updater = new OpenApiSchemaReferenceUpdater();
                            updater.MergeSchemasAndUpdateRefs(document, doc, key =>
                            {
                                return key.Split(".").Last();
                            });
                        }

                        var outputString = document.Serialize(OpenApiSpecVersion.OpenApi3_0, OpenApiFormat.Json);

                        // storing json schema bundle (just for verification)
                        using (StreamWriter writer = new StreamWriter(modelSharedDir + Path.DirectorySeparatorChar + JSON_BUNDLE))
                        {
                            writer.WriteLine(outputString);
                        }

                        PrettyPrint(PRETTY_STRING, "OpenApi document has been generated successfully!");

                        // C# code generation through NSwag library (NSwag.OpenApiDocument is used)
                        NSwag.OpenApiDocument nswDocument = await NSwag.OpenApiDocument.FromJsonAsync(outputString);

                        var settings = new CSharpClientGeneratorSettings
                        {
                            CSharpGeneratorSettings =
                            {
                                Namespace = NAMESPACE,
                                TypeNameGenerator = new CustomTypeNameGenerator(), // strip type names to short names
                                JsonLibrary = CSharpJsonLibrary.SystemTextJson
                            },
                            GenerateClientClasses = true,
                            GenerateDtoTypes = true,
                            GenerateOptionalParameters = true
                        };
                        var generator = new CSharpClientGenerator(nswDocument, settings);
                        var code = generator.GenerateFile();
                        using (StreamWriter writer = new StreamWriter(modelSharedDir + Path.DirectorySeparatorChar + CSHARP_MODEL))
                        {
                            writer.WriteLine(code);
                        }
                        error = false;
                        PrettyPrint(PRETTY_STRING, "C# client and base classes have been generated successfully!");
                    }
                    else
                    {
                        PrintError($"Folder {jsonDirectory} where shared model json schemas dependencies should be stored does not exist. Create it first!");
                    }
                }
                catch (HttpRequestException ex)
                {
                    PrintError("Problem with HttpRequest sent to a microservice", ex.Message);
                }
                catch (Exception ex)
                {
                    PrintError("Problem occurred", ex.Message);
                }
            }
            else
            {
                PrintError("The shared model files (Service) will not be overriden:\n" +
                            modelSharedDir + Path.DirectorySeparatorChar + CSHARP_MODEL + "\n" +
                            modelSharedDir + Path.DirectorySeparatorChar + JSON_BUNDLE);
            }
        }
        else
        {
            PrintError("Unable to locate current folder");
        }

        lock (lock_)
        {
            finished_ = true;
        }

        return error;
    }

    static HttpClient SetHttpClient(string host, string hostBasePath)
    {
        HttpClient httpClient = new()
        {
            BaseAddress = new Uri(host + hostBasePath)
        };
        httpClient.DefaultRequestHeaders.Accept.Clear();
        httpClient.DefaultRequestHeaders.Accept.Add(new System.Net.Http.Headers.MediaTypeWithQualityHeaderValue("application/json"));
        return httpClient;
    }

    /// <summary>
    /// This custom type generator is designed to format type names: getting rid of namespaces; and getting rid of '+' signs (associated with enums type names)
    /// Note that '+' signs associated with enums should have been filtered out at this stage (Startup.cs::AddSwaggerGen() service settings are tuned to achieve this at the swagger.json stage)
    /// </summary>
    public class CustomTypeNameGenerator : ITypeNameGenerator
    {
        public string Generate(JsonSchema schema, string? typeNameHint, IEnumerable<string> reservedTypeNames)
        {
            // strip namespaces & nested-class '+' to leave only the short name
            return typeNameHint?.Split('.', '+').Last() ?? "Anonymous";
        }
    }

    static void PrettyPrint(string header, string message, string exception = "")
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("> " + header).
           Append("\n\t" + message);
        if (exception != "")
            sb.Append("\n\t" + exception);
        Console.WriteLine(sb);
    }

    static void PrintError(string message, string exception = "")
    {
        Console.BackgroundColor = ConsoleColor.Red;
        StringBuilder sb = new StringBuilder();
        sb.Append('\n', 1).
           Append("\n" + message + "\n").
           Append("\t" + exception + "\n").
           Append('\n', 1);
        Console.WriteLine(sb);
    }
}