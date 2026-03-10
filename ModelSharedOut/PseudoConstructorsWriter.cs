using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Reflection;
using TplNamespace.ModelShared;
 
namespace TplNamespace.PseudoConstructorsWriter
    {
    class Writer
    {     
        private static readonly string NAMESPACE = "TplNamespace.ModelShared";    
        private static readonly string PSEUDO_CTOR = "PseudoConstructors.cs";
        private static readonly string MODELSHARED_FOLDER = "ModelSharedOut";
        private static string fullPath = ""; 
        private static string IDENTATION = "\n\t\t\t\t";
        private static string ICOLLECTION_FULL_NAME = "System.Collections.Generic.ICollection`1[";                      
        private static string? dictionaryKey;
        private static string? dictionaryValue;
        private static bool isFromNamespace = false;
        private static int CollectionStacks(PropertyInfo propertyInfo)
        {
            //Get number of list stacks
            if (propertyInfo.PropertyType.AssemblyQualifiedName != null)
                return propertyInfo.PropertyType.AssemblyQualifiedName.Split(ICOLLECTION_FULL_NAME).Length - 1;
            else
                return 0;
        }        
        private static string ReturnBaseType(Type type)
        {
            if (type.GenericTypeArguments.Length > 0)        
                return ReturnBaseType(type.GenericTypeArguments[0]);        
            else if (type.IsPrimitive || type.Name == "String")
                return type.Name.ToLower();
            else      
                return type.Name;                      
        }
        private static string ReturnFullType(Type type)
        {
            if (type.GenericTypeArguments.Length > 0)
            {
                //If it is a list, create a list stack in the type
                if (type.Name == "ICollection`1")
                    return "List<" + ReturnFullType(type.GenericTypeArguments[0]) + ">";        
                else if (type.Name == "Nullable`1")
                    return ReturnFullType(type.GenericTypeArguments[0]) + "?";
                else if (type.Name == "IDictionary`2")
                {
                    dictionaryKey = ReturnFullType(type.GenericTypeArguments[0]); 
                    dictionaryValue = ReturnFullType(type.GenericTypeArguments[1]);                     
                    return $"Dictionary<{dictionaryKey},{dictionaryValue}>";      
                }
                else
                    return ReturnFullType(type.GenericTypeArguments[0]);
            }
            else
            {
                if (type.IsPrimitive || type.Name == "String")
                {
                    return type.Name.ToLower();
                }
                else      
                    return type.Name;      
            }        
        }
        public static bool ChangeICollectionToList(DirectoryInfo directory)
        {
            bool success = false;
            try
            {                
                //Dynamic implementation seems to work better with relative paths
                string filePath = fullPath + "TplProjectNameMergedModel.cs";
                //  Change all instances of "ICollection" to "List" in the MergedModel namespace. 
                //this improves instantiation of stacked collections.
                string fileContent = File.ReadAllText(filePath);
                string iCollectionString = "ICollection";
                string listString = "List";
                string updatedMergedModel = fileContent.Replace(iCollectionString, listString);
                File.WriteAllText(filePath, updatedMergedModel);
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine("\t\x1b[1m ✓ - Update of MergedModel namespace succeeded! \x1b[0m");
                Console.ForegroundColor = ConsoleColor.White;          
                success = true;
            }    
            catch (Exception e)
            {
                Console.WriteLine("\t\x1b[1m\x1b[31m ⚠ - Update of MergedModel namespace failed.  \x1b[0m");
            }

            return success;
        }
        private static string CreateDefaultSystemValue(string propBaseName, string propTypeName)
        {
            string defaultValue = "";
            if (isFromNamespace)
            {
                //  Custom properties have default value ConstructMyClass()
                // which is a call to their respective constructor.     
                defaultValue = $"Construct{propBaseName}(),";
            }
            
            //Simple values  
            if (propBaseName.Contains("DateTimeOffset"))
            {
                // Date & time are always set to "now"
                defaultValue = "DateTimeOffset.UtcNow,";
            }                                 
            else if (propTypeName.Contains('?')) 
            {
                //  Nullables are set to null.
                //if this is not desired, this  
                //'else if' should be commented
                defaultValue = $"null, ";   
            }
            else if (propBaseName == "double")
            {
                // doubles are set to 0.0
                defaultValue = "0.0, ";  
            }
            else if (propBaseName == "string")
            {
                //Will only be used in dictionary
                defaultValue = "\"\",";
            }
            //int does not descriminate int64, int32, int16...
            else if (propBaseName == "int64" || propBaseName == "int32" || propBaseName == "int16" || propBaseName == "int8" || propBaseName == "int") 
            {
                //  Set int to 0
                defaultValue = "0, ";  
            }
            else if (propBaseName.Contains("boolean")) 
            {
                // Set booleans to "false"
                defaultValue = "false, ";  
            }
            else if (propBaseName == "guid")
            {
                // Guid are set to "new Guid()"
                defaultValue = "new Guid(),";  
            }            
            else if (propBaseName == "datetime")
            {
                // Date & time are always set to "now"
                defaultValue = "DateTime.UtcNow,";
            }
            else if (!isFromNamespace)
            {
                // default default value
                defaultValue = $"new {propBaseName}(),";   
            }
     
            return defaultValue;
        }
        public static string CreateMetaInfoConstructor()
        {
            string code =   
                "\n\t\tpublic static MetaInfo ConstructMetaInfo()" +
                "\n\t\t\t{" +                            
                "\n\t\t\t\treturn new MetaInfo " +
                "\n\t\t\t\t{" +
                "\n\t\t\t\t\tID = Guid.NewGuid()," +
                "\n\t\t\t\t\tHttpHostName = \"https://dev.digiwells.no/\"," +
                "\n\t\t\t\t\tHttpHostBasePath = \"TplProjectName/api/\"," +
                "\n\t\t\t\t\tHttpEndPoint = \"TplParentData/\"," +                                                                                                                
                "\n\t\t\t\t};" +
                "\n\t\t\t}";  


            code += "\n"+   
                "\n\t\tpublic static MetaInfo ConstructMetaInfo(Guid id)" +
                "\n\t\t\t{" +                            
                "\n\t\t\t\treturn new MetaInfo " +
                "\n\t\t\t\t{" +
                "\n\t\t\t\t\tID = id," +
                "\n\t\t\t\t\tHttpHostName = \"https://dev.digiwells.no/\"," +
                "\n\t\t\t\t\tHttpHostBasePath = \"TplProjectName/api/\"," +
                "\n\t\t\t\t\tHttpEndPoint = \"TplParentData/\"," +                                                                                                                
                "\n\t\t\t\t};" +
                "\n\t\t\t}";                                       
                            
            return code ;
        }
        public static bool CreatePseudoConstructors()
        {     

            try
            {  
                DirectoryInfo directory = new DirectoryInfo(Directory.GetCurrentDirectory());
                //Loop until solution        
                while (directory != null && !directory.GetFiles("*.sln").Any())
                {
                    directory = directory.Parent;
                }
                fullPath = directory.ToString() + Path.DirectorySeparatorChar + MODELSHARED_FOLDER + Path.DirectorySeparatorChar;                          
                //Get current file
                //Change merged model ICollection instances to List
                bool formatedMergedModel = ChangeICollectionToList(directory);
                //Get all classes from current assembly
                var classes = from t in Assembly.GetExecutingAssembly().GetTypes()
                              where t.IsClass && t.Namespace == NAMESPACE
                              select t;        
                classes = classes.ToList();
                //Start main code header by creating the class and namespace
                string pseudoConstructorsText = "namespace " + NAMESPACE + "\n{\n\tpublic class PseudoConstructors\n\t{";
                pseudoConstructorsText += CreateMetaInfoConstructor();
                foreach (var q in classes)
                {                
                    //  Classes with exceptions in here are either not relevant
                    //or are residual from the methods generated from the schema.
                    if (q.Name != "Client" &&
                        q.Name != "PseudoConstructors" &&
                        q.Name[0] != '<' &&
                        q.Name != "ApiException" &&
                        q.Name != "ApiException`1" &&
                        q.Name != "MetaInfo" &&
                        !q.Name.Contains("Light"))
                    {
                        //Create constructor method for given object
                        pseudoConstructorsText += $"\n\t\tpublic static {q.Name} Construct{q.Name}()" + "\n\t\t{";
                        pseudoConstructorsText += "\n\t\t\treturn new " + q.Name + "\n\t\t\t{";
                        string propertiesText = "";
                        foreach (var p in q.GetProperties())
                        {
                            if (p.Name != "AdditionalProperties")
                            {                                                                                          
                                //Get the namespce of the properties
                                isFromNamespace = (p.ToString()!.Contains(NAMESPACE));
                                //Check if it is an enum
                                bool isEnum = (p.PropertyType.BaseType != null) ? (p.PropertyType.BaseType.ToString() == "System.Enum") : false;
                                // Extract property name from the class.
                                string propertyName = IDENTATION + p.Name + " = ";
                                //  defaultValueString containts the lowest level of the default value. If p is of type
                                // List<double>, then defaultValueString = "new double()". If the type of p is an arbitrary
                                // type "MyType", then defaultValueString = "ConstructMyType()".   
                                string defaultValueString = "";
                                //Get "full" type (e.g.: "List<double?>")
                                string propTypeName = ReturnFullType(p.PropertyType);
                                //Get base type (e.g.: "double")                            
                                string propBaseName = ReturnBaseType(p.PropertyType);                           
                                //Number of stacks of the list (e.g.: listStacks = 2 if p is a List<List<double>>)
                                int listStacks = CollectionStacks(p);                                                                                                    
                                //Handle ENUMs
                                if (isEnum)
                                {
                                    //The value is set to the first enum option
                                    defaultValueString = $"({propTypeName})0,";
                                }
                                else
                                {                                                                             
                                    if (p.PropertyType.ToString() == "System.String")
                                    {
                                        //If it is a reference type, it is assumed to be a 
                                        //string and a default name is used.
                                        defaultValueString = "\"Default " + p.Name + "\",";
                                    }
                                    else if (propTypeName.Contains("Dictionary"))
                                    {

                                        string keyDefaultValue = CreateDefaultSystemValue((string) dictionaryKey, (string) dictionaryKey).Replace(",","");
                                        string valueDefaultValue = CreateDefaultSystemValue((string) dictionaryValue, (string) dictionaryValue).Replace(",","");
                                        
                                        defaultValueString = $"new {propTypeName}" +
                                        "\n\t\t\t\t\t{"
                                        +"\n\t\t\t\t\t\t{ " + $"{keyDefaultValue}, {valueDefaultValue}" + " }"
                                        +"\n\t\t\t\t\t},"
                                        ;   
                                    }
                                    else
                                    {
                                        defaultValueString = CreateDefaultSystemValue(propBaseName, propTypeName);
                                    }                                 
                                }//Close 'IF isEnum{} ELSE{}' section 
                                //Special treatment in case of collections/lists
                                if (listStacks > 0)
                                {   
                                    //Create identation for a list constructors, e.g.:
                                    // List<List<var>> myVar = new List<List<var>>
                                    //      {
                                    //          new List<var>
                                    //          {
                                    //              new var(),
                                    //          }
                                    //      }
                                    string identationList = "";
                                    for (int j = 0; j < listStacks; j++)
                                    {
                                        identationList = identationList + "\t";
                                    }
                                    identationList = IDENTATION + identationList;
                                    //Radical is the "left side" of the constructor 
                                    string listRadical = $"List<{propBaseName}>";  
                                    //listInnerInitializer contains the the code that is contained within the "{ }", e.g.: { new List<double> { new double()} }                      
                                    string listInnerInitializer = listRadical + identationList + "{" + identationList + "\t" + defaultValueString +  identationList + "}";
                                    for (int i = listStacks - 1; i > 0; i--)
                                    {
                                        //Handle identation
                                        identationList = "";
                                        for (int j = 0; j < i; j++)
                                        {
                                            identationList = identationList + "\t";
                                        }
                                        identationList = IDENTATION + identationList;
                                        listRadical = $"List<{listRadical}>";   
                                        listInnerInitializer = listRadical + identationList + "{" + identationList + "\tnew " + listInnerInitializer + identationList + "}";
                                    }                                                                                                     
                                    propertiesText += propertyName + "new " + listInnerInitializer + ",";
                                }
                                //Else, create an empty instance of the property
                                else
                                {
                                    propertiesText += propertyName + defaultValueString;
                                }//Close 'IF isList{} ELSE{}' logic 
                            }
                        }//Close "is not AdditionalProperty"
                        //Add property instance to the constructor
                        pseudoConstructorsText += propertiesText;
                        //Add } and identation
                        pseudoConstructorsText += "\n\t\t\t};";
                        pseudoConstructorsText += "\n\t\t}";
                    }
                }// Close IF IsRelevantClass
                //Close pseudo constructor class & namespace
                pseudoConstructorsText += "\n\t}\n}";
                //Create path to save
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine("\t\x1b[1m ✓ - PseudoConstuctors.cs file generated successfully! \x1b[0m");
                Console.ForegroundColor = ConsoleColor.White; // Reset color to default
                //Dynamic implementation seems to work better with relative paths
                File.WriteAllText(fullPath + PSEUDO_CTOR, pseudoConstructorsText);                
                return true;
            }//Close TRY
            catch (Exception e)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("\t\x1b[1m ⚠ - PseudoConstuctors.cs generation failed!  \x1b[0m" + e);            
                Console.ForegroundColor = ConsoleColor.White; // Reset color to default
                return false;
            }//Close CATCH
        }
    }
}