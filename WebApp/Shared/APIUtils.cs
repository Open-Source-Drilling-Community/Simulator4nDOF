using OSDC.UnitConversion.DrillingRazorMudComponents;

public static class APIUtils
{
    // API parameters
    public static readonly string HostNameSimulator4nDOF = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.Simulator4nDOFHostURL!;
    public static readonly string HostBasePathSimulator4nDOF = "Simulator4nDOF/api/";
    public static readonly HttpClient HttpClientSimulator4nDOF = APIUtils.SetHttpClient(HostNameSimulator4nDOF, HostBasePathSimulator4nDOF);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientSimulator4nDOF = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientSimulator4nDOF.BaseAddress!.ToString(), APIUtils.HttpClientSimulator4nDOF);

    public static readonly string HostNameWellBore = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellBoreHostURL!;
    public static readonly string HostBasePathWellBore = "WellBore/api/";
    public static readonly HttpClient HttpClientWellBore = APIUtils.SetHttpClient(HostNameWellBore, HostBasePathWellBore);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientWellBore = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientWellBore.BaseAddress!.ToString(), APIUtils.HttpClientWellBore);

    public static readonly string HostNameWell = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellHostURL!;
    public static readonly string HostBasePathWell = "Well/api/";
    public static readonly HttpClient HttpClientWell = APIUtils.SetHttpClient(HostNameWell, HostBasePathWell);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientWell = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientWell.BaseAddress!.ToString(), APIUtils.HttpClientWell);

    public static readonly string HostNameCluster = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.ClusterHostURL!;
    public static readonly string HostBasePathCluster = "Cluster/api/";
    public static readonly HttpClient HttpClientCluster = APIUtils.SetHttpClient(HostNameCluster, HostBasePathCluster);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientCluster = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientCluster.BaseAddress!.ToString(), APIUtils.HttpClientCluster);

    public static readonly string HostNameField = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.FieldHostURL!;
    public static readonly string HostBasePathField = "Field/api/";
    public static readonly HttpClient HttpClientField = APIUtils.SetHttpClient(HostNameField, HostBasePathField);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientField = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientField.BaseAddress!.ToString(), APIUtils.HttpClientField);

    public static readonly string HostNameDrillString = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.DrillStringHostURL!;
    public static readonly string HostBasePathDrillString = "DrillString/api/";
    public static readonly HttpClient HttpClientDrillString = APIUtils.SetHttpClient(HostNameDrillString, HostBasePathDrillString);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientDrillString = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientDrillString.BaseAddress!.ToString(), APIUtils.HttpClientDrillString);

    public static readonly string HostNameTrajectory = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.TrajectoryHostURL!;
    public static readonly string HostBasePathTrajectory = "Trajectory/api/";
    public static readonly HttpClient HttpClientTrajectory = APIUtils.SetHttpClient(HostNameTrajectory, HostBasePathTrajectory);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientTrajectory = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientTrajectory.BaseAddress!.ToString(), APIUtils.HttpClientTrajectory);

    public static readonly string HostNameDrillingFluid = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.DrillingFluidHostURL!;
    public static readonly string HostBasePathDrillingFluid = "DrillingFluid/api/";
    public static readonly HttpClient HttpClientDrillingFluid = APIUtils.SetHttpClient(HostNameDrillString, HostBasePathDrillString);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientDrillingFluid = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientDrillingFluid.BaseAddress!.ToString(), APIUtils.HttpClientDrillingFluid);

    public static readonly string HostNameWellBoreArchitecture = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.WellBoreArchitectureHostURL!;
    public static readonly string HostBasePathWellBoreArchitecture = "Trajectory/api/";
    public static readonly HttpClient HttpClientWellBoreArchitecture = APIUtils.SetHttpClient(HostNameWellBoreArchitecture, HostBasePathWellBoreArchitecture);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientWellBoreArchitecture = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientWellBoreArchitecture.BaseAddress!.ToString(), APIUtils.HttpClientWellBoreArchitecture);

    public static readonly string HostNameUnitConversion = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.UnitConversionHostURL!;
    public static readonly string HostBasePathUnitConversion = "UnitConversion/api/";

    // API utility methods
    public static HttpClient SetHttpClient(string host, string microServiceUri)
    {
        var handler = new HttpClientHandler();
        handler.ServerCertificateCustomValidationCallback = (message, cert, chain, errors) => { return true; }; // temporary workaround for testing purposes: bypass certificate validation (not recommended for production environments due to security risks)
        HttpClient httpClient = new(handler)
        {
            BaseAddress = new Uri(host + microServiceUri)
        };
        httpClient.DefaultRequestHeaders.Accept.Clear();
        httpClient.DefaultRequestHeaders.Accept.Add(new System.Net.Http.Headers.MediaTypeWithQualityHeaderValue("application/json"));
        return httpClient;
    }
}

public class GroundMudLineDepthReferenceSource : IGroundMudLineDepthReferenceSource
{
    public double? GroundMudLineDepthReference { get; set; }
}

public class RotaryTableDepthReferenceSource : IRotaryTableDepthReferenceSource
{
    public double? RotaryTableDepthReference { get; set; }
}

public class SeaWaterLevelDepthReferenceSource : ISeaWaterLevelDepthReferenceSource
{
    public double? SeaWaterLevelDepthReference { get; set; }
}

public class WellHeadDepthReferenceSource : IWellHeadDepthReferenceSource
{
    public double? WellHeadDepthReference { get; set; }
}