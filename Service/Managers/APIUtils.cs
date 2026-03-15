using NORCE.Drilling.Simulator4nDOF.ModelShared;
using System;
using System.Net.Http;

public static class APIUtils
{
    // API parameters
    public static readonly string HostNameCluster = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.ClusterHostURL!;
    public static readonly string HostBasePathCluster = "Cluster/api/";
    public static readonly HttpClient HttpClientCluster = APIUtils.SetHttpClient(HostNameCluster, HostBasePathCluster);
    public static readonly Client ClientCluster = new Client(APIUtils.HttpClientCluster.BaseAddress!.ToString(), APIUtils.HttpClientCluster);

    public static readonly string HostNameWellBore = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.WellBoreHostURL!;
    public static readonly string HostBasePathWellBore = "WellBore/api/";
    public static readonly HttpClient HttpClientWellBore = APIUtils.SetHttpClient(HostNameWellBore, HostBasePathWellBore);
    public static readonly Client ClientWellBore = new Client(APIUtils.HttpClientWellBore.BaseAddress!.ToString(), APIUtils.HttpClientWellBore);

    public static readonly string HostNameWell = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.WellHostURL!;
    public static readonly string HostBasePathWell = "Well/api/";
    public static readonly HttpClient HttpClientWell = APIUtils.SetHttpClient(HostNameWell, HostBasePathWell);
    public static readonly Client ClientWell = new Client(APIUtils.HttpClientWell.BaseAddress!.ToString(), APIUtils.HttpClientWell);

    public static readonly string HostNameDrillString = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.DrillStringHostURL!;
    public static readonly string HostBasePathDrillString = "DrillString/api/";
    public static readonly HttpClient HttpClientDrillString = APIUtils.SetHttpClient(HostNameDrillString, HostBasePathDrillString);
    public static readonly Client ClientDrillString = new Client(APIUtils.HttpClientDrillString.BaseAddress!.ToString(), APIUtils.HttpClientDrillString);

    public static readonly string HostNameTrajectory = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.TrajectoryHostURL!;
    public static readonly string HostBasePathTrajectory = "Trajectory/api/";
    public static readonly HttpClient HttpClientTrajectory = APIUtils.SetHttpClient(HostNameTrajectory, HostBasePathTrajectory);
    public static readonly Client ClientTrajectory = new Client(APIUtils.HttpClientTrajectory.BaseAddress!.ToString(), APIUtils.HttpClientTrajectory);

    public static readonly string HostNameDrillingFluid = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.DrillingFluidHostURL!;
    public static readonly string HostBasePathDrillingFluid = "DrillingFluid/api/";
    public static readonly HttpClient HttpClientDrillingFluid = APIUtils.SetHttpClient(HostNameDrillingFluid, HostBasePathDrillingFluid);
    public static readonly Client ClientDrillingFluid = new Client(APIUtils.HttpClientDrillingFluid.BaseAddress!.ToString(), APIUtils.HttpClientDrillingFluid);

    public static readonly string HostNameWellBoreArchitecture = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.WellBoreArchitectureHostURL!;
    public static readonly string HostBasePathWellBoreArchitecture = "WellBoreArchitecture/api/";
    public static readonly HttpClient HttpClientWellBoreArchitecture = APIUtils.SetHttpClient(HostNameWellBoreArchitecture, HostBasePathWellBoreArchitecture);
    public static readonly Client ClientWellBoreArchitecture = new Client(APIUtils.HttpClientWellBoreArchitecture.BaseAddress!.ToString(), APIUtils.HttpClientWellBoreArchitecture);

    public static readonly string HostNameRig = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.RigHostURL!;
    public static readonly string HostBasePathRig = "Rig/api/";
    public static readonly HttpClient HttpClientRig = APIUtils.SetHttpClient(HostNameRig, HostBasePathRig);
    public static readonly Client ClientRig = new Client(APIUtils.HttpClientRig.BaseAddress!.ToString(), APIUtils.HttpClientRig);

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
