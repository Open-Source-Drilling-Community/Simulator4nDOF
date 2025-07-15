using NORCE.Drilling.Simulator4nDOF.ModelShared;
using System;
using System.Net.Http;

public static class APIUtils
{
    // API parameters
    public static readonly string HostNameDrillString = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.DrillStringHostURL!;
    public static readonly string HostBasePathDrillString = "DrillString/api/";
    public static readonly HttpClient HttpClientDrillString = APIUtils.SetHttpClient(HostNameDrillString, HostBasePathDrillString);
    public static readonly Client ClientDrillString = new Client(APIUtils.HttpClientDrillString.BaseAddress!.ToString(), APIUtils.HttpClientDrillString);

    public static readonly string HostNameDrillStringOpenLab = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.DrillStringOpenLabHostURL!;
    public static readonly string HostBasePathDrillStringOpenLab = "DrillStringOpenLab/api/";
    public static readonly HttpClient HttpClientDrillStringOpenLab = APIUtils.SetHttpClient(HostNameDrillStringOpenLab, HostBasePathDrillStringOpenLab);
    public static readonly Client ClientDrillStringOpenLab = new Client(APIUtils.HttpClientDrillStringOpenLab.BaseAddress!.ToString(), APIUtils.HttpClientDrillStringOpenLab);

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