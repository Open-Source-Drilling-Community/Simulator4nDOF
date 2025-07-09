using System;
using System.Net.Http;

public static class APIUtils
{
    // API parameters
    public static readonly string HostNameDrillString = NORCE.Drilling.Simulator4nDOF.Service.ServiceConfiguration.DrillStringHostURL!;
    public static readonly string HostBasePathDrillString = "DrillString/api/";
    public static readonly HttpClient HttpClientDrillString = APIUtils.SetHttpClient(HostNameDrillString, HostBasePathDrillString);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientDrillString = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientDrillString.BaseAddress!.ToString(), APIUtils.HttpClientDrillString);

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