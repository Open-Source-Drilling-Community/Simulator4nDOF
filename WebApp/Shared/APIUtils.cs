public static class APIUtils
{
    // API parameters
    public static readonly string HostNameSimulator4nDOF = NORCE.Drilling.Simulator4nDOF.WebApp.Configuration.Simulator4nDOFHostURL!;
    public static readonly string HostBasePathSimulator4nDOF = "Simulator4nDOF/api/";
    public static readonly HttpClient HttpClientSimulator4nDOF = APIUtils.SetHttpClient(HostNameSimulator4nDOF, HostBasePathSimulator4nDOF);
    public static readonly NORCE.Drilling.Simulator4nDOF.ModelShared.Client ClientSimulator4nDOF = new NORCE.Drilling.Simulator4nDOF.ModelShared.Client(APIUtils.HttpClientSimulator4nDOF.BaseAddress!.ToString(), APIUtils.HttpClientSimulator4nDOF);

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