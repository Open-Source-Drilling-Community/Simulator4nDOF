using System.Net.Http.Headers;
using NORCE.Drilling.Simulator4nDOF.ModelShared;

namespace ServiceTest
{
    public class Tests
    {
        // testing outside Visual Studio requires using http port (https faces authentication issues both in console and on github)
        private static string host = "http://localhost:8080/";
        //private static string host = "https://localhost:5001/";
        //private static string host = "https://localhost:44368/";
        //private static string host = "http://localhost:54949/";
        private static HttpClient httpClient;
        private static Client nSwagClient;

        [OneTimeSetUp]
        public void OneTimeSetUp()
        {
            var handler = new HttpClientHandler();
            handler.ServerCertificateCustomValidationCallback = (message, cert, chain, errors) => { return true; }; // temporary workaround for testing purposes: bypass certificate validation (not recommended for production environments due to security risks)
            httpClient = new HttpClient(handler);
            httpClient.BaseAddress = new Uri(host + "Simulator4nDOF/api/");
            httpClient.DefaultRequestHeaders.Accept.Clear();
            httpClient.DefaultRequestHeaders.Accept.Add(new MediaTypeWithQualityHeaderValue("application/json"));
            nSwagClient = new Client(httpClient.BaseAddress.ToString(), httpClient);
        }

        //[Test]
        //public async Task Test_Simulation_GET()
        //{
        //    #region post a Simulation
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    MetaInfo metaInfoToBeRemoved1 = new() { ID = Guid.NewGuid() };
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved1 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved1,
        //        Name = "My test ToBeRemoved name 1",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1
        //    };
        //    MetaInfo metaInfoToBeRemoved2 = new() { ID = Guid.NewGuid() };
        //    DerivedData2 derivedData2 = new() { DerivedData2Param = 10 };
        //    ToBeRemoved toBeRemoved2 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved2,
        //        Name = "My test ToBeRemoved name 2",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 11.0 } },
        //        DerivedData2 = derivedData2,
        //        Type = ToBeRemovedType.DerivedData2
        //    };
        //    Simulation simulation = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test Simulation",
        //        ToBeRemovedList = [toBeRemoved1, toBeRemoved2]
        //    };

        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST given Simulation\n" + ex.Message);
        //    }
        //    #endregion

        //    #region GetAllSimulationId
        //    List<Guid> idList = [];
        //    try
        //    {
        //        idList = (List<Guid>)await nSwagClient.GetAllSimulationIdAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET all Simulation ids\n" + ex.Message);
        //    }
        //    Assert.That(idList, Is.Not.Null);
        //    Assert.That(idList, Does.Contain(guid));
        //    #endregion

        //    #region GetAllSimulationMetaInfo
        //    List<MetaInfo> metaInfoList = [];
        //    try
        //    {
        //        metaInfoList = (List<MetaInfo>)await nSwagClient.GetAllSimulationMetaInfoAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET all Simulation metainfos\n" + ex.Message);
        //    }
        //    Assert.That(metaInfoList, Is.Not.Null);
        //    IEnumerable<MetaInfo> metaInfoList2 =
        //        from elt in metaInfoList
        //        where elt.ID == guid
        //        select elt;
        //    Assert.That(metaInfoList2, Is.Not.Null);
        //    Assert.That(metaInfoList2, Is.Not.Empty);
        //    #endregion

        //    #region GetAllSimulationById
        //    Simulation? simulation2 = null;
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Not.Null);
        //    Assert.That(simulation2.Name, Is.EqualTo(simulation.Name));
        //    #endregion

        //    #region GetAllSimulationLight
        //    List<SimulationLight> simulationLightList = [];
        //    try
        //    {
        //        simulationLightList = (List<SimulationLight>)await nSwagClient.GetAllSimulationLightAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the list of SimulationLight\n" + ex.Message);
        //    }
        //    Assert.That(simulationLightList, Is.Not.Null);
        //    Assert.That(simulationLightList, Is.Not.Empty);
        //    IEnumerable<SimulationLight> simulationLightList2 =
        //        from elt in simulationLightList
        //        where elt.Name == simulation.Name
        //        select elt;
        //    Assert.That(simulationLightList2, Is.Not.Null);
        //    Assert.That(simulationLightList2, Is.Not.Empty);
        //    #endregion

        //    #region GetAllSimulation
        //    List<Simulation> simulationList = new();
        //    try
        //    {
        //        simulationList = (List<Simulation>)await nSwagClient.GetAllSimulationAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the list of Simulation\n" + ex.Message);
        //    }
        //    Assert.That(simulationList, Is.Not.Null);
        //    IEnumerable<Simulation> simulationList2 =
        //        from elt in simulationList
        //        where elt.Name == simulation.Name
        //        select elt;
        //    Assert.That(simulationList2, Is.Not.Null);
        //    Assert.That(simulationList2, Is.Not.Empty);
        //    #endregion

        //    #region finally delete the new ID
        //    simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE Simulation of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_Simulation_POST()
        //{
        //    #region trying to post an empty guid
        //    Guid guid = Guid.Empty;
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    MetaInfo metaInfoToBeRemoved1 = new() { ID = Guid.NewGuid() };
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved1 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved1,
        //        Name = "My test ToBeRemoved name 1",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1
        //    };
        //    MetaInfo metaInfoToBeRemoved2 = new() { ID = Guid.NewGuid() };
        //    DerivedData2 derivedData2 = new() { DerivedData2Param = 10 };
        //    ToBeRemoved toBeRemoved2 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved2,
        //        Name = "My test ToBeRemoved name 2",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 11.0 } },
        //        DerivedData2 = derivedData2,
        //        Type = ToBeRemovedType.DerivedData2
        //    };
        //    Simulation simulation = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test Simulation",
        //        Description = "My test Simulation",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedList = [toBeRemoved1, toBeRemoved2]
        //    };
        //    Simulation? simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(400));
        //        TestContext.WriteLine("Impossible to POST Simulation with empty Guid\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(Guid.Empty);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(400));
        //        TestContext.WriteLine("Impossible to GET Simulation identified by an empty Guid\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Null);
        //    #endregion

        //    #region post some corrupted data
        //    // post data with missing input that fails the calculation process
        //    #endregion

        //    #region posting a new ID in a valid state
        //    guid = Guid.NewGuid();
        //    metaInfo = new() { ID = guid };
        //    simulation.MetaInfo = metaInfo;
        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST Simulation although it is in a valid state\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo.ID, Is.EqualTo(guid));
        //    Assert.That(simulation2.Name, Is.EqualTo(simulation.Name));
        //    #endregion

        //    #region trying to repost the same ID
        //    bool conflict = false;
        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        conflict = true;
        //        Assert.That(ex.StatusCode, Is.EqualTo(409));
        //        TestContext.WriteLine("Impossible to POST existing Simulation\n" + ex.Message);
        //    }
        //    Assert.That(conflict, Is.True);
        //    #endregion

        //    #region finally delete the new ID
        //    simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE Simulation of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_Simulation_PUT()
        //{
        //    #region posting a new ID
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    MetaInfo metaInfoToBeRemoved1 = new() { ID = Guid.NewGuid() };
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved1 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved1,
        //        Name = "My test ToBeRemoved name 1",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1
        //    };
        //    MetaInfo metaInfoToBeRemoved2 = new() { ID = Guid.NewGuid() };
        //    DerivedData2 derivedData2 = new() { DerivedData2Param = 10 };
        //    ToBeRemoved toBeRemoved2 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved2,
        //        Name = "My test ToBeRemoved name 2",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 11.0 } },
        //        DerivedData2 = derivedData2,
        //        Type = ToBeRemovedType.DerivedData2
        //    };
        //    Simulation simulation = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test Simulation",
        //        Description = "My test Simulation",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedList = [toBeRemoved1, toBeRemoved2]
        //    };
        //    Simulation? simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST Simulation\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo.ID, Is.EqualTo(guid));
        //    Assert.That(simulation2.Name, Is.EqualTo(simulation.Name));
        //    #endregion

        //    #region updating the new Id
        //    simulation.Name = "My test Simulation with modified name";
        //    simulation.LastModificationDate = DateTimeOffset.UtcNow;
        //    try
        //    {
        //        await nSwagClient.PutSimulationByIdAsync(simulation.MetaInfo.ID, simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to PUT Simulation of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the updated Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo.ID, Is.EqualTo(simulation.MetaInfo.ID));
        //    Assert.That(simulation2.Name, Is.EqualTo(simulation.Name));
        //    #endregion

        //    #region finally delete the new ID
        //    simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE Simulation of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_Simulation_DELETE()
        //{
        //    #region posting a new ID
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    MetaInfo metaInfoToBeRemoved1 = new() { ID = Guid.NewGuid() };
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved1 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved1,
        //        Name = "My test ToBeRemoved name 1",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1
        //    };
        //    MetaInfo metaInfoToBeRemoved2 = new() { ID = Guid.NewGuid() };
        //    DerivedData2 derivedData2 = new() { DerivedData2Param = 10 };
        //    ToBeRemoved toBeRemoved2 = new()
        //    {
        //        MetaInfo = metaInfoToBeRemoved2,
        //        Name = "My test ToBeRemoved name 2",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 11.0 } },
        //        DerivedData2 = derivedData2,
        //        Type = ToBeRemovedType.DerivedData2
        //    };
        //    Simulation simulation = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test Simulation",
        //        ToBeRemovedList = [toBeRemoved1, toBeRemoved2]
        //    };
        //    Simulation? simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.PostSimulationAsync(simulation);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST Simulation\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo, Is.Not.Null);
        //    Assert.That(simulation2.MetaInfo.ID, Is.EqualTo(simulation.MetaInfo.ID));
        //    Assert.That(simulation2.Name, Is.EqualTo(simulation.Name));
        //    #endregion

        //    #region finally delete the new ID
        //    simulation2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE Simulation of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        simulation2 = await nSwagClient.GetSimulationByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted Simulation of given Id\n" + ex.Message);
        //    }
        //    Assert.That(simulation2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_ToBeRemoved_GET()
        //{
        //    #region post a ToBeRemoved
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test ToBeRemoved name",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1,

        //    };
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST given ToBeRemoved\n" + ex.Message);
        //    }
        //    #endregion

        //    #region GetAllToBeRemovedId
        //    List<Guid> idList = [];
        //    try
        //    {
        //        idList = (List<Guid>)await nSwagClient.GetAllToBeRemovedIdAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET all ToBeRemoved ids\n" + ex.Message);
        //    }
        //    Assert.That(idList, Is.Not.Null);
        //    Assert.That(idList, Does.Contain(guid));
        //    #endregion

        //    #region GetAllToBeRemovedMetaInfo
        //    List<MetaInfo> metaInfoList = [];
        //    try
        //    {
        //        metaInfoList = (List<MetaInfo>)await nSwagClient.GetAllToBeRemovedMetaInfoAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET all ToBeRemoved metainfos\n" + ex.Message);
        //    }
        //    Assert.That(metaInfoList, Is.Not.Null);
        //    IEnumerable<MetaInfo> metaInfoList2 =
        //        from elt in metaInfoList
        //        where elt.ID == guid
        //        select elt;
        //    Assert.That(metaInfoList2, Is.Not.Null);
        //    Assert.That(metaInfoList2, Is.Not.Empty);
        //    #endregion

        //    #region GetAllToBeRemovedById
        //    ToBeRemoved? toBeRemoved2 = null;
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo.ID, Is.EqualTo(guid));
        //    Assert.That(toBeRemoved2.Name, Is.EqualTo(toBeRemoved.Name));
        //    #endregion

        //    #region GetAllToBeRemoved
        //    List<ToBeRemoved> toBeRemovedList = [];
        //    try
        //    {
        //        toBeRemovedList = (List<ToBeRemoved>)await nSwagClient.GetAllToBeRemovedAsync();
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the list of ToBeRemoved\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemovedList, Is.Not.Null);
        //    IEnumerable<ToBeRemoved> toBeRemovedList2 =
        //        from elt in toBeRemovedList
        //        where elt.Name == toBeRemoved.Name
        //        select elt;
        //    Assert.That(toBeRemovedList2, Is.Not.Null);
        //    Assert.That(toBeRemovedList2, Is.Not.Empty);
        //    #endregion

        //    #region finally delete the new ID
        //    toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_ToBeRemoved_POST()
        //{
        //    #region trying to post an empty guid
        //    Guid guid = Guid.Empty;
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test ToBeRemoved name",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1,

        //    };
        //    ToBeRemoved? toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(400));
        //        TestContext.WriteLine("Impossible to POST ToBeRemoved with empty Guid\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(Guid.Empty);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(400));
        //        TestContext.WriteLine("Impossible to GET ToBeRemoved identified by an empty Guid\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Null);
        //    #endregion

        //    #region posting a new ID in a valid state
        //    guid = Guid.NewGuid();
        //    metaInfo = new() { ID = guid };
        //    toBeRemoved.MetaInfo = metaInfo;
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST ToBeRemoved although it is in a valid state\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo.ID, Is.EqualTo(guid));
        //    Assert.That(toBeRemoved2.Name, Is.EqualTo(toBeRemoved.Name));
        //    #endregion

        //    #region trying to repost the same ID
        //    bool conflict = false;
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        conflict = true;
        //        Assert.That(ex.StatusCode, Is.EqualTo(409));
        //        TestContext.WriteLine("Impossible to POST existing ToBeRemoved\n" + ex.Message);
        //    }
        //    Assert.That(conflict, Is.True);
        //    #endregion

        //    #region finally delete the new ID
        //    toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_ToBeRemoved_PUT()
        //{
        //    #region posting a new ID
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test ToBeRemoved name",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1,

        //    };
        //    ToBeRemoved? toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST ToBeRemoved\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(toBeRemoved.MetaInfo.ID);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo.ID, Is.EqualTo(toBeRemoved.MetaInfo.ID));
        //    Assert.That(toBeRemoved2.Name, Is.EqualTo(toBeRemoved.Name));
        //    #endregion

        //    #region updating the new Id
        //    toBeRemoved.Name = "My test ToBeRemoved with modified name";
        //    toBeRemoved.LastModificationDate = DateTimeOffset.UtcNow;
        //    try
        //    {
        //        await nSwagClient.PutToBeRemovedByIdAsync(toBeRemoved.MetaInfo.ID, toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to PUT ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(toBeRemoved.MetaInfo.ID);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the updated ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo.ID, Is.EqualTo(toBeRemoved.MetaInfo.ID));
        //    Assert.That(toBeRemoved2.Name, Is.EqualTo(toBeRemoved.Name));
        //    #endregion

        //    #region finally delete the new ID
        //    toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(toBeRemoved.MetaInfo.ID);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Null);
        //    #endregion
        //}

        //[Test]
        //public async Task Test_ToBeRemoved_DELETE()
        //{
        //    #region posting a new ID
        //    Guid guid = Guid.NewGuid();
        //    MetaInfo metaInfo = new() { ID = guid };
        //    DateTimeOffset creationDate = DateTimeOffset.UtcNow;
        //    ScalarDrillingProperty derivedData1Param = new() { DiracDistributionValue = new DiracDistribution() { Value = 1.0 } };
        //    DerivedData1 derivedData1 = new() { DerivedData1Param = derivedData1Param };
        //    ToBeRemoved toBeRemoved = new()
        //    {
        //        MetaInfo = metaInfo,
        //        Name = "My test ToBeRemoved name",
        //        Description = "My test ToBeRemoved",
        //        CreationDate = creationDate,
        //        LastModificationDate = creationDate,
        //        ToBeRemovedParam = new ScalarDrillingProperty() { DiracDistributionValue = new DiracDistribution() { Value = 2.0 } },
        //        DerivedData1 = derivedData1,
        //        Type = ToBeRemovedType.DerivedData1,

        //    };
        //    ToBeRemoved? toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.PostToBeRemovedAsync(toBeRemoved);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to POST ToBeRemoved\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to GET the ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo, Is.Not.Null);
        //    Assert.That(toBeRemoved2.MetaInfo.ID, Is.EqualTo(guid));
        //    Assert.That(toBeRemoved2.Name, Is.EqualTo(toBeRemoved.Name));
        //    #endregion

        //    #region finally delete the new ID
        //    toBeRemoved2 = null;
        //    try
        //    {
        //        await nSwagClient.DeleteToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        TestContext.WriteLine("Impossible to DELETE ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    try
        //    {
        //        toBeRemoved2 = await nSwagClient.GetToBeRemovedByIdAsync(guid);
        //    }
        //    catch (ApiException ex)
        //    {
        //        Assert.That(ex.StatusCode, Is.EqualTo(404));
        //        TestContext.WriteLine("Impossible to GET deleted ToBeRemoved of given Id\n" + ex.Message);
        //    }
        //    Assert.That(toBeRemoved2, Is.Null);
        //    #endregion
        //}
        ////#endif

        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
            httpClient?.Dispose();
        }
    }
}