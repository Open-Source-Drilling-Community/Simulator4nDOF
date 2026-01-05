using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.Data.Sqlite;
using Microsoft.Extensions.Logging;
using Model;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using OSDC.DotnetLibraries.General.DataManagement;
using OSDC.DotnetLibraries.General.Math;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Net.NetworkInformation;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using MetaInfo = OSDC.DotnetLibraries.General.DataManagement.MetaInfo;

namespace NORCE.Drilling.Simulator4nDOF.Service.Managers
{
    /// <summary>
    /// A manager for Simulation. The manager implements the singleton pattern as defined by 
    /// Gamma, Erich, et al. "Design patterns: Abstraction and reuse of object-oriented design." 
    /// European Conference on Object-Oriented Programming. Springer, Berlin, Heidelberg, 1993.
    /// </summary>
    public class SimulationManager
    {
        private static int _maxCount = 250;
        private static SimulationManager? _instance = null;
        private readonly ILogger<SimulationManager> _logger;
        private readonly SqlConnectionManager _connectionManager;
        private static readonly SemaphoreSlim _simulationSemaphore = new SemaphoreSlim(2); // limit to 1 concurrent simulations


        public static ConcurrentDictionary<Guid, Simulation> RunningSimulations = new();

        private SimulationManager(ILogger<SimulationManager> logger, SqlConnectionManager connectionManager)
        {
            _logger = logger;
            _connectionManager = connectionManager;
        }

        public static SimulationManager GetInstance(ILogger<SimulationManager> logger, SqlConnectionManager connectionManager)
        {
            _instance ??= new SimulationManager(logger, connectionManager);
            return _instance;
        }

        public int Count
        {
            get
            {
                int count = 0;
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    var command = connection.CreateCommand();
                    command.CommandText = "SELECT COUNT(*) FROM SimulationTable";
                    try
                    {
                        using SqliteDataReader reader = command.ExecuteReader();
                        if (reader.Read())
                        {
                            count = (int)reader.GetInt64(0);
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to count records in the SimulationTable");
                    }
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
                return count;
            }
        }

        public bool Clear()
        {
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                bool success = false;

                using var transaction = connection.BeginTransaction();
                try
                {
                    //empty SimulationTable
                    var command = connection.CreateCommand();
                    command.CommandText = "DELETE FROM SimulationTable";
                    command.ExecuteNonQuery();

                    transaction.Commit();
                    success = true;
                }
                catch (SqliteException ex)
                {
                    transaction.Rollback();
                    _logger.LogError(ex, "Impossible to clear the SimulationTable");
                }

                return success;
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
                return false;
            }
        }

        public bool Contains(Guid guid)
        {
            int count = 0;
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                var command = connection.CreateCommand();
                command.CommandText = $"SELECT COUNT(*) FROM SimulationTable WHERE ID = '{guid}'";
                try
                {
                    using SqliteDataReader reader = command.ExecuteReader();
                    if (reader.Read())
                    {
                        count = (int)reader.GetInt64(0);
                    }
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to count rows from SimulationTable");
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return count >= 1;
        }

        /// <summary>
        /// Returns the list of Guid of all Simulation present in the microservice database 
        /// </summary>
        /// <returns>the list of Guid of all Simulation present in the microservice database</returns>
        public List<Guid>? GetAllSimulationId()
        {
            List<Guid> ids = [];
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                var command = connection.CreateCommand();
                command.CommandText = "SELECT ID FROM SimulationTable";
                try
                {
                    using var reader = command.ExecuteReader();
                    while (reader.Read() && !reader.IsDBNull(0))
                    {
                        Guid id = reader.GetGuid(0);
                        ids.Add(id);
                    }
                    _logger.LogInformation("Returning the list of ID of existing records from SimulationTable");
                    return ids;
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to get IDs from SimulationTable");
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return null;
        }

        /// <summary>
        /// Returns the list of MetaInfo of all Simulation present in the microservice database 
        /// </summary>
        /// <returns>the list of MetaInfo of all Simulation present in the microservice database</returns>
        public List<MetaInfo?>? GetAllSimulationMetaInfo()
        {
            List<MetaInfo?> metaInfos = new();
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                var command = connection.CreateCommand();
                command.CommandText = "SELECT MetaInfo FROM SimulationTable";
                try
                {
                    using var reader = command.ExecuteReader();
                    while (reader.Read() && !reader.IsDBNull(0))
                    {
                        string mInfo = reader.GetString(0);
                        MetaInfo? metaInfo = JsonSerializer.Deserialize<MetaInfo>(mInfo, JsonSettings.Options);
                        metaInfos.Add(metaInfo);
                    }
                    _logger.LogInformation("Returning the list of MetaInfo of existing records from SimulationTable");
                    return metaInfos;
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to get IDs from SimulationTable");
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return null;
        }

        private static readonly object _simulationLock = new();

        private Model.Simulation DeepCopySimulation(Model.Simulation original)
        {
            Scalars scalars = new Model.Scalars();
            if (original?.Results?.Scalars != null)
            {
                scalars = new Model.Scalars
                {
                    AvgCumulativeSSI = original.Results.Scalars.AvgCumulativeSSI,

                    Time = original.Results.Scalars.Time.ToList(),
                    SurfaceRPM = original.Results.Scalars.SurfaceRPM.ToList(),
                    BitRPM = original.Results.Scalars.BitRPM.ToList(),
                    BitDepth = original.Results.Scalars.BitDepth.ToList(),
                    HoleDepth = original.Results.Scalars.HoleDepth.ToList(),
                    SurfaceTorque = original.Results.Scalars.SurfaceTorque.ToList(),
                    BitTorque = original.Results.Scalars.BitTorque.ToList(),
                    TopOfStringAxialVelocity = original.Results.Scalars.TopOfStringAxialVelocity.ToList(),
                    BitAxialVelocity = original.Results.Scalars.BitAxialVelocity.ToList(),
                    WOB = original.Results.Scalars.WOB.ToList(),
                    SSI = original.Results.Scalars.SSI.ToList(),
                    SensorAngularVelocity = original.Results.Scalars.SensorAngularVelocity.ToList(),
                    SensorWhirlVelocity = original.Results.Scalars.SensorWhirlVelocity.ToList(),
                    SensorAxialVelocity = original.Results.Scalars.SensorAxialVelocity.ToList(),
                    SensorRadialVelocity = original.Results.Scalars.SensorRadialVelocity.ToList(),
                    SensorRadialAcc = original.Results.Scalars.SensorRadialAcc.ToList(),
                    SensorTangentialAcc = original.Results.Scalars.SensorTangentialAcc.ToList(),
                    SensorAxialAcc = original.Results.Scalars.SensorAxialAcc.ToList(),
                    SensorBendingMomentX = original.Results.Scalars.SensorBendingMomentX.ToList(),
                    SensorBendingMomentY = original.Results.Scalars.SensorBendingMomentY.ToList(),
                    SensorTension = original.Results.Scalars.SensorTension.ToList(),
                    SensorTorque = original.Results.Scalars.SensorTorque.ToList()

                };
            }
            List<Profiles> profiles = new List<Profiles>();
            if (original?.Results?.Profiles != null)
            {
                profiles = original.Results.Profiles
                        .Select(p => new Model.Profiles
                        {
                            Time = p.Time,
                            Depth = p.Depth.ToList(),
                            DepthAll = p.DepthAll.ToList(),
                            SleevesDepth = p.SleevesDepth.ToList(),
                            SideForce = p.SideForce.ToList(),
                            SideForceSoftString = p.SideForceSoftString.ToList(),
                            PipeAngularVelocity = p.PipeAngularVelocity.ToList(),
                            SleevesAngularVelocity = p.SleevesAngularVelocity.ToList(),
                            RadialClearance = p.RadialClearance.ToList(),
                            LateralDisplacement = p.LateralDisplacement.ToList(),
                            LateralDisplacementAngle = p.LateralDisplacementAngle.ToList(),
                            BendingMoment = p.BendingMoment.ToList(),
                            Torque = p.Torque.ToList(),
                            Tension = p.Tension.ToList(),
                            AxialVelocityD = p.AxialVelocityD.ToList(),
                            Inclination = p.Inclination.ToList(),
                            Azimuth = p.Azimuth.ToList(),
                            Curvature = p.Curvature.ToList(),
                            BuildUpRate = p.BuildUpRate.ToList()
                        })
                        .ToList();
            }
            return new Model.Simulation
            {
                CreationDate = original.CreationDate,
                TerminationState = original.TerminationState,
                CurrentTime = original.CurrentTime,
                Progress = original.Progress,
                ContextualData = original.ContextualData, // Consider deep copy if needed
                InitialValues = original.InitialValues,   // Consider deep copy if mutable
                LastModificationDate = original.LastModificationDate,
                WellBoreID = original.WellBoreID,
                Name = original.Name,
                Description = original.Description,
                MetaInfo = original.MetaInfo,             // Consider deep copy if needed

                Results = new Model.Results
                {
                    Scalars = scalars,
                    Profiles = profiles
                }
            };
        }


        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database without the results
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the Simulation identified by its Guid from the microservice database</returns>
        public Model.Simulation? GetSimulationById(Guid guid)
        {
            if (!guid.Equals(Guid.Empty))
            {

                RunningSimulations.TryGetValue(guid, out var runningSimulation);
                if (runningSimulation != null)
                {
                    lock (_simulationLock) // Only lock long enough to copy
                    {
                        if (runningSimulation?.TerminationState == 0)
                        {
                            return DeepCopySimulation(runningSimulation);
                        }
                    }
                }


                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    Model.Simulation? simulation;
                    var command = connection.CreateCommand();
                    command.CommandText = $"SELECT Simulation FROM SimulationTable WHERE ID = '{guid}'";
                    try
                    {
                        using var reader = command.ExecuteReader();
                        if (reader.Read() && !reader.IsDBNull(0))
                        {
                            string data = reader.GetString(0);
                            simulation = JsonSerializer.Deserialize<Model.Simulation>(data, JsonSettings.Options);
                            if (simulation != null && simulation.MetaInfo != null && !simulation.MetaInfo.ID.Equals(guid))
                                throw new SqliteException("SQLite database corrupted: returned Simulation is null or has been jsonified with the wrong ID.", 1);
                        }
                        else
                        {
                            _logger.LogInformation("No Simulation of given ID in the database");
                            return null;
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to get the Simulation with the given ID from SimulationTable");
                        return null;
                    }
                    _logger.LogInformation("Returning the Simulation of given ID from SimulationTable");
                    return simulation;
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The given Simulation ID is null or empty");
            }
            return null;
        }

        public Model.Results? GetSimulationResultById(Guid guid, uint sequenceNumber)
        {
            if (guid != Guid.Empty)
            {
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    Model.Results? results = null;
                    var command = connection.CreateCommand();
                    command.CommandText = $"SELECT Result FROM SimulationResultsTable WHERE SimulationID = '{guid}' AND SequenceNumber = '{sequenceNumber}'";
                    try
                    {
                        using var reader = command.ExecuteReader();
                        if (reader.Read() && !reader.IsDBNull(0))
                        {

                            string data = reader.GetString(0);
                            if (!string.IsNullOrEmpty(data))
                            {
                                results = JsonSerializer.Deserialize<Results>(data, JsonSettings.Options);
                            }
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to get the Simulation Results with the given SimulationID from SimulationResultsTable");
                        return null;
                    }
                    _logger.LogInformation("Returning the Simulation of given ID from SimulationTable");
                    return results;
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The given Simulation ID is null or empty");
            }
            return null;
        }

        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database 
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the Simulation identified by its Guid from the microservice database</returns>
        public Model.SimulationLight? GetSimulationLightById(Guid guid)
        {
            if (!guid.Equals(Guid.Empty))
            {
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    Model.SimulationLight? simulation;
                    var command = connection.CreateCommand();
                    //command.CommandText = $"SELECT Simulation FROM SimulationTable WHERE ID = '{guid}'";
                    command.CommandText = $"SELECT MetaInfo, Name, Description, CreationDate, LastModificationDate, WellBoreID, Progress, TerminationState FROM SimulationTable WHERE ID = '{guid}'";

                    try
                    {
                        using var reader = command.ExecuteReader();
                        if (reader.Read() && !reader.IsDBNull(0))
                        {
                            var metaInfoJson = reader["MetaInfo"]?.ToString();
                            var metaInfo = string.IsNullOrEmpty(metaInfoJson) ? null : JsonSerializer.Deserialize<MetaInfo>(metaInfoJson);

                            var name = reader["Name"]?.ToString();

                            var description = reader["Description"]?.ToString();

                            var creationDate = DateTimeOffset.TryParse(reader["CreationDate"]?.ToString(), out var cdt) ? cdt : (DateTimeOffset?)null;

                            var lastModificationDate = DateTimeOffset.TryParse(reader["LastModificationDate"]?.ToString(), out var ldt) ? ldt : (DateTimeOffset?)null;

                            Guid wellBoreID = Guid.Empty;
                            if (Guid.TryParse(reader.GetString(5), out Guid lwellBoreID))
                            {
                                wellBoreID = lwellBoreID;
                            }

                            var progress = reader["Progress"] is double d ? d : Convert.ToDouble(reader["Progress"]);

                            var terminationState = Convert.ToInt32(reader["TerminationState"]);

                            simulation = new Model.SimulationLight(metaInfo, name, description, creationDate, lastModificationDate, wellBoreID, progress, terminationState);
                            //string data = reader.GetString(0);
                            //simulation = JsonSerializer.Deserialize<Model.SimulationLight>(data, JsonSettings.Options);
                            if (simulation != null && simulation.MetaInfo != null && !simulation.MetaInfo.ID.Equals(guid))
                                throw new SqliteException("SQLite database corrupted: returned Simulation is null or has been jsonified with the wrong ID.", 1);
                        }
                        else
                        {
                            _logger.LogInformation("No Simulation of given ID in the database");
                            return null;
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to get the Simulation with the given ID from SimulationTable");
                        return null;
                    }
                    _logger.LogInformation("Returning the Simulation of given ID from SimulationTable");
                    return simulation;
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The given Simulation ID is null or empty");
            }
            return null;
        }

        /// <summary>
        /// Returns the list of all Simulation present in the microservice database 
        /// </summary>
        /// <returns>the list of all Simulation present in the microservice database</returns>
        public List<Model.Simulation?>? GetAllSimulation()
        {
            List<Model.Simulation?> vals = [];
            List<MetaInfo?>? IDs = GetAllSimulationMetaInfo();
            if (IDs != null)
            {
                foreach (var id in IDs)
                {
                    if (id != null && id.ID != Guid.Empty)
                    {
                        Simulation? simulation = GetSimulationById(id.ID);
                        if (simulation != null)
                        {
                            vals.Add(simulation);
                        }
                    }
                }
            }
            return vals;
        }

        /// <summary>
        /// Returns the list of all SimulationLight present in the microservice database 
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the list of SimulationLight present in the microservice database</returns>
        public List<Model.SimulationLight>? GetAllSimulationLight()
        {
            List<Model.SimulationLight>? simulationLightList = [];
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                var command = connection.CreateCommand();
                command.CommandText = "SELECT MetaInfo, Name, Description, CreationDate, LastModificationDate, WellBoreID, Progress, TerminationState FROM SimulationTable";
                try
                {
                    using var reader = command.ExecuteReader();
                    while (reader.Read() && !reader.IsDBNull(0))
                    {
                        string metaInfoStr = reader.GetString(0);
                        MetaInfo? metaInfo = JsonSerializer.Deserialize<MetaInfo>(metaInfoStr, JsonSettings.Options);
                        string name = reader.GetString(1);
                        string descr = reader.GetString(2);
                        // make sure DateTimeOffset are properly instantiated when stored values are null (and parsed as empty string)
                        DateTimeOffset? creationDate = null;
                        if (DateTimeOffset.TryParse(reader.GetString(3), out DateTimeOffset cDate))
                            creationDate = cDate;
                        DateTimeOffset? lastModificationDate = null;
                        if (DateTimeOffset.TryParse(reader.GetString(4), out DateTimeOffset lDate))
                            lastModificationDate = lDate;
                        Guid wellBoreID = Guid.Empty;
                        if (Guid.TryParse(reader.GetString(5), out Guid lwellBoreID))
                        {
                            wellBoreID = lwellBoreID;
                        }
                        double progress = reader.GetDouble(6);

                        int terminationState = 0;
                        if (Int32.TryParse(reader.GetString(7), out int lterminationState))
                            terminationState = lterminationState;

                        simulationLightList.Add(new Model.SimulationLight(
                                metaInfo,
                                string.IsNullOrEmpty(name) ? null : name,
                                string.IsNullOrEmpty(descr) ? null : descr,
                                creationDate,
                                lastModificationDate,
                                wellBoreID,
                                progress,
                                terminationState));
                    }
                    _logger.LogInformation("Returning the list of existing SimulationLight from SimulationTable");
                    return simulationLightList;
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to get light datas from SimulationTable");
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return null;
        }

        /// <summary>
        /// Performs calculation on the given Simulation and adds it to the microservice database
        /// </summary>
        /// <param name="simulation"></param>
        /// <returns>true if the given Simulation has been added successfully to the microservice database</returns>
        public async Task<bool> AddSimulation(Model.Simulation? simulation)
        {
            if (simulation != null && simulation.MetaInfo != null && simulation.MetaInfo.ID != Guid.Empty)
            {
                // get drillstring from MS
                DrillString drillString = null;

                DrillStringOpenLab drillStringOpenLab = null;
                switch (simulation.ContextualData.DrillStringSource)
                {
                    case DrillStringSourceType.DrillStringOpenLabFile:
                        // Use AnnulusPressureFile (assumed used elsewhere in your logic)
                        break;

                    case DrillStringSourceType.DrillStringOpenLabMS:
                        try
                        {
                            drillStringOpenLab = await APIUtils.ClientDrillStringOpenLab.GetDrillStringOpenLabByIdAsync(simulation.ContextualData.DrillStringOpenLabID);
                        }

                        catch (Exception ex)
                        {
                            _logger.LogWarning("Unable to get the drillstring from OpenLab MS for the 4ndof simulation: " + ex);
                            return false;
                        }
                        break;

                    case DrillStringSourceType.DrillStringMS:
                        try
                        {
                            drillString = await APIUtils.ClientDrillString.GetDrillStringByIdAsync(simulation.ContextualData.DrillStringID);
                        }
                        catch (Exception ex)
                        {
                            _logger.LogWarning("Unable to get the drillstring from MS for the 4ndof simulation: " + ex);
                            return false;
                        }
                        break;
                }



                Simulator.DataModel.Configuration config;
                //initialize simulation
                try
                {
                    config = Initialize(simulation, drillString, drillStringOpenLab, simulation.ContextualData.DrillStringSource);
                }
                catch (Exception ex)
                {
                    _logger.LogWarning("Impossible to intialize the 4ndof simulation: " + ex.ToString());
                    return false;
                }

                //if successful, check if another parent data with the same ID was calculated/added during the calculation time
                Model.Simulation? newSimulation = GetSimulationById(simulation.MetaInfo.ID);
                if (newSimulation == null)
                {
                    //update SimulationTable
                    var connection = _connectionManager.GetConnection();
                    if (connection != null)
                    {
                        using SqliteTransaction transaction = connection.BeginTransaction();
                        bool success = true;
                        try
                        {
                            //add the Simulation to the SimulationTable
                            string metaInfo = JsonSerializer.Serialize(simulation.MetaInfo, JsonSettings.Options);
                            string? cDate = null;
                            if (simulation.CreationDate != null)
                                cDate = ((DateTimeOffset)simulation.CreationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                            string? lDate = null;
                            if (simulation.LastModificationDate != null)
                                lDate = ((DateTimeOffset)simulation.LastModificationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                            string data = JsonSerializer.Serialize(simulation, JsonSettings.Options);
                            var command = connection.CreateCommand();
                            double progress = 0.0;
                            int terminationState = 0;
                            command.CommandText = "INSERT INTO SimulationTable (" +
                                "ID, " +
                                "MetaInfo, " +
                                "Name, " +
                                "Description, " +
                                "CreationDate, " +
                                "LastModificationDate, " +
                                "WellBoreID, " +
                                "Progress, " +
                                "TerminationState, " +
                                "Simulation" +
                                ") VALUES (" +
                                $"'{simulation.MetaInfo.ID}', " +
                                $"'{metaInfo}', " +
                                $"'{simulation.Name}', " +
                                $"'{simulation.Description}', " +
                                $"'{cDate}', " +
                                $"'{lDate}', " +
                                $"'{simulation.WellBoreID}', " +
                                $"'{progress}', " +
                                $"'{terminationState}', " +
                                $"'{data}'" +
                                ")";
                            int count = command.ExecuteNonQuery();
                            if (count != 1)
                            {
                                _logger.LogWarning("Impossible to insert the given Simulation into the SimulationTable");
                                success = false;
                            }
                        }
                        catch (SqliteException ex)
                        {
                            _logger.LogError(ex, "Impossible to add the given Simulation into SimulationTable");
                            success = false;
                        }
                        //finalizing SQL transaction
                        if (success)
                        {
                            transaction.Commit();
                            _logger.LogInformation("Added the given Simulation of given ID into the SimulationTable successfully");
                        }
                        else
                        {
                            transaction.Rollback();
                        }
                    }
                    else
                    {
                        _logger.LogWarning("Impossible to access the SQLite database");
                    }
                }
                else
                {
                    _logger.LogWarning("Impossible to post Simulation. ID already found in database.");
                    return false;
                }

                // Run simulation in the background
#pragma warning disable CS4014 // Because this call is not awaited, execution of the current method continues before the call is completed
                Task.Run(async () =>
                {
                    bool acquired = false;
                    simulation.TerminationState = 0;
                    RunningSimulations[simulation.MetaInfo.ID] = simulation;
                    try
                    {
                        await _simulationSemaphore.WaitAsync();
                        acquired = true;

                        if (await CalculateAsync(simulation, config))
                        {
                            FinalUpdateSimulatorById(simulation.MetaInfo.ID, simulation);
                        }
                        else
                        {
                            _logger.LogWarning("Background simulation failed");
                        }
                    }
                    catch (Exception ex)
                    {
                        _logger.LogError(ex, "Exception during background simulation");
                    }
                    finally
                    {
                        if (acquired)
                        {
                            _simulationSemaphore.Release();
                            RunningSimulations.TryRemove(simulation.MetaInfo.ID, out _);
                        }
                    }
                });
#pragma warning restore CS4014 // Because this call is not awaited, execution of the current method continues before the call is completed
                return true; // ✅ Return immediately

            }
            else
            {
                _logger.LogWarning("The Simulation ID or the ID of its input are null or empty");
            }
            return false;
        }

        public bool UpdateProgressSimulationById(Guid guid, Model.Simulation? simulation)
        {
            bool success = true;
            if (guid != Guid.Empty && simulation != null && simulation.MetaInfo != null && simulation.MetaInfo.ID == guid)
            {

                //update SimulationTable
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    using SqliteTransaction transaction = connection.BeginTransaction();
                    //update fields in SimulationTable
                    try
                    {
                        string metaInfo = JsonSerializer.Serialize(simulation.MetaInfo, JsonSettings.Options);
                        string? cDate = null;
                        if (simulation.CreationDate != null)
                            cDate = ((DateTimeOffset)simulation.CreationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                        string? lDate = null;
                        if (simulation.LastModificationDate != null)
                            lDate = ((DateTimeOffset)simulation.LastModificationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                        //string data = JsonSerializer.Serialize(simulator, JsonSettings.Options);
                        double progress = simulation.Progress ?? 0.0;
                        int terminationState = simulation.TerminationState ?? 0;
                        var command = connection.CreateCommand();
                        command.CommandText = $"UPDATE SimulationTable SET " +
                            $"MetaInfo = '{metaInfo}', " +
                            $"Name = '{simulation.Name}', " +
                            $"Description = '{simulation.Description}', " +
                            $"CreationDate = '{cDate}', " +
                            $"LastModificationDate = '{lDate}', " +
                            $"WellBoreID = '{simulation.WellBoreID}', " +
                            $"Progress = {progress.ToString(CultureInfo.InvariantCulture)}, " +
                            $"TerminationState= '{terminationState}' " +
                            $"WHERE ID = '{guid}'";
                        int count = command.ExecuteNonQuery();
                        if (count != 1)
                        {
                            _logger.LogWarning("Impossible to update the Simulator");
                            success = false;
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to update the Simulator");
                        success = false;
                    }

                    // Finalizing
                    if (success)
                    {
                        transaction.Commit();
                        //_logger.LogInformation("Updated the given Simulator successfully");
                        return true;
                    }
                    else
                    {
                        transaction.Rollback();
                    }
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The Simulator ID or the ID of some of its attributes are null or empty");
            }
            return false;
        }

        /// <summary>
        /// Performs calculation on the given Simulation and updates it in the microservice database
        /// </summary>
        /// <param name="simulation"></param>
        /// <returns>true if the given Simulation has been updated successfully</returns>
        public bool FinalUpdateSimulatorById(Guid guid, Model.Simulation? simulation)
        {
            bool success = true;
            if (guid != Guid.Empty && simulation != null && simulation.MetaInfo != null && simulation.MetaInfo.ID == guid)
            {

                //update SimulationTable
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    using SqliteTransaction transaction = connection.BeginTransaction();
                    //update fields in SimulationTable
                    try
                    {
                        string metaInfo = JsonSerializer.Serialize(simulation.MetaInfo, JsonSettings.Options);
                        string? cDate = null;
                        if (simulation.CreationDate != null)
                            cDate = ((DateTimeOffset)simulation.CreationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                        string? lDate = null;
                        if (simulation.LastModificationDate != null)
                            lDate = ((DateTimeOffset)simulation.LastModificationDate).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                        var results = simulation.Results;
                        simulation.Results = null;
                        string data = JsonSerializer.Serialize(simulation, JsonSettings.Options);
                        var command = connection.CreateCommand();
                        double progress = simulation.Progress ?? 0.0;
                        int terminationState = simulation.TerminationState ?? 0;
                        command.CommandText = $"UPDATE SimulationTable SET " +
                            $"MetaInfo = '{metaInfo}', " +
                            $"Name = '{simulation.Name}', " +
                            $"Description = '{simulation.Description}', " +
                            $"CreationDate = '{cDate}', " +
                            $"LastModificationDate = '{lDate}', " +
                            $"WellBoreID = '{simulation.WellBoreID}', " +
                            $"Progress = {progress.ToString(CultureInfo.InvariantCulture)}, " +
                            $"TerminationState= '{terminationState}', " +
                            $"Simulation = '{data}' " +
                            $"WHERE ID = '{guid}'";
                        int count = command.ExecuteNonQuery();
                        if (count != 1)
                        {
                            _logger.LogWarning("Impossible to update the Simulation");
                            success = false;
                        }
                        if (success)
                        {
                            command.CommandText = $"DELETE FROM SimulationResultsTable WHERE SimulationID = '{guid}'";
                            count = command.ExecuteNonQuery();
                            if (count < 0)
                            {
                                _logger.LogWarning("Impossible to delete the Simulation Results of given ID from the SimulationResultsTable");
                                success = false;
                            }
                            if (success && results != null)
                            {

                                List<Results> resultsList = SplitResults(results, _maxCount);
                                uint sequenceNumber = 0;
                                foreach (Results r in resultsList)
                                {
                                    command.CommandText = "INSERT INTO SimulationResultsTable (" +
                                        "SimulationID, " +
                                        "SequenceNumber, " +
                                        "Result" +
                                        ") VALUES (" +
                                        $"'{guid}', " +
                                        $"'{sequenceNumber}', " +
                                        $"'{JsonSerializer.Serialize(r, JsonSettings.Options)}'" +
                                        ")";
                                    count = command.ExecuteNonQuery();
                                    if (count != 1)
                                    {
                                        _logger.LogWarning("Impossible to insert the given Simulation Result into the SimulationResultsTable");
                                        success = false;
                                        break;
                                    }
                                    sequenceNumber++;
                                }
                            }
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to update the Simulation");
                        success = false;
                    }

                    // Finalizing
                    if (success)
                    {
                        transaction.Commit();
                        _logger.LogInformation("Simulation finished. Updated successfully");
                        return true;
                    }
                    else
                    {
                        transaction.Rollback();
                    }
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The Simulation ID or the ID of some of its attributes are null or empty");
            }
            return false;
        }

        /// <summary>
        /// Deletes the Simulation of given ID from the microservice database
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>true if the Simulation was deleted from the microservice database</returns>
        public bool DeleteSimulationById(Guid guid)
        {
            if (!guid.Equals(Guid.Empty))
            {
                var connection = _connectionManager.GetConnection();
                if (connection != null)
                {
                    using var transaction = connection.BeginTransaction();
                    bool success = true;
                    //delete Simulation from SimulationTable
                    try
                    {
                        var command = connection.CreateCommand();
                        command.CommandText = $"DELETE FROM SimulationResultsTable WHERE SimulationID = '{guid}'";
                        int count = command.ExecuteNonQuery();
                        if (count < 0)
                        {
                            _logger.LogWarning("Impossible to delete the Simulation of given ID from the SimulationResultsTable");
                            success = false;
                        }
                        command.CommandText = $"DELETE FROM SimulationTable WHERE ID = '{guid}'";
                        count = command.ExecuteNonQuery();
                        if (count < 0)
                        {
                            _logger.LogWarning("Impossible to delete the Simulation of given ID from the SimulationTable");
                            success = false;
                        }
                    }
                    catch (SqliteException ex)
                    {
                        _logger.LogError(ex, "Impossible to delete the Simulation of given ID from SimulationTable");
                        success = false;
                    }
                    if (success)
                    {
                        transaction.Commit();
                        _logger.LogInformation("Removed the Simulation of given ID from the SimulationTable successfully");
                    }
                    else
                    {
                        transaction.Rollback();
                    }
                    return success;
                }
                else
                {
                    _logger.LogWarning("Impossible to access the SQLite database");
                }
            }
            else
            {
                _logger.LogWarning("The Simulation ID is null or empty");
            }
            return false;
        }

        private List<Results> SplitResults(Results? results, int maxCount)
        {
            List<Results> output = new List<Results>();
            if (results != null)
            {
                int countScalars = 0;
                if (results.Scalars != null && results.Scalars.Time != null)
                {
                    countScalars = results.Scalars.Time.Count;
                }
                int countProfiles = 0;
                if (results.Profiles != null)
                {
                    countProfiles = results.Profiles.Count;
                }
                int numberChunkScalars = countScalars / maxCount;
                if (countScalars % maxCount > 0)
                {
                    numberChunkScalars++;
                }
                int numberChunkProfiles = countProfiles / maxCount;
                if (countProfiles % maxCount > 0)
                {
                    numberChunkProfiles++;
                }
                int countScalarSamples = 0;
                int countProfilesSamples = 0;
                if (numberChunkProfiles > numberChunkScalars)
                {
                    countProfilesSamples = maxCount;
                    if (numberChunkProfiles > 0)
                    {
                        countScalarSamples = countScalars / numberChunkProfiles;
                    }
                }
                else
                {
                    countScalarSamples = maxCount;
                    if (numberChunkScalars > 0)
                    {
                        countProfilesSamples = countProfiles / numberChunkScalars;
                    }
                }
                int idxScalars = 0;
                int idxProfiles = 0;
                var srcScalars = results.Scalars;
                while (idxScalars < countScalars || idxProfiles < countProfiles)
                {
                    Results res = new Results();
                    res.Scalars = new Scalars();
                    res.Profiles = new List<Profiles>();
                    res.AvgCumulativeSSI = results.AvgCumulativeSSI;

                    var dstScalars = res.Scalars;
                    int c = 0;
                    for (int i = idxScalars; i < Math.Min(idxScalars + countScalarSamples, countScalars); i++)
                    {
                        foreach (var prop in typeof(Scalars).GetProperties()
                                                         .Where(p => p.PropertyType == typeof(List<double>)))
                        {
                            var src = (List<double>?)prop.GetValue(srcScalars);
                            if (src is null || src.Count == 0) continue;

                            var dst = (List<double>?)prop.GetValue(dstScalars);
                            if (dst is null)
                            {
                                dst = new List<double>();
                                prop.SetValue(dstScalars, dst);
                            }
                            if (i < src.Count)
                            {
                                dst.Add(src[i]);
                            }
                        }
                        c++;
                    }
                    idxScalars += c;
                    c = 0;
                    for (int i = idxProfiles; i < Math.Min(idxProfiles + countProfilesSamples, countProfiles); i++)
                    {
                        if (results.Profiles != null && i < results.Profiles.Count)
                        {
                            res.Profiles.Add(results.Profiles[i]);
                        }
                        c++;
                    }
                    idxProfiles += c;
                    output.Add(res);
                }
            }
            return output;
        }
        public Simulator.DataModel.Configuration Initialize(Simulation simulation, DrillString drillString, DrillStringOpenLab drillStringOpenLab, DrillStringSourceType DrillStringSource)
        {

            var config = new Simulator.DataModel.Configuration()
            {
                AnnulusPressureFile = simulation.ContextualData.AnnulusPressureFile,
                TrajectoryFile = simulation.ContextualData.TrajectoryFile,
                DrillStringSourceType = DrillStringSource,
                DrillstringFile = simulation.ContextualData.DrillstringFile,
                DrillString = drillString,
                DrillStringOpenLab = drillStringOpenLab,
                StringPressureFile = simulation.ContextualData.DrillstringPressureFile,
                BitDepth = simulation.InitialValues.BitDepth,                            // [m]
                HoleDepth = simulation.InitialValues.HoleDepth,                           // [m]
                TopOfStringPosition = simulation.InitialValues.TopOfStringPosition,                   // [m]
                SurfaceRPM = simulation.SetPointsList?.Count > 0 ? simulation.SetPointsList[0].SurfaceRPM : 0,            // [rad/s]
                TopOfStringVelocity = simulation.SetPointsList?.Count > 0 ? simulation.SetPointsList[0].TopOfStringVelocity : 0,         // [m/s] 
                BoreHoleSizes = simulation.ContextualData.BoreHoleSizeList,
                BitRadius = simulation.ContextualData.BitRadius,                     // [m]
                //SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700 }),
                SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { }),
                SensorDistanceFromBit = simulation.Config.VirtualSensorPositionFromBit,
                FluidDensity = simulation.ContextualData.FluidDensity,                       // [kg/m3] Density of drilling mud
                LengthBetweenLumpedElements = simulation.Config.LengthBetweenLumpedElements,
                CoulombKineticFriction = simulation.Config.CoulombKineticFriction,
                CoulombStaticFriction = simulation.Config.CoulombStaticFriction,
                UseHeave = simulation.Config.UseHeave,
                HeaveAmplitude = simulation.Config.HeaveAmplitude,
                HeavePeriod = simulation.Config.HeavePeriod,
                TopdriveStartupTime = simulation.Config.TopDriveStartupTime,
                TopDriveController = simulation.Config.TopDriveController,
                VFDFilterTimeconstantZTorque = simulation.Config.VFDFilterTimeconstantZTorque,
                EncoderTimeConstantZTorque = simulation.Config.EncoderTimeConstantZTorque,
                AccelerationFilterTimeConstantZTorque = simulation.Config.AccelerationFilterTimeConstantZTorque,
                TorqueHighPassFilterTimeConstantZTorque = simulation.Config.TorqueHighPassFilterTimeConstantZTorque,
                TorqueLowPassFilterTimeConstantZTorque = simulation.Config.TorqueLowPassFilterTimeConstantZTorque,
                AdditionalTuningFactorZTorque = simulation.Config.AdditionalTuningFactorZTorque,
                InertiaCorrectionFactorZTorque = simulation.Config.InertiaCorrectionFactorZTorque,
                KpSoftTorqueSpeed = simulation.Config.KpSoftTorqueSpeed,
                KiSoftTorqueSpeed = simulation.Config.KiSoftTorqueSpeed,
                TuningFrequenceySoftTorqueSpeed = simulation.Config.TuningFrequenceySoftTorqueSpeed,
                KpStiffAndZTorque = simulation.Config.KpStiffAndZTorque,
                KiStiffAndZTorque = simulation.Config.KiStiffAndZTorque,

            };

            return config;
        }

        private Profiles CreateProfile(double time, Output output, State state, SimulationParameters parameters, Input u, Simulator.DataModel.Configuration config)
        {
            return new Profiles
            {
                Time = time,
                Depth = output.Depth.ToList(),
                DepthAll = parameters.DistributedCells.x.ToList(),
                SleevesDepth = parameters.Drillstring.SleeveIndexPosition.Select(index => parameters.LumpedCells.ElementLength[(int)index]).ToList(),
                SideForce = Utilities.ExtendVectorStart(0, output.NormalForceProfileStiffString).ToList(),
                SideForceSoftString = Utilities.ExtendVectorStart(0, output.NormalForceProfileSoftString).ToList(),
                PipeAngularVelocity = !config.UseMudMotor
                    ? Utilities.ExtendVectorStart(state.TopDriveAngularVelocity, state.AngularVelocity).Append(output.BitRotationInRPM).ToList()
                    : Utilities.ExtendVectorStart(state.TopDriveAngularVelocity,
                        Utilities.ToVector(state.AngularVelocity.SubVector(1, state.AngularVelocity.Count - 1).Append(output.BitRotationInRPM).ToArray())).ToList(),
                SleevesAngularVelocity = state.SleeveAngularVelocity.ToList(),
                RadialClearance = parameters.Wellbore.rc.Append(0).ToList(),
                LateralDisplacement = output.RadialDisplacement.Append(0).ToList(),
                BendingMoment = output.BendingMoment.Append(0).ToList(),
                Torque = output.Torque.ToList(),
                Tension = output.TensionProfile.ToList(),
                AxialVelocityD = Utilities.ExtendVectorStart(u.CalculateSurfaceAxialVelocity, state.AngularVelocity).ToList(),
                LateralDisplacementAngle = output.WhirlAngle.Append(0).ToList(),
                Inclination = parameters.Trajectory.thetaVec.Append(parameters.Trajectory.thetaVec.LastOrDefault<double>()).ToList(),
                Azimuth = parameters.Trajectory.phiVec.Append(parameters.Trajectory.phiVec.LastOrDefault<double>()).ToList(),
                Curvature = parameters.Trajectory.curvature.Append(parameters.Trajectory.curvature.LastOrDefault<double>()).ToList(),
                BuildUpRate = parameters.Trajectory.thetaVec_dot.Append(parameters.Trajectory.thetaVec_dot.LastOrDefault<double>()).ToList()
            };
        }

        private void LogScalarValues(Scalars scalars, double time, Output output, State state, Input u)
        {
            // Ensure Scalars is initialized
            scalars ??= new Scalars();

            scalars.Time.Add(time);
            scalars.SurfaceRPM.Add(output.TopDriveRotationInRPM);
            scalars.BitRPM.Add(output.BitRotationInRPM);
            scalars.BitDepth.Add(state.BitDepth);
            scalars.HoleDepth.Add(state.HoleDepth);
            scalars.SurfaceTorque.Add(u.TopDriveTorque);
            scalars.BitTorque.Add(output.TorqueOnBit);
            scalars.TopOfStringAxialVelocity.Add(u.CalculateSurfaceAxialVelocity);
            scalars.BitAxialVelocity.Add(output.BitVelocity);
            scalars.WOB.Add(output.WeightOnBit);
            scalars.SSI.Add(output.SSI);
            scalars.AvgCumulativeSSI = output.AverageStickSlipIndex;
            scalars.SensorAngularVelocity.Add(output.SensorAngularVelocity);
            scalars.SensorWhirlVelocity.Add(output.SensorWhirlSpeed);
            scalars.SensorAxialVelocity.Add(output.SensorAxialVelocity);
            scalars.SensorRadialVelocity.Add(output.SensorRadialSpeed);
            scalars.SensorRadialAcc.Add(output.SensorRadialAccelerationLocalFrame);
            scalars.SensorTangentialAcc.Add(output.SensorTangentialAccelerationLocalFrame);
            scalars.SensorAxialAcc.Add(output.SensorAxialAccelerationLocalFrame);
            scalars.SensorBendingMomentX.Add(output.SensorBendingMomentX);
            scalars.SensorBendingMomentY.Add(output.SensorBendingMomentY);
            scalars.SensorTension.Add(output.SensorTension);
            scalars.SensorTorque.Add(output.SensorTorque);
        }


        /// <summary>
        /// Executes the main simulation loop for the given Simulation and ServiceConfiguration.
        /// 
        /// Key Responsibilities:
        /// - Initializes simulation state and solver.
        /// - Iterates over SetPoints using fixed outer time steps.
        /// - Logs scalar values (time-based) at defined intervals.
        /// - Logs profile values (depth-based) with the following behavior:
        ///     - A new profile is added on the first scalar log after a profile log.
        ///     - That profile is then updated at each scalar log.
        ///     - If the current time matches the profile logging interval, the last profile is finalized.
        /// - Ensures time starts at 0 by adjusting currentTime = (state.step - 1) * outerTimeStep.
        /// - Periodically updates progress and simulation result in the database.
        /// - Finalizes the last scalar and profile entry if needed.
        /// 
        /// This structure ensures:
        /// - Accurate logging according to defined intervals.
        /// - Efficient updates during simulation (one DB write per interval).
        /// - Real-time access to up-to-date scalar and profile results throughout the run.
        /// </summary>

        public async Task<bool> CalculateAsync(Simulation simulation, Simulator.DataModel.Configuration config)
        {
            if (simulation.SetPointsList == null || simulation.SetPointsList.Count == 0)
                return false;

            var parameters = new SimulationParameters(configuration: config);
            var solver = new Solver(parameters, in config);
            simulation.Results = new Results { Scalars = new Scalars() };

            double totalDuration = simulation.SetPointsList.Sum(sp => sp.TimeDuration);
            double outerTimeStep = config.TimeStep;

            // Setup scalar logging interval
            double scalarIntervalRaw = simulation.Config.LoggingIntervalScalarValues;
            double scalarInterval = (scalarIntervalRaw <= 0 || scalarIntervalRaw < outerTimeStep)
                ? outerTimeStep
                : Math.Round(scalarIntervalRaw / outerTimeStep) * outerTimeStep;

            // Setup profile logging interval
            double profileIntervalRaw = simulation.Config.LoggingIntervalProfiles;
            bool logProfilesAtFixedInterval = profileIntervalRaw >= outerTimeStep;
            double profileInterval = logProfilesAtFixedInterval
                ? Math.Round(profileIntervalRaw / outerTimeStep) * outerTimeStep
                : double.MaxValue;

            double nextScalarLogTime = 0.0;
            double nextProfileLogTime = 0.0;
            double currentTime = 0.0;

            State lastState = null;
            Output lastOutput = null;
            Input lastControl = null;

            bool shouldAddNewProfile = true;

            foreach (var setPoints in simulation.SetPointsList)
            {
                double duration = setPoints.TimeDuration;
                int steps = (int)(duration / outerTimeStep);

                for (int i = 0; i < steps; i++)
                {
                    var (state, output, u) = solver.OuterStep(setPoints.SurfaceRPM, setPoints.TopOfStringVelocity, setPoints.BottomExtraSideForce, setPoints.DifferenceStaticKineticFriction, setPoints.StribeckCriticalVelocity, setPoints.Sticking);
                    currentTime = (state.Step - 1) * outerTimeStep; // start with 0 sec
                    simulation.Progress = currentTime / totalDuration;

                    lastState = state;
                    lastOutput = output;
                    lastControl = u;

                    bool logScalar = currentTime >= nextScalarLogTime;
                    bool logProfile = logProfilesAtFixedInterval && currentTime + 0.5 * outerTimeStep >= nextProfileLogTime;

                    if (logScalar)
                    {
                        nextScalarLogTime += scalarInterval;
                        LogScalarValues(simulation.Results.Scalars, currentTime, output, state, u);
                    }

                    if (logProfile)
                    {
                        nextProfileLogTime += profileInterval;
                        simulation.Results.Profiles.Add(CreateProfile(currentTime, output, state, parameters, u, config));
                    }

                    if (logProfile || logScalar)
                    {
                        await Task.Run(() =>
                        {
                            UpdateProgressSimulationById(simulation.MetaInfo.ID, simulation);
                            RunningSimulations[simulation.MetaInfo.ID] = simulation;
                        });
                    }
                }
            }

            // Final scalar log if not already up-to-date
            if (lastOutput != null && lastState != null && lastControl != null)
            {
                LogScalarValues(simulation.Results.Scalars, totalDuration, lastOutput, lastState, lastControl);

                if (simulation.Results.Profiles.Count == 0 || simulation.Results.Profiles[^1].Time < totalDuration)
                {
                    var finalProfileTime = logProfilesAtFixedInterval
                        ? Math.Floor(totalDuration / profileInterval) * profileInterval
                        : totalDuration;

                    var finalProfile = CreateProfile(finalProfileTime, lastOutput, lastState, parameters, lastControl, config);
                    if (shouldAddNewProfile)
                        simulation.Results.Profiles.Add(finalProfile);
                    else
                        simulation.Results.Profiles[^1] = finalProfile;
                }
            }

            simulation.TerminationState = 1;
            simulation.Progress = 1;

            await Task.Run(() => UpdateProgressSimulationById(simulation.MetaInfo.ID, simulation));
            RunningSimulations[simulation.MetaInfo.ID] = simulation;

            return true;
        }

    }
}