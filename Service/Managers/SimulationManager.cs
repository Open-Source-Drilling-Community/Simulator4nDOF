using MathNet.Numerics.LinearAlgebra;
using Microsoft.Data.Sqlite;
using Microsoft.Extensions.Logging;
using NORCE.Drilling.Simulator;
using NORCE.Drilling.Simulator.DataModel;
using NORCE.Drilling.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Model;
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

namespace NORCE.Drilling.Simulator4nDOF.Service.Managers
{
    /// <summary>
    /// A manager for Simulation. The manager implements the singleton pattern as defined by 
    /// Gamma, Erich, et al. "Design patterns: Abstraction and reuse of object-oriented design." 
    /// European Conference on Object-Oriented Programming. Springer, Berlin, Heidelberg, 1993.
    /// </summary>
    public class SimulationManager
    {
        private static SimulationManager? _instance = null;
        private readonly ILogger<SimulationManager> _logger;
        private readonly object _lock = new();
        private readonly SqlConnectionManager _connectionManager;
        private static readonly SemaphoreSlim _simulationSemaphore = new SemaphoreSlim(1); // limit to 1 concurrent simulations


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
                lock (_lock)
                {
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
            return new Model.Simulation
            {
                CreationDate = original.CreationDate,
                TerminationState = original.TerminationState,
                CurrentTime = original.CurrentTime,
                Progress = original.Progress,
                ContextualData = original.ContextualData, // You might want to deep copy this too
                InitialValues = original.InitialValues,   // Same here, deep copy if it's mutable
                LastModificationDate = original.LastModificationDate,
                Name = original.Name,
                Description = original.Description,
                MetaInfo = original.MetaInfo,             // Same here if needed

                Results = new Model.Results
                {
                    AvgCumulativeSSI = original.Results.AvgCumulativeSSI,

                    Time = original.Results.Time.ToList(),
                    SurfaceRPM = original.Results.SurfaceRPM.ToList(),
                    BitRPM = original.Results.BitRPM.ToList(),
                    BitDepth = original.Results.BitDepth.ToList(),
                    HoleDepth = original.Results.HoleDepth.ToList(),
                    SurfaceTorque = original.Results.SurfaceTorque.ToList(),
                    BitTorque = original.Results.BitTorque.ToList(),
                    TopOfStringAxialVelocity = original.Results.TopOfStringAxialVelocity.ToList(),
                    BitAxialVelocity = original.Results.BitAxialVelocity.ToList(),
                    WOB = original.Results.WOB.ToList(),
                    SSI = original.Results.SSI.ToList(),

                    SensorAngularVelocity = original.Results.SensorAngularVelocity.ToList(),
                    SensorWhirlVelocity = original.Results.SensorWhirlVelocity.ToList(),
                    SensorAxialVelocity = original.Results.SensorAxialVelocity.ToList(),
                    SensorRadialVelocity = original.Results.SensorRadialVelocity.ToList(),
                    SensorRadialAcc = original.Results.SensorRadialAcc.ToList(),
                    SensorTangentialAcc = original.Results.SensorTangentialAcc.ToList(),
                    SensorAxialAcc = original.Results.SensorAxialAcc.ToList(),
                    SensorBendingMomentX = original.Results.SensorBendingMomentX.ToList(),
                    SensorBendingMomentY = original.Results.SensorBendingMomentY.ToList(),

                    Depth = original.Results.Depth.ToList(),
                    DepthAll = original.Results.DepthAll.ToList(),
                    SleevesDepth = original.Results.SleevesDepth.ToList(),
                    SideForce = original.Results.SideForce.ToList(),
                    SideForceSoftString = original.Results.SideForceSoftString.ToList(),
                    PipeAngularVelocity = original.Results.PipeAngularVelocity.ToList(),
                    SleevesAngularVelocity = original.Results.SleevesAngularVelocity.ToList(),
                    RadialClearance = original.Results.RadialClearance.ToList(),
                    LateralDisplacement = original.Results.LateralDisplacement.ToList(),
                    LateralDisplacementAngle = original.Results.LateralDisplacementAngle.ToList(),
                    BendingMoment = original.Results.BendingMoment.ToList(),
                    Torque = original.Results.Torque.ToList(),
                    Tension = original.Results.Tension.ToList(),
                    AxialVelocityD = original.Results.AxialVelocityD.ToList()
                }
            };
        }


        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database 
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
                    command.CommandText = $"SELECT MetaInfo, Name, Description, CreationDate, LastModificationDate, Progress, TerminationState FROM SimulationTable WHERE ID = '{guid}'";

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

                            var progress = reader["Progress"] is double d ? d : Convert.ToDouble(reader["Progress"]);
                            var terminationState = Convert.ToInt32(reader["TerminationState"]);

                            simulation = new Model.SimulationLight(metaInfo, name, description, creationDate, lastModificationDate, progress, terminationState);
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
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                var command = connection.CreateCommand();
                command.CommandText = "SELECT Simulation FROM SimulationTable";
                try
                {
                    using var reader = command.ExecuteReader();
                    while (reader.Read() && !reader.IsDBNull(0))
                    {
                        string data = reader.GetString(0);
                        Model.Simulation? simulation = JsonSerializer.Deserialize<Model.Simulation>(data, JsonSettings.Options);
                        vals.Add(simulation);
                    }
                    _logger.LogInformation("Returning the list of existing Simulation from SimulationTable");
                    return vals;
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to get Simulation from SimulationTable");
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return null;
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
                command.CommandText = "SELECT MetaInfo, Name, Description, CreationDate, LastModificationDate, Progress, TerminationState FROM SimulationTable";
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

                        double progress = reader.GetDouble(5);
                        
                        int terminationState = 0;
                        if (Int32.TryParse(reader.GetString(6), out int lterminationState))
                            terminationState = lterminationState;

                        simulationLightList.Add(new Model.SimulationLight(
                                metaInfo,
                                string.IsNullOrEmpty(name) ? null : name,
                                string.IsNullOrEmpty(descr) ? null : descr,
                                creationDate,
                                lastModificationDate,
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
        public bool AddSimulation(Model.Simulation? simulation)
        {
            if (simulation != null && simulation.MetaInfo != null && simulation.MetaInfo.ID != Guid.Empty)
            {
                Configuration config;
                //initialize simulation
                try
                {
                    config = Initialize(simulation);
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
                        _logger.LogInformation("Updated the given Simulator successfully");
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
                        _logger.LogInformation("Updated the given Simulation successfully");
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
                        command.CommandText = $"DELETE FROM SimulationTable WHERE ID = '{guid}'";
                        int count = command.ExecuteNonQuery();
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



        private double outerTimeStep;

        public Configuration Initialize(Simulation simulation)
        {
            var config = new Configuration()
            {
                AnnulusPressureFile = simulation.ContextualData.AnnulusPressureFile,
                TrajectoryFile = simulation.ContextualData.TrajectoryFile,
                DrillstringFile = simulation.ContextualData.DrillstringFile,
                StringPressureFile = simulation.ContextualData.DrillstringPressureFile,
                BitDepth = simulation.InitialValues.BitDepth,                            // [m]
                HoleDepth = simulation.InitialValues.HoleDepth,                           // [m]
                TopOfStringPosition = simulation.InitialValues.TopOfStringPosition,                   // [m]
                SurfaceRPM = simulation.SetPointsList?.Count > 0 ? simulation.SetPointsList[0].SurfaceRPM : 0,            // [rad/s]
                TopOfStringVelocity = simulation.SetPointsList?.Count > 0 ? simulation.SetPointsList[0].TopOfStringVelocity : 0,         // [m/s] 
                CasingShoeDepth = simulation.ContextualData.CasingShoeDepth,                     // [m] Casing shoe depth
                LinerShoeDepth = simulation.ContextualData.LinerShoeDepth,                         // [m] Liner shoe depth, set to 0 if there is no liner
                CasingID = simulation.ContextualData.CasingID,                        // [m] Casing inner diameter
                LinerID = simulation.ContextualData.LinerID,                         // [m] Casing outer diameter
                WellheadDepth = simulation.ContextualData.WellheadDepth,                        // [m] Well head depth
                RiserID = simulation.ContextualData.RiserID,                        // [m]
                BitRadius = simulation.ContextualData.BitRadius,                     // [m]
                //SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700 }),
                SleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(new double[] { }),
                SensorDistanceFromBit = 63,
                FluidDensity = 1200,                       // [kg/m3] Density of drilling mud
                LengthBetweenLumpedElements = 30,
            };

            outerTimeStep = config.TimeStep;

            return config;
        }

        public async Task<bool> CalculateAsync(Simulation simulation, Configuration config)
        {
            var parameters = new Parameters(c: config);
            Solver solver = new Solver(parameters, in config); ;
            if (simulation.SetPointsList == null || simulation.SetPointsList.Count == 0)
                return false;

            simulation.Results = new Results();
            double totalDuration = simulation.SetPointsList.Sum(sp => sp.TimeDuration);

            var newRestults = new Results();

            foreach (var setPoints in simulation.SetPointsList)
            {
                double duration = setPoints.TimeDuration;
                int steps = (int)(duration / outerTimeStep);

                for (int i = 0; i < steps; i++)
                {
                    // Simulate one step (assumed to be a fast, synchronous calculation)
                    var (state, output, u) = solver.OuterStep(setPoints.SurfaceRPM, setPoints.TopOfStringVelocity);

                    double Mb_x = output.sensorMb_x;
                    double Mb_y = output.sensorMb_y;
                    newRestults.Time.Add(state.step * outerTimeStep);
                    newRestults.SurfaceRPM.Add(output.omega_td);
                    newRestults.BitRPM.Add(output.omega_b);
                    newRestults.BitDepth.Add(state.BitDepth);
                    newRestults.HoleDepth.Add(state.HoleDepth);
                    newRestults.SurfaceTorque.Add(u.tau_Motor);
                    newRestults.BitTorque.Add(output.tob);
                    newRestults.TopOfStringAxialVelocity.Add(u.v0);
                    newRestults.BitAxialVelocity.Add(output.vb);
                    newRestults.WOB.Add(output.wob);
                    newRestults.SSI.Add(output.SSI);
                    newRestults.AvgCumulativeSSI = output.average_cumulative_ssi;
                    newRestults.SensorAngularVelocity.Add(output.sensorAngularVelocity);
                    newRestults.SensorWhirlVelocity.Add(output.sensorWhirlSpeed);
                    newRestults.SensorAxialVelocity.Add(output.sensorAxialVelocity);
                    newRestults.SensorRadialVelocity.Add(output.sensorRadialSpeed);
                    newRestults.SensorRadialAcc.Add(output.sensorRadialAccelerationLocalFrame);
                    newRestults.SensorTangentialAcc.Add(output.sensorTangentialAccelerationLocalFrame);
                    newRestults.SensorAxialAcc.Add(output.sensorAxialAccelerationLocalFrame);
                    newRestults.SensorBendingMomentX.Add(output.sensorBendingMomentX);
                    newRestults.SensorBendingMomentY.Add(output.sensorBendingMomentY);

                    // depth based values
                    newRestults.SideForce = Utilities.ExtendVectorStart(0, output.F_N).ToList();
                    newRestults.SideForceSoftString = Utilities.ExtendVectorStart(0, output.F_N_softstring).ToList();
                    newRestults.Depth = output.depths.ToList();

                    if (!config.UseMudMotor)
                        newRestults.PipeAngularVelocity = Utilities.ExtendVectorStart(state.Otd, state.OL).Append(output.omega_b).ToList();
                    else
                    {
                        var OLMinusLast = state.OL.SubVector(1, state.OL.Count - 1);
                        newRestults.PipeAngularVelocity = Utilities.ExtendVectorStart(state.Otd, Utilities.ToVector(OLMinusLast.Append(output.omega_b).ToArray())).ToList();
                    }
                    newRestults.SleevesAngularVelocity = state.OS.ToList();
                    var SleevesDepth = parameters.ds.iS.Select(index => parameters.lc.xL[(int)index]).ToArray();
                    newRestults.SleevesDepth = SleevesDepth.ToList();
                    newRestults.RadialClearance = parameters.w.rc.Append(0).ToList();
                    newRestults.LateralDisplacement = output.rc.Append(0).ToList();
                    newRestults.BendingMoment = output.Mb.Append(0).ToList();
                    newRestults.DepthAll = parameters.dc.x.ToList();
                    newRestults.Torque = output.torque.ToList();
                    newRestults.Tension = output.tension.ToList();
                    newRestults.AxialVelocityD = Utilities.ExtendVectorStart(u.v0, state.VL).ToList();
                    newRestults.LateralDisplacementAngle = output.phi.Append(0).ToList();

                    simulation.Results = newRestults;

                    simulation.Progress = (double)state.step * outerTimeStep / totalDuration;


                    // Avoid blocking UI or database thread with repeated updates; optionally batch this
                    await Task.Run(() => UpdateProgressSimulationById(simulation.MetaInfo.ID, simulation));
                    RunningSimulations[simulation.MetaInfo.ID] = simulation;
                }
            }

            simulation.TerminationState = 1;
            return true;
        }


    }

}