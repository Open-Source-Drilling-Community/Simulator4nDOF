using Microsoft.Data.Sqlite;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using System;
using System.Threading;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Service.Managers
{
    public class DatabaseCleanerService : BackgroundService
    {
        private readonly ILogger<DatabaseCleanerService> _logger;
        private readonly SqlConnectionManager _connectionManager;
        private readonly string[] _dataTables = ["SimulationTable"];
        private static readonly TimeSpan _cleaningInterval = TimeSpan.FromDays(1);
        private static readonly TimeSpan _retirementTime = TimeSpan.FromDays(90);

        public DatabaseCleanerService(ILogger<DatabaseCleanerService> logger, SqlConnectionManager connectionManager)
        {
            _logger = logger;
            _connectionManager = connectionManager;
        }

        protected override async Task ExecuteAsync(CancellationToken stoppingToken)
        {
            _logger.LogInformation("DatabaseCleanerService is starting.");

            while (!stoppingToken.IsCancellationRequested)
            {
                try
                {
                    foreach (string table in _dataTables)
                    {
                        if (!CleanDatabase(table))
                            _logger.LogError("{table} was not cleaned successfully at time {time}", table, DateTimeOffset.UtcNow);
                    }
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "An error occurred while cleaning the database");
                }
                await Task.Delay(_cleaningInterval, stoppingToken);
            }
            _logger.LogInformation("DatabaseCleanerService is stopping.");
        }

        /// <summary>
        /// Removes old records from database table
        ///     - records are removed if LastModificationDate is null or older that DateTimeOffset.UtcNow-CleaningInterval
        ///     - CreationDate is not inspected
        /// </summary>
        /// <returns>true if older ToBeRemoved were successfully deleted</returns>
        private bool CleanDatabase(string dataTable)
        {
            var connection = _connectionManager.GetConnection();
            if (connection != null)
            {
                try
                {
                    var command = connection.CreateCommand();
                    _logger.LogInformation("Looking in {table} for records older than {timeSpan} days, at time {time}", dataTable, _retirementTime.TotalDays, DateTimeOffset.UtcNow);
                    string formattedUtcNowRebase = (DateTimeOffset.UtcNow - _retirementTime).ToString(SqlConnectionManager.DATE_TIME_FORMAT);
                    command.CommandText = $"DELETE FROM {dataTable} WHERE LastModificationDate < '{formattedUtcNowRebase}'";
                    int count = command.ExecuteNonQuery();
                    if (count < 0)
                    {
                        _logger.LogWarning("Impossible to delete the data from {table} at time {time}", dataTable, DateTimeOffset.UtcNow);
                    }
                    else if (count > 0)
                    {
                        _logger.LogInformation("{count} records removed successfully from {table} at time {time}", count, dataTable, DateTimeOffset.UtcNow);
                    }
                    return true;
                }
                catch (SqliteException ex)
                {
                    _logger.LogError(ex, "Impossible to clean old ToBeRemoved from {table}", dataTable);
                }
            }
            else
            {
                _logger.LogWarning("Impossible to access the SQLite database");
            }
            return false;
        }
    }
}
