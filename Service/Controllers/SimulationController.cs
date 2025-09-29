using System;
using System.Collections.Generic;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging;
using OSDC.DotnetLibraries.General.DataManagement;
using NORCE.Drilling.Simulator4nDOF.Service.Managers;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Service.Controllers
{
    [Produces("application/json")]
    [Route("[controller]")]
    [ApiController]
    public class SimulationController : ControllerBase
    {
        private readonly ILogger<SimulationManager> _logger;
        private readonly SimulationManager _simulationManager;

        public SimulationController(ILogger<SimulationManager> logger, SqlConnectionManager connectionManager)
        {
            _logger = logger;
            _simulationManager = SimulationManager.GetInstance(_logger, connectionManager);
        }

        /// <summary>
        /// Returns the list of Guid of all Simulation present in the microservice database at endpoint Simulator4nDOF/api/Simulation
        /// </summary>
        /// <returns>the list of Guid of all Simulation present in the microservice database at endpoint Simulator4nDOF/api/Simulation</returns>
        [HttpGet(Name = "GetAllSimulationId")]
        public ActionResult<IEnumerable<Guid>> GetAllSimulationId()
        {
            var ids = _simulationManager.GetAllSimulationId();
            if (ids != null)
            {
                return Ok(ids);
            }
            else
            {
                return StatusCode(StatusCodes.Status500InternalServerError);
            }
        }

        /// <summary>
        /// Returns the list of MetaInfo of all Simulation present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/MetaInfo
        /// </summary>
        /// <returns>the list of MetaInfo of all Simulation present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/MetaInfo</returns>
        [HttpGet("MetaInfo", Name = "GetAllSimulationMetaInfo")]
        public ActionResult<IEnumerable<MetaInfo>> GetAllSimulationMetaInfo()
        {
            var vals = _simulationManager.GetAllSimulationMetaInfo();
            if (vals != null)
            {
                return Ok(vals);
            }
            else
            {
                return StatusCode(StatusCodes.Status500InternalServerError);
            }
        }


        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Light/id
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Light/id</returns>
        [HttpGet("Light/{id}\"", Name = "GetSimulatioLightById")]
        public ActionResult<Model.SimulationLight?> GetSimulatioLightById(Guid id)
        {
            if (!id.Equals(Guid.Empty))
            {
                var val = _simulationManager.GetSimulationLightById(id);
                if (val != null)
                {
                    return Ok(val);
                }
                else
                {
                    return NotFound();
                }
            }
            else
            {
                return BadRequest();
            }
        }

        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Heavy/id
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Heavy/id</returns>
        [HttpGet("Heavy/{id}\"", Name = "GetSimulationById")]
        public ActionResult<Model.Simulation?> GetSimulationById(Guid id)
        {
            if (!id.Equals(Guid.Empty))
            {
                var val = _simulationManager.GetSimulationById(id);
                if (val != null)
                {
                    return Ok(val);
                }
                else
                {
                    return NotFound();
                }
            }
            else
            {
                return BadRequest();
            }
        }

        /// <summary>
        /// Returns the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Results/id/page/sequenceNumber
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>the Simulation identified by its Guid from the microservice database, at endpoint Simulator4nDOF/api/Simulation/Results/id/page/sequenceNumber</returns>
        [HttpGet("Results/{id}/page/{sequenceNumber}\"", Name = "GetSimulationResultById")]
        public ActionResult<Model.Results?> GetSimulationResultById(Guid id, uint sequenceNumber)
        {
            if (!id.Equals(Guid.Empty))
            {
                var val = _simulationManager.GetSimulationResultById(id, sequenceNumber);
                if (val != null)
                {
                    return Ok(val);
                }
                else
                {
                    return NotFound();
                }
            }
            else
            {
                return BadRequest();
            }
        }


        /// <summary>
        /// Returns the list of all SimulationLight present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/LightData
        /// </summary>
        /// <returns>the list of all SimulationLight present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/LightData</returns>
        [HttpGet("LightData", Name = "GetAllSimulationLight")]
        public ActionResult<IEnumerable<Model.SimulationLight>> GetAllSimulationLight()
        {
            var vals = _simulationManager.GetAllSimulationLight();
            if (vals != null)
            {
                return Ok(vals);
            }
            else
            {
                return StatusCode(StatusCodes.Status500InternalServerError);
            }
        }

        /// <summary>
        /// Returns the list of all Simulation present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/HeavyData
        /// </summary>
        /// <returns>the list of all Simulation present in the microservice database, at endpoint Simulator4nDOF/api/Simulation/HeavyData</returns>
        [HttpGet("HeavyData", Name = "GetAllSimulation")]
        public ActionResult<IEnumerable<Model.Simulation?>> GetAllSimulation()
        {
            var vals = _simulationManager.GetAllSimulation();
            if (vals != null)
            {
                return Ok(vals);
            }
            else
            {
                return StatusCode(StatusCodes.Status500InternalServerError);
            }
        }

        /// <summary>
        /// Performs calculation on the given Simulation and adds it to the microservice database, at the endpoint Simulator4nDOF/api/Simulation
        /// </summary>
        /// <param name="simulation"></param>
        /// <returns>true if the given Simulation has been added successfully to the microservice database, at the endpoint Simulator4nDOF/api/Simulation</returns>
        [HttpPost(Name = "PostSimulation")]
        public async Task<ActionResult> PostSimulation([FromBody] Model.Simulation? data)
        {
            // Check if simulation exists in the database through ID
            if (data != null && data.MetaInfo != null && data.MetaInfo.ID != Guid.Empty)
            {
                var existingData = _simulationManager.GetSimulationById(data.MetaInfo.ID);
                if (existingData == null)
                {   
                    //  If simulation was not found, call AddSimulation, where the simulation.Calculate()
                    // method is called. 
                    if (await _simulationManager.AddSimulation(data))
                    {
                        return Ok(); // status=OK is used rather than status=Created because NSwag auto-generated controllers use 200 (OK) rather than 201 (Created) as return codes
                    }
                    else
                    {
                        return StatusCode(StatusCodes.Status500InternalServerError);
                    }
                }
                else
                {
                    _logger.LogWarning("The given Simulation already exists and will not be added");
                    return StatusCode(StatusCodes.Status409Conflict);
                }
            }
            else
            {
                _logger.LogWarning("The given Simulation is null, badly formed, or its ID is empty");
                return BadRequest();
            }
        }

        /// <summary>
        /// Performs calculation on the given Simulation and updates it in the microservice database, at the endpoint Simulator4nDOF/api/Simulation/id
        /// </summary>
        /// <param name="simulation"></param>
        /// <returns>true if the given Simulation has been updated successfully to the microservice database, at the endpoint Simulator4nDOF/api/Simulation/id</returns>
        [HttpPut("{id}", Name = "PutSimulationById")]
        public async Task<ActionResult> PutSimulationById(Guid id, [FromBody] Model.Simulation? data)
        {
            // Check if Simulation is in the data base
            if (data != null && data.MetaInfo != null && data.MetaInfo.ID.Equals(id))
            {
                var existingData = _simulationManager.GetSimulationById(id);

                if (existingData != null)
                {
                    if (_simulationManager.DeleteSimulationById(id))
                    {
                        if (await _simulationManager.AddSimulation(data))
                        {
                            return Ok();
                        }
                        else
                        {
                            return StatusCode(StatusCodes.Status500InternalServerError);
                        }
                    } else
                    {
                        _logger.LogWarning("The simulation could not be deleted from the database before posting it again");
                        return StatusCode(StatusCodes.Status500InternalServerError);
                    }
                }
                else
                {
                    _logger.LogWarning("The given Simulation has not been found in the database");
                    return NotFound();
                }
            }
            else
            {
                _logger.LogWarning("The given Simulation is null, badly formed, or its does not match the ID to update");
                return BadRequest();
            }
        }

        /// <summary>
        /// Deletes the Simulation of given ID from the microservice database, at the endpoint Simulator4nDOF/api/Simulation/id
        /// </summary>
        /// <param name="guid"></param>
        /// <returns>true if the Simulation was deleted from the microservice database, at the endpoint Simulator4nDOF/api/Simulation/id</returns>
        [HttpDelete("{id}", Name = "DeleteSimulationById")]
        public ActionResult DeleteSimulationById(Guid id)
        {
            if (_simulationManager.DeleteSimulationById(id))
            {
                return Ok();
            }
            else
            {
                _logger.LogWarning("The simulation could not be deleted from the database");
                return StatusCode(StatusCodes.Status500InternalServerError);
            }
        }
    }
}
