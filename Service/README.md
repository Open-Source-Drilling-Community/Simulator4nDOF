# Simulator4nDOF microservice

The Simulator4nDOF microservice is packaged as a docker container named:

``norcedrillingsimulator4ndofservice``

It is available on dockerhub, under the digiwells organization, at:

https://hub.docker.com/?namespace=digiwells

The API (OpenApi schema) of the microservice is available and testable at:

https://dev.digiwells.no/Simulator4nDOF/api/swagger (development server) 

https://app.digiwells.no/Simulator4nDOF/api/swagger (production server)

The microservice itself is available at:

https://dev.digiwells.no/Simulator4nDOF/api/Simulation

https://app.digiwells.no/Simulator4nDOF/api/Simulation

When simulation contextual data does not explicitly select a Rig, the resolver uses the latest chronological RigJob on the selected WellBore. An empty RigJob history is authoritative and reports that no rig is available; only a null legacy history falls back to the deprecated WellBore `RigID` and then the Well/Cluster association.

# Funding

The current work has been funded by the [Research Council of Norway](https://www.forskningsradet.no/) and [Industry partners](https://www.digiwells.no/about/board/) in the framework of the cent for research-based innovation [SFI Digiwells (2020-2028)](https://www.digiwells.no/) focused on Digitalization, Drilling Engineering and GeoSteering. 

# Contributors

**Sonja Moi**, *NORCE Energy Modelling and Automation*

**Eric Cayeux**, *NORCE Energy Modelling and Automation*

## Calculation lifecycle semantics

OpenAPI publishes SemanticCatalogue 0.15.0 metadata for the queued simulation lifecycle: case retrieval, light status retrieval, queued submission/replacement, progress and state fields, and paged server-derived results. The existing MCP host currently exposes no simulation calculation tool, so there is no parallel calculation MCP contract to annotate.



