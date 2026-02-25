# Simulator4nDOF

The Simulator4nDOF repository hosts a microservice and client webapp for Simulator4nDOF.

# Simulator4nDOF - Model definition

Every model is defined as a variation of IModel< ModelType > interaface. In this case, each IModel must contain a definition of the treatment in case of addition of a lumped parameter and a definition of how the accelerations/forces are calculated.

An ODE solver option requires a model input. It is when the accelerations are calculated. This choice was made to facilitate the addition of new solvers/models.

# Solution architecture

The solution is composed of:
- **ModelSharedIn**
  - contains C# auto-generated classes of Model dependencies
  - these dependencies are stored as json files (following the OpenApi standard) and C# classes are generated on execution of the program
  - *dependencies* = some external microservices (OpenApi schemas in json format)
- **Model**
  - defines the main classes and methods to run the microservice
  - *dependencies* = BaseModels
- **Service**
  - defines the proper microservice API
  - *dependencies* = Model
- **ModelSharedOut**
  - contains C# auto-generated classes for microservice clients dependencies
  - these dependencies are stored as json files (following the OpenAPI standard) and C# classes are generated on execution of the program
  - these dependencies include the OpenApi schema of the microservice itself as well as other dependencies that may be useful to run the microservice
  - *dependencies* = Simulator4nDOF.json + some external microservices (OpenApi schemas in json format)
- **ModelTest**
  - performs unit tests on the Model (in particular for base computations)
  - *dependencies* = Model
- **ServiceTest**
  - microservice client that performs unit tests on the microservice (by default, an instance of the microservice must be running on http port 8080 to run tests)
  - *dependencies* = ModelShared
- **WebApp**
  - microservice web app client that manages data associated with Simulator4nDOF and allow to interact with the microservice
  - *dependencies* = ModelShared
- **home** (auto-generated)
  - data are persisted in the microservice container using the Sqlite database located at *home/Simulator4nDOF.db*

# Security/Confidentiality

Data are persisted as clear text in a unique Sqlite database hosted in the docker container.
Neither authentication nor authorization have been implemented.
Would you like or need to protect your data, docker containers of the microservice and webapp are available on dockerhub, under the digiwells organization, at:

https://hub.docker.com/?namespace=digiwells

More info on how to run the container and map its database to a folder on your computer, at:

https://github.com/NORCE-DrillingAndWells/DrillingAndWells/wiki

# Deployment

Microservice is available at:

https://dev.digiwells.no/Simulator4nDOF/api/Simulation

https://app.digiwells.no/Simulator4nDOF/api/Simulation

Web app is available at:

https://dev.digiwells.no/Simulator4nDOF/webapp/Simulation

https://app.digiwells.no/Simulator4nDOF/webapp/Simulation

The OpenApi schema of the microservice is available and testable at:

https://dev.digiwells.no/Simulator4nDOF/swagger (development server) 

https://app.digiwells.no/Simulator4nDOF/swagger (production server)

The microservice and webapp are deployed as Docker containers using Kubernetes and Helm. More info at:

https://github.com/NORCE-DrillingAndWells/DrillingAndWells/wiki

# Funding

The current work has been funded by the [Research Council of Norway](https://www.forskningsradet.no/) and [Industry partners](https://www.digiwells.no/about/board/) in the framework of the cent for research-based innovation [SFI Digiwells (2020-2028)](https://www.digiwells.no/) focused on Digitalization, Drilling Engineering and GeoSteering. 

# Contributors

**Sonja Moi**, *NORCE Energy Modelling and Automation*

**Eric Cayeux**, *NORCE Energy Modelling and Automation*
