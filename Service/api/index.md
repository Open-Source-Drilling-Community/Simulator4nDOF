---
title: "Project: Microservice for Simulator4nDOF"
output: html_document
---

Objective
===
This project contains the microservice that allows to access the management of Simulation.


Principles
===
The microservice provides a web API and is containerized using Docker. The web api endpoint is `Simulator4nDOF/api/`. It is possible to upload new data 
(`Post` method), to modify already uploaded data (`Put` method), to delete uploaded data (`Delete` method). By calling the `Get` method, 
it is possible to obtain a list of all the identifiers of the children data that have been uploaded. It is also possible 
to call the `Get` method with the ID of an uploaded data.


