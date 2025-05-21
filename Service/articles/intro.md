---
title: "How to use the Simulator4nDOF microservice?"
output: html_document
---

Typical Usage
===
1. Upload a new Simulation using the `Post` web api method.
2. Call the `Get` method with the identifier of the uploaded Simulation as argument. 
The return Json object contains the Simulation description.
3. Optionally send a `Delete` request with the identifier of the Simulation in order to delete the Simulation if you do not 
want to keep the Simulation uploaded on the microservice.


