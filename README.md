# waypt-AWS-rover

This repository holds the source code for an obstacle avoiding waypoint rover, which uses inputs from a camera transmitting positioning data using AWS Greengrass. 

The rover is capable of attaining dynamic waypoints within a camera moniotored area. 

PID algorithms have been used to compute the shortest path to waypoints, avoid moving out of camera range and avoid obstacles. 

AWS Greengrass is used to set-up publisher-subscriber devices over MQTT to transmit positioning data to rover. 

The documentation is incomplete - please contact for further details.

