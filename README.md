# FireSoft

FireSwarm software for on the Gumstix. Part of the FireSwarm project: http://www.fireswarm.nl

## Requirements
This software uses libraries:
* YARP
* Boost
* Eigen

Next to those libraries, it makes use of
* rur-builder
* aimtools

## Compilation
To compile simply type:
* make

## Running
* Start a yarp server: yarp server
* Start a yarp write: yarp write /simout
* Start modules: ./start.sh
* Type 1<return> in the yarp write to start the simulation.
* Stop all modules: ./stop.sh

## Copyrights
The copyrights (2012) belong to:

- Author: Bart van Vliet
- Almende B.V., http://www.almende.com and DO bots B.V., http://www.dobots.nl
- Rotterdam, The Netherlands
