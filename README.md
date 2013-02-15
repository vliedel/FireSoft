# FireSoft

FireSwarm Software for on the Gumstix

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
Start a yarp server:
* yarp server
Start a yarp write
* yarp write /simout
Start the modules:
./start
Once all modules started (this might take a while), connect the modules:
./connect
Then start the simulation by typing "1" and press return in the yarp write.

To stop all modules:
./stop

## Copyrights
The copyrights (2012) belong to:

- Author: Bart van Vliet
- Almende B.V., http://www.almende.com and DO bots B.V., http://www.dobots.nl
- Rotterdam, The Netherlands
