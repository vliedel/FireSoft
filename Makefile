#!/bin/make

# Author: Anne C. van Rossum
# Date: Aug. 04, 2011

all:
	cd serial && make
	cd autoPilot && make
	cd autoPilotSim && make
	cd comSalland && make
	cd fitnessGenBattery && make
	cd fitnessGenCollision && make
	cd fitnessGenCoverage && make
	cd fitnessGenStatic && make
	cd groundStation && make
	cd groundStationSim && make
	cd gsGuiInterface && make
	cd gsVisualizer && make
#	cd map && make
	cd mapFire && make
	cd mapFitness && make
	cd mapUAVs && make
	cd mapSelf && make
	cd msgPlanner && make
#	cd ping && make
#	cd pong && make
	cd radio && make
	cd radioSim && make
	cd sensorCO && make
	cd sim && make
	cd tests && make
	cd wpPlanner && make
	#cd detection && make olinux
	
debug:
	cd serial && make debug
	cd autoPilot && make debug
	cd autoPilotSim && make debug
	cd comSalland && make debug
	cd fitnessGenBattery && make debug
	cd fitnessGenCollision && make debug
	cd fitnessGenCoverage && make debug
	cd fitnessGenStatic && make debug
	cd groundStation && make debug
	cd groundStationSim && make debug
	cd gsGuiInterface && make debug
	cd gsVisualizer && make debug
#	cd map && make debug
	cd mapFire && make debug
	cd mapFitness && make debug
	cd mapUAVs && make debug
	cd mapSelf && make debug
	cd msgPlanner && make debug
#	cd ping && make debug
#	cd pong && make debug
	cd radio && make debug
	cd radioSim && make debug
	cd sensorCO && make debug
	cd sim && make debug
	cd tests && make debug
	cd wpPlanner && make debug
	
gumstix:
	cd serial && make gumstix
	cd autoPilot && make gumstix
	cd autoPilotSim && make gumstix
	cd comSalland && make gumstix
	cd fitnessGenBattery && make gumstix
	cd fitnessGenCollision && make gumstix
	cd fitnessGenCoverage && make gumstix
	cd fitnessGenStatic && make gumstix
	cd groundStation && make gumstix
	cd groundStationSim && make gumstix
	cd gsGuiInterface && make gumstix
	cd gsVisualizer && make gumstix
#	cd map && make gumstix
	cd mapFire && make gumstix
	cd mapFitness && make gumstix
	cd mapUAVs && make gumstix
	cd mapSelf && make gumstix
	cd msgPlanner && make gumstix
#	cd ping && make gumstix
#	cd pong && make gumstix
	cd radio && make gumstix
	cd radioSim && make gumstix
	cd sensorCO && make gumstix
	cd sim && make gumstix
	cd tests && make gumstix
	cd wpPlanner && make gumstix
	
clean:
	cd serial && make clean
	cd autoPilot && make clean
	cd autoPilotSim && make clean
	cd comSalland && make clean
	cd fitnessGenBattery && make clean
	cd fitnessGenCollision && make clean
	cd fitnessGenCoverage && make clean
	cd fitnessGenStatic && make clean
	cd groundStation && make clean
	cd groundStationSim && make clean
#	cd map && make clean
	cd mapFire && make clean
	cd mapFitness && make clean
	cd mapUAVs && make clean
	cd mapSelf && make clean
	cd msgPlanner && make clean
#	cd ping && make clean
#	cd pong && make clean
	cd radio && make clean
	cd radioSim && make clean
	cd sensorCO && make clean
	cd sim && make clean
	cd tests && make clean
	cd wpPlanner && make clean
	#cd detection && make clean

install:
	echo "Do not install!"
#	cd build && make install

uninstall:
	echo "Do not install, I said! I don't know how to uninstall..."
	
.PHONY: gumstix
