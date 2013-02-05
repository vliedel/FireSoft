#!/bin/sh

# Stop server and modules
#killall yarpserver

#pgrep -n -f sim/build/main
#killall sim/build/main
#killall autoPilotSim/build/main
#killall radioSim/build/main
#killall mapSelf/build/main
#killall mapUAVs/build/main
#killall msgPlanner/build/main

pkill -f sim/build/main
pkill -f autoPilot/build/main
pkill -f autoPilotSim/build/main
pkill -f mapFitness/build/main
pkill -f mapSelf/build/main
pkill -f mapUAVs/build/main
pkill -f fitnessGenBattery/build/main
pkill -f fitnessGenCollision/build/main
pkill -f fitnessGenCoverage/build/main
pkill -f fitnessGenStatic/build/main
pkill -f groundStationSim/build/main
pkill -f gsGuiInterface/build/main
pkill -f gsVisualizer/build/main
pkill -f msgPlanner/build/main
pkill -f radio/build/main
pkill -f radioSim/build/main
pkill -f wpPlanner/build/main
