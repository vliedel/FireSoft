#!/bin/sh

# Usage: ./start_sim.sh <total_uavs> <num_uavs_with_sim_auto_pilot> <num_uavs_with_sim_radio> <max_sim_time> <start_ground_station? 0|1|2>
NUM=10
if [ $1 ]; then
	NUM=$1
fi

NUM_AP=$NUM
if [ $2 ]; then
	NUM_AP=$2
fi

NUM_RADIO=$NUM_AP
if [ $3 ]; then
	NUM_RADIO=$3
fi

SIM_TIME=120
if [ $4 ]; then
	SIM_TIME=$4
fi

GS_START="1"
if [ $5 -a $5 = "0" ]; then
	GS_START="0"
fi
if [ $5 -a $5 = "2" ]; then
	GS_START="2"
fi

# Start the central simulator
sim/build/main 0 $NUM $NUM_AP $NUM_RADIO $SIM_TIME $GS_START > output/output_sim &

# Wait for the simulator to be started
sleep 10

# Connect simout to simulator
yarp connect /simout /sim0/command

