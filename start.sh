#!/bin/sh

# Usage: ./start.sh <total_uavs> <num_uavs_with_sim_auto_pilot> <num_uavs_with_sim_radio> <max_sim_time> <first_uav_is_hil? 1|0> <start_ground_station? 0|1|2>
GS_ID=10

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

SIM_TIME=600
if [ $4 ]; then
	SIM_TIME=$4
fi

HIL="0"
if [ $5 -a $5 = "1" ]; then
	HIL="1"
fi

GS_START="1"
if [ $6 -a $6 = "0" ]; then
	GS_START="0"
fi
if [ $6 -a $6 = "2" ]; then
	GS_START="2"
fi

# Start the central simulator
./start_sim.sh $NUM $NUM_AP $NUM_RADIO $SIM_TIME $GS_START

# Start the ground station
./start_gs.sh $GS_START $GS_ID

# If there is hardware in the loop, don't start its modules on this pc
i=$HIL

# Start the UAVs
while [ $i -lt $NUM ]
do
	REAL_AP="0"
	REAL_RADIO="0"
	
	# If there are UAVs with a real autopilot, start autoPilot instead of autoPilotSim
	if [ $NUM_AP -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_AP + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_AP] ]; then
			REAL_AP="1"
		fi
	fi
	
	# If there are UAVs with a real radio, start radio instead of radioSim module
	if [ $NUM_RADIO -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_RADIO + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_RADIO] ]; then
			REAL_RADIO="1"
		fi
	fi

	./start_uav.sh $i $REAL_AP $REAL_RADIO

	i=$[$i+1]
done

