#!/bin/sh

# Usage: ./start.sh <total_uavs> <num_uavs_with_sim_auto_pilot> <num_uavs_with_sim_radio> <max_sim_time> <first_uav_is_hil? 1|0> <start_ground_station? 1|0>

# Start the modules

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

SIM_TIME=120
if [ $4 ]; then
	SIM_TIME=$4
fi

if [ $5 -a $5 = "1" ]; then
	HIL="1"
else
	HIL="0"
fi

if [ $6 -a $6 = "0" ]; then
	GS_START="0"
else
	GS_START="1"
fi

# Start the central simulator
sim/build/main 0 $NUM $NUM_AP $NUM_RADIO $SIM_TIME $GS_START > output/output_sim &

if [ $GS_START = "1" ]; then
	# Start the simulated ground station with id GS_ID
	groundStationSim/build/main $GS_ID > output/output_gs &
	mapUAVs/build/main $GS_ID > output/output_mapuavs_${GS_ID} &
	gsGuiInterface/build/main $GS_ID > output/output_gsGuiInterface &
fi

# If there is hardware in the loop, don't start its modules on this pc
i=$HIL


# Start modules
while [ $i -lt $NUM ]
do
	mapSelf/build/main $i > output/output_mapself_${i} &
	mapUAVs/build/main $i > output/output_mapuavs_${i} &
	mapFitness/build/main $i > output/output_mapfitness_${i} &
	mapFire/build/main $i > output/output_mapfire_${i} &
	
	# If there are UAVs with a real autopilot, start autoPilot instead of autoPilotSim
	if [ $NUM_AP -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_AP + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_AP] ]; then
			autoPilot/build/main $i > output/output_ap_${i} &
		else
			autoPilotSim/build/main $i > output/output_ap_${i} &
		fi
	else
		autoPilotSim/build/main $i > output/output_ap_${i} &
	fi
	
	# If there are UAVs with a real radio, start radio instead of radioSim module
	if [ $NUM_RADIO -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_RADIO + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_RADIO] ]; then
			radio/build/main $i > output/output_radio_${i} &
		else
			radioSim/build/main $i > output/output_radio_${i} &
		fi
	else
		radioSim/build/main $i > output/output_radio_${i} &
	fi
	

	i=$[$i+1]
done


# Modules that need to read shared memory need to wait a bit until shared memory is created
sleep $[15 + NUM * 12]

if [ $GS_START = "1" ]; then
	gsVisualizer/build/main $GS_ID > output/output_gsVis &
fi

i=$HIL
while [ $i -lt $NUM ]
do
	msgPlanner/build/main $i > output/output_msgplan_${i} &
	wpPlanner/build/main $i > output/output_wpplan_${i} &
	fitnessGenBattery/build/main $i > output/output_fgenBatt_${i} &
	fitnessGenCollision/build/main $i > output/output_fgenCol_${i} &
	fitnessGenCoverage/build/main $i > output/output_fgenCov_${i} &
	fitnessGenStatic/build/main $i > output/output_fgenStat_${i} &

	i=$[$i+1]
done

