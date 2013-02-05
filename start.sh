#!/bin/sh

# Start the yarp server
#yarpserver &
#sleep 1

# Start the modules

GS_ID=10
NUM_AP=10
if [ $1 ]
then
        NUM_AP=$1
fi

NUM_RADIO=$NUM_AP
if [ $2 ]
then
        NUM_RADIO=$2
fi

if [ $NUM_RADIO -lt $NUM_AP ]
then
	NUM=$NUM_AP
else
	NUM=$NUM_RADIO
fi

SIM_TIME=120
if [ $3 ]
then
        SIM_TIME=$3
fi

# Start the central simulator
sim/build/main 0 ${NUM_AP} ${NUM_RADIO} ${SIM_TIME} > output/output_sim &

# Start the simulated ground station with id GS_ID
groundStationSim/build/main $GS_ID > output/output_gs &
mapUAVs/build/main $GS_ID > output/output_mapuavs_${GS_ID} &
gsGuiInterface/build/main $GS_ID > output/output_gsGuiInterface &

# If there is hardware in the loop, don't start its modules on this pc
i="0"
if [ $4 ]
then
	i="1"
	HIL="1"
else
	HIL="0"
fi

# Start modules
while [ $i -lt $NUM ]
do
	mapSelf/build/main $i > output/output_mapself_${i} &
	mapUAVs/build/main $i > output/output_mapuavs_${i} &
	mapFitness/build/main $i > output/output_mapfitness_${i} &
	
	# If there are UAVs with a real autopilot, start autoPilot instead of autoPilotSim
	if [ $NUM_AP -lt $NUM_RADIO ]; then
		if [ $i -lt $[$NUM_RADIO - $NUM_AP + $HIL] ]; then
			autoPilot/build/main $i > output/output_ap_${i} &
		else
			autoPilotSim/build/main $i > output/output_ap_${i} &
		fi
	else
		autoPilotSim/build/main $i > output/output_ap_${i} &
	fi
	
	# If there are UAVs with a real radio, start radio instead of radioSim module
	if [ $NUM_RADIO -lt $NUM_AP ]; then
		if [ $i -lt $[$NUM_AP - $NUM_RADIO + $HIL] ]; then
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
sleep $[15 + NUM * 10]

gsVisualizer/build/main $GS_ID > output/output_gsVis &

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

