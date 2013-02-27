#!/bin/sh

ID=0

if [ $1 -a $1 = "h" ]; then
	echo "Usage: ${0} <id> <real_ap? 0|1> <real_radio? 0|1>"
	exit 0
fi

if [ $1 ]; then
	ID=$1
fi

REAL_AP="0"
if [ $2 -a $2 = "1" ]; then
	REAL_AP="1"
fi

REAL_RADIO="0"
if [ $3 -a $3 = "1" ]; then
	REAL_RADIO="1"
fi

# Start modules
mapSelf/build/main $ID > output/output_mapself_${ID} &
mapUAVs/build/main $ID > output/output_mapuavs_${ID} &
mapFitness/build/main $ID > output/output_mapfitness_${ID} &
mapFire/build/main $ID > output/output_mapfire_${ID} &
	
# If there are UAVs with a real autopilot, start autoPilot instead of autoPilotSim
if [ $REAL_AP = "1" ]; then
	autoPilot/build/main $ID > output/output_ap_${ID} &
	YARP_AP_NAME=autopilot${ID}
else
	autoPilotSim/build/main $ID > output/output_ap_${ID} &
	YARP_AP_NAME=autopilotsim${ID}
fi
	
# If there are UAVs with a real radio, start radio instead of radioSim module
if [ $REAL_RADIO = "1" ]; then
	radio/build/main $ID > output/output_radio_${ID} &
	YARP_RADIO_NAME=radio${ID}
else
	radioSim/build/main $ID > output/output_radio_${ID} &
	YARP_RADIO_NAME=radiosim${ID}
fi
	
# Modules that need to read shared memory need to wait a bit until shared memory is created
sleep 15

msgPlanner/build/main $ID > output/output_msgplan_${ID} &
wpPlanner/build/main $ID > output/output_wpplan_${ID} &
fitnessGenBattery/build/main $ID > output/output_fgenBatt_${ID} &
fitnessGenCollision/build/main $ID > output/output_fgenCol_${ID} &
fitnessGenCoverage/build/main $ID > output/output_fgenCov_${ID} &
fitnessGenStatic/build/main $ID > output/output_fgenStat_${ID} &

# Wait for all modules to be ready
sleep 15

# Connect modules, assume simulator is ready
if [ $REAL_AP = "0" ]; then
	yarp connect /sim0/autopilotcommand${ID} /${YARP_AP_NAME}/simcommand
	yarp connect /${YARP_AP_NAME}/simstate /sim0/autopilotstate${ID}
fi
yarp connect /${YARP_AP_NAME}/tomapself /mapself${ID}/fromautopilot
yarp connect /mapself${ID}/toautopilot /${YARP_AP_NAME}/frommapself
yarp connect /wpplanner${ID}/toautopilot /${YARP_AP_NAME}/fromwaypointplanner

if [ $REAL_RADIO = "0" ]; then
	yarp connect /sim0/radiocommand${ID} /${YARP_RADIO_NAME}/simcommand
	yarp connect /${YARP_RADIO_NAME}/simstate /sim0/radiostate${ID}
fi
yarp connect /${YARP_RADIO_NAME}/tomapself /mapself${ID}/fromradio
yarp connect /${YARP_RADIO_NAME}/tomapuavs /mapuavs${ID}/fromradio
yarp connect /mapuavs${ID}/toradio /${YARP_RADIO_NAME}/frommapuavs
yarp connect /${YARP_RADIO_NAME}/tomsgplanner /msgplanner${ID}/fromradio
yarp connect /msgplanner${ID}/toradio /${YARP_RADIO_NAME}/frommsgplanner
yarp connect /${YARP_RADIO_NAME}/tomapfire /mapfire${ID}/fromradio

