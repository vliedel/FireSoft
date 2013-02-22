#!/bin/sh

# Usage: ./start_gs.sh <start_ground_station? 0|1|2> <gs_id>

GS_START="1"
if [ $1 -a $1 = "0" ]; then
	GS_START="0"
fi
if [ $1 -a $1 = "2" ]; then
	GS_START="2"
fi

GS_ID=10
if [ $2 ]; then
	GS_ID=$2
fi

if [ $GS_START = "1" ]; then
	# Start the simulated ground station with id GS_ID
	groundStationSim/build/main $GS_ID > output/output_gs &
	YARP_NAME=groundstationsim${GS_ID}
fi
if [ $GS_START = "2" ]; then
	# Start the real ground station with id GS_ID
	groundStation/build/main $GS_ID > output/output_gs &
	YARP_NAME=groundstation${GS_ID}
fi
if [ $GS_START != "0" ]; then
	# Start common ground station modules
	mapUAVs/build/main $GS_ID > output/output_mapuavs_${GS_ID} &
	mapFire/build/main $GS_ID > output/output_mapfire_${GS_ID} &
	gsGuiInterface/build/main $GS_ID > output/output_gsGuiInterface &
fi

# Modules that need to read shared memory need to wait a bit until shared memory is created
sleep 5

if [ $GS_START != "0" ]; then
	gsVisualizer/build/main $GS_ID > output/output_gsVis &
fi

# Wait for all modules to be ready
sleep 5

# Connect the modules, assume simulator is ready
if [ $GS_START = "1" ]; then
	# Connect ground station to simulator
	yarp connect /sim0/groundstation /${YARP_NAME}/sim
	yarp connect /${YARP_NAME}/tosim /sim0/fromgroundstation
fi

if [ $GS_START != "0" ]; then
	# Connect modules within simulated ground station
	yarp connect /${YARP_NAME}/tomapuavs /mapuavs${GS_ID}/fromradio
	yarp connect /${YARP_NAME}/tomapfire /mapfire${GS_ID}/fromradio
	yarp connect /${YARP_NAME}/toguiinterface /gsguiinterface${GS_ID}/fromradio
	yarp connect /gsguiinterface${GS_ID}/toradio /${YARP_NAME}/fromguiinterface
fi

