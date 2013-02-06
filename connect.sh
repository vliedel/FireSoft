#!/bin/sh

# Connect simout to simulator
yarp connect /simout /sim0/command

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

# Connect ground station to simulator
yarp connect /sim0/groundstation /groundstationsim${GS_ID}/sim
yarp connect /groundstationsim${GS_ID}/tosim /sim0/fromgroundstation

# Connect modules within ground station
yarp connect /groundstationsim${GS_ID}/tomapuavs /mapuavs${GS_ID}/fromradio
yarp connect /groundstationsim${GS_ID}/toguiinterface /gsguiinterface${GS_ID}/fromradio
yarp connect /gsguiinterface${GS_ID}/toradio /groundstationsim${GS_ID}/fromguiinterface

# Connect simulator to UAVs
i="0"
while [ $i -lt $NUM ]
do
	# If there are UAVs with a real autopilot, don't try to connect to it
	if [ $NUM_AP -lt $NUM_RADIO ]; then
		if [ $i -lt $[$NUM_RADIO - $NUM_AP] ]; then
			echo "Not connecting auto pilot ${i} to sim"
		else
			yarp connect /sim0/autopilotcommand${i} /autopilotsim${i}/simcommand
			yarp connect /autopilotsim${i}/simstate /sim0/autopilotstate${i}
		fi
	else
		yarp connect /sim0/autopilotcommand${i} /autopilotsim${i}/simcommand
		yarp connect /autopilotsim${i}/simstate /sim0/autopilotstate${i}
	fi
	
	# If there are UAVs with a real radio, don't try to connect to it
	if [ $NUM_RADIO -lt $NUM_AP ]; then
		if [ $i -lt $[$NUM_AP - $NUM_RADIO + $HIL] ]; then
			echo "Not connecting radio ${i} to sim"
		else
			yarp connect /sim0/radiocommand${i} /radiosim${i}/simcommand
			yarp connect /radiosim${i}/simstate /sim0/radiostate${i}
		fi
	else
		yarp connect /sim0/radiocommand${i} /radiosim${i}/simcommand
		yarp connect /radiosim${i}/simstate /sim0/radiostate${i}
	fi
	i=$[$i+1]
done


# Connect modules within UAVs
i="0"
while [ $i -lt $NUM ]
do
	if [ $NUM_AP -lt $NUM_RADIO ]; then
		if [ $i -lt $[$NUM_RADIO - $NUM_AP] ]; then
			# Connect real auto pilot
			yarp connect /autopilot${i}/mapself /mapself${i}/autopilot
			yarp connect /wpplanner${i}/toautopilot /autopilot${i}/fromwaypointplanner
		else
			# Connect sim auto pilot
			yarp connect /autopilotsim${i}/mapself /mapself${i}/autopilot
			yarp connect /wpplanner${i}/toautopilot /autopilotsim${i}/fromwaypointplanner
		fi
	else
		# Connect sim auto pilot
		yarp connect /autopilotsim${i}/mapself /mapself${i}/autopilot
		yarp connect /wpplanner${i}/toautopilot /autopilotsim${i}/fromwaypointplanner
	fi
	
	if [ $NUM_RADIO -lt $NUM_AP ]; then
		if [ $i -lt $[$NUM_AP - $NUM_RADIO + $HIL] ]; then
			# Connect real radio
			yarp connect /radio${i}/tomapself /mapself${i}/fromradio
			yarp connect /radio${i}/tomapuavs /mapuavs${i}/fromradio
			yarp connect /mapuavs${i}/toradio /radio${i}/frommapuavs
			yarp connect /radio${i}/tomsgplanner /msgplanner${i}/fromradio
			yarp connect /msgplanner${i}/toradio /radio${i}/frommsgplanner
		else
			# Connect sim radio
			yarp connect /radiosim${i}/tomapself /mapself${i}/fromradio
			yarp connect /radiosim${i}/tomapuavs /mapuavs${i}/fromradio
			yarp connect /mapuavs${i}/toradio /radiosim${i}/frommapuavs
			yarp connect /radiosim${i}/tomsgplanner /msgplanner${i}/fromradio
			yarp connect /msgplanner${i}/toradio /radiosim${i}/frommsgplanner
		fi
	else
		# Connect sim radio
		yarp connect /radiosim${i}/tomapself /mapself${i}/fromradio
		yarp connect /radiosim${i}/tomapuavs /mapuavs${i}/fromradio
		yarp connect /mapuavs${i}/toradio /radiosim${i}/frommapuavs
		yarp connect /radiosim${i}/tomsgplanner /msgplanner${i}/fromradio
		yarp connect /msgplanner${i}/toradio /radiosim${i}/frommsgplanner
	fi
	i=$[$i+1]
done
