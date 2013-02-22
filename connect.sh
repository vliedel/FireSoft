#!/bin/sh

# Connect simout to simulator
yarp connect /simout /sim0/command

GS_ID=10

NUM=10
if [ $1 ]
then
	NUM=$1
fi

NUM_AP=$NUM
if [ $2 ]
then
	NUM_AP=$2
fi

NUM_RADIO=$NUM_AP
if [ $3 ]
then
	NUM_RADIO=$3
fi

HIL="0"
if [ $4 -a $4 = "1" ]; then
	HIL="1"
fi

GS_START="1"
if [ $5 -a $5 = "0" ]; then
	GS_START="0"
fi
if [ $5 -a $5 = "2" ]; then
	GS_START="2"
fi


if [ $GS_START = "1" ]; then
	# Connect ground station to simulator
	yarp connect /sim0/groundstation /groundstationsim${GS_ID}/sim
	yarp connect /groundstationsim${GS_ID}/tosim /sim0/fromgroundstation
	# Connect modules within simulated ground station
	yarp connect /groundstationsim${GS_ID}/tomapuavs /mapuavs${GS_ID}/fromradio
	yarp connect /groundstationsim${GS_ID}/tomapfire /mapfire${GS_ID}/fromradio
	yarp connect /groundstationsim${GS_ID}/toguiinterface /gsguiinterface${GS_ID}/fromradio
	yarp connect /gsguiinterface${GS_ID}/toradio /groundstationsim${GS_ID}/fromguiinterface
fi

if [ $GS_START = "2" ]; then
	# Connect modules within real ground station
	yarp connect /groundstation${GS_ID}/tomapuavs /mapuavs${GS_ID}/fromradio
	yarp connect /groundstation${GS_ID}/tomapfire /mapfire${GS_ID}/fromradio
	yarp connect /groundstation${GS_ID}/toguiinterface /gsguiinterface${GS_ID}/fromradio
	yarp connect /gsguiinterface${GS_ID}/toradio /groundstation${GS_ID}/fromguiinterface
fi

# Connect simulator to UAVs

# If there is hardware in the loop, then what?
#i=$HIL
i="0"

while [ $i -lt $NUM ]
do
	# If there are UAVs with a real autopilot, don't try to connect to it
	if [ $NUM_AP -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_AP + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_AP] ]; then
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
	if [ $NUM_RADIO -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_RADIO + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_RADIO] ]; then
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
#i=$HIL
i="0"
while [ $i -lt $NUM ]
do
	if [ $NUM_AP -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_AP + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_AP] ]; then
			# Connect real auto pilot
			yarp connect /autopilot${i}/tomapself /mapself${i}/fromautopilot
			yarp connect /mapself${i}/toautopilot /autopilot${i}/frommapself
			yarp connect /wpplanner${i}/toautopilot /autopilot${i}/fromwaypointplanner
		else
			# Connect sim auto pilot
			yarp connect /autopilotsim${i}/tomapself /mapself${i}/fromautopilot
			yarp connect /mapself${i}/toautopilot /autopilotsim${i}/frommapself
			yarp connect /wpplanner${i}/toautopilot /autopilotsim${i}/fromwaypointplanner
		fi
	else
		# Connect sim auto pilot
		yarp connect /autopilotsim${i}/tomapself /mapself${i}/fromautopilot
		yarp connect /mapself${i}/toautopilot /autopilotsim${i}/frommapself
		yarp connect /wpplanner${i}/toautopilot /autopilotsim${i}/fromwaypointplanner
	fi
	
	if [ $NUM_RADIO -lt $NUM ]; then
		#if [ $i -lt $[$NUM - $NUM_RADIO + $HIL] ]; then
		if [ $i -lt $[$NUM - $NUM_RADIO] ]; then
			# Connect real radio
			yarp connect /radio${i}/tomapself /mapself${i}/fromradio
			yarp connect /radio${i}/tomapuavs /mapuavs${i}/fromradio
			yarp connect /mapuavs${i}/toradio /radio${i}/frommapuavs
			yarp connect /radio${i}/tomsgplanner /msgplanner${i}/fromradio
			yarp connect /msgplanner${i}/toradio /radio${i}/frommsgplanner
			yarp connect /radio${i}/tomapfire /mapfire${i}/fromradio
		else
			# Connect sim radio
			yarp connect /radiosim${i}/tomapself /mapself${i}/fromradio
			yarp connect /radiosim${i}/tomapuavs /mapuavs${i}/fromradio
			yarp connect /mapuavs${i}/toradio /radiosim${i}/frommapuavs
			yarp connect /radiosim${i}/tomsgplanner /msgplanner${i}/fromradio
			yarp connect /msgplanner${i}/toradio /radiosim${i}/frommsgplanner
			yarp connect /radiosim${i}/tomapfire /mapfire${i}/fromradio
		fi
	else
		# Connect sim radio
		yarp connect /radiosim${i}/tomapself /mapself${i}/fromradio
		yarp connect /radiosim${i}/tomapuavs /mapuavs${i}/fromradio
		yarp connect /mapuavs${i}/toradio /radiosim${i}/frommapuavs
		yarp connect /radiosim${i}/tomsgplanner /msgplanner${i}/fromradio
		yarp connect /msgplanner${i}/toradio /radiosim${i}/frommsgplanner
		yarp connect /radiosim${i}/tomapfire /mapfire${i}/fromradio
	fi
	i=$[$i+1]
done
