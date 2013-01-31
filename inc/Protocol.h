/**
 * @brief 
 * @file Protocol.h
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from
 * thread pools and TCP/IP components to control architectures and learning algorithms.
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Bart van Vliet <bart@almende.com>
 *
 * @author        Bart van Vliet
 * @date          Aug 22, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

enum EProtCommand {
	CMD_INIT=0,
	CMD_START,
	CMD_STOP
};

enum EProtMapUavDataIn {
	PROT_MAPUAV_DATAIN_UAV=0,
	PROT_MAPUAV_DATAIN_RADIOMSGPOS
};

enum EProtMapUavStatus {
	PROT_MAPUAV_STATUS_UPDATED=0
};

enum EProtMapSelfDataIn {
	PROT_MAPSELF_DATAIN_GEOM=0,
	PROT_MAPSELF_DATAIN_STATE,
	PROT_MAPSELF_DATAIN_AP_STATUS,
	PROT_MAPSELF_DATAIN_WAYPOINTS,
	PROT_MAPSELF_DATAIN_BATTERY,
//	PROT_MAPSELF_DATAIN_HOME,
	PROT_MAPSELF_DATAIN_NEIGHBOURS,
//	PROT_MAPSELF_DATAIN_ID,
	PROT_MAPSELF_GS_CMD
};

//enum EProtGSCommand {
//	PROT_GS_CMD_LAND=0
//};

enum EProtAutoPilot {
	PROT_AP_SET_WAYPOINTS=0,
	//PROT_AP_LAND
	PROT_AP_SET_LAND,
	PROT_AP_SET_MODE
};

enum EProtRadioMsg {
	PROT_RADIO_MSG_SELF=0,
	PROT_RADIO_MSG_RELAY_UAV,
	PROT_RADIO_MSG_RELAY_FIRE
};

enum EProtRadioState {
	PROT_RADIOSTATUS_INITIALIZING=0,
	PROT_RADIOSTATUS_SENDRECEIVE,
	PROT_RADIOSTATUS_END,
	PROT_RADIOSTATUS_IDLE,
	PROT_RADIOSTATUS_NUM
};

enum EProtMsgPlanner {
	PROT_MSGPLANNER_MSG_CMD=0
};

enum ESimCmd {
	PROT_SIMCMD_INIT=0,
	PROT_SIMCMD_SET_STATE,
	//PROT_SIMCMD_SET_UAVSTATE,
	PROT_SIMCMD_SET_GEOM,
	PROT_SIMCMD_SET_GEOM_EXTRA, // roll angle, heading
	PROT_SIMCMD_SET_WAYPOINTS,
	PROT_SIMCMD_SET_WIND,
	PROT_SIMCMD_SET_BATTERY,
	PROT_SIMCMD_START, // Do we need this for something?
	PROT_SIMCMD_LIFTOFF,
	PROT_SIMCMD_LAND,
	PROT_SIMCMD_TIMESTEP,
	PROT_SIMCMD_RADIO_NEIGHBOURS,
	PROT_SIMCMD_RADIO_ROUND_START,
	//PROT_SIMCMD_RADIO_ROUND_SEND_MSG,
	PROT_SIMCMD_RADIO_ROUND_BROADCAST_MSG,
	PROT_SIMCMD_RADIO_ROUND_END,
	PROT_SIMCMD_NUM
};

enum ESimStatus {
	PROT_SIMSTAT_ACK=0, // Command has been processed
	PROT_SIMSTAT_STATE,
	PROT_SIMSTAT_GEOM,
	PROT_SIMSTAT_GEOM_EXTRA, // roll angle, heading
	PROT_SIMSTAT_BATTERY,
	PROT_SIMSTAT_WP,
	PROT_SIMSTAT_BROADCAST_MSG,
	PROT_SIMSTAT_NUM
};


#endif /* PROTOCOL_H_ */
