/**
 * @brief 
 * @file CGroundStationSim.cpp
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
 * @date          Jul 25, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#include "CGroundStationSim.h"
#include "Protocol.h"
#include "Print.hpp"

using namespace rur;

CGroundStationSim::~CGroundStationSim()
{

}

void CGroundStationSim::Init(std::string module_id)
{
	groundStationSim::Init(module_id);
	ModuleId = module_id;
	UavId = UAVS_NUM;
	config.load("config.json");
	RadioRoundState = RADIO_STATE_ROUND_IDLE;

	GsCmdStruct gsCmd;
	gsCmd.UavId = 15;
	gsCmd.MsgId = 0;
	gsCmd.HeightMin = config.MinHeight;
	gsCmd.HeightMax = config.MaxHeight;
	gsCmd.AreaZero << config.AreaOriginX, config.AreaOriginY, 0;
	gsCmd.AreaSize << config.AreaSizeX, config.AreaSizeY, 0;
	gsCmd.AreaRotation.angle() = config.AreaRotation;
	gsCmd.Landing.Pos << config.LandPointX, config.LandPointY, 0;
	gsCmd.Landing.Heading.angle() = config.LandHeading;
	gsCmd.Landing.LeftTurn = config.LandLeftTurn;
	gsCmd.Mode = AP_PROT_MODE_WP;
	gsCmd.EnablePlanner = true;



	CmdMsg.MessageType = RADIO_MSG_POS_CMD;
	CmdMsg.Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
	CmdMsg.Data.Data[0].Pos.UavId = 0; // Invalid msg

	CmdMsg.Data.Data[1].MessageType = RADIO_MSG_RELAY_CMD;
	gsCmd.toMsg(CmdMsg.Data.Data[1].Cmd);
//	CmdMsg.Data.Data[1].Cmd.UavId = 15;
//	CmdMsg.Data.Data[1].Cmd.MsgId = 0;
//	CmdMsg.Data.Data[1].Cmd.HeightMin = config.MinHeight - 50;
//	CmdMsg.Data.Data[1].Cmd.HeightMax = config.MaxHeight - 50;
//	CmdMsg.Data.Data[1].Cmd.AreaMinX = config.AreaOriginX / 5;
//	CmdMsg.Data.Data[1].Cmd.AreaMinY = config.AreaOriginY / 5;
//	CmdMsg.Data.Data[1].Cmd.AreaDX = config.AreaSizeX / 5;
//	CmdMsg.Data.Data[1].Cmd.AreaDY = config.AreaSizeY / 5;
//	// Rotation is 0 to 0.5*pi translate to 0 to 1023
//	CmdMsg.Data.Data[1].Cmd.AreaRotation = config.AreaRotation * 1023*2/M_PI;
//	CmdMsg.Data.Data[1].Cmd.LandX = config.LandPointX;
//	CmdMsg.Data.Data[1].Cmd.LandY = config.LandPointY;
//	// Heading is 0 to 2*pi, translate to 0 to 255
//	CmdMsg.Data.Data[1].Cmd.LandHeading = config.LandHeading * 255/2/M_PI;
//	CmdMsg.Data.Data[1].Cmd.LandLeftTurn = config.LandLeftTurn;
//	CmdMsg.Data.Data[1].Cmd.Mode = AP_PROT_MODE_WP;
//	CmdMsg.Data.Data[1].Cmd.EnablePlanner = true;
}

void CGroundStationSim::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}

	VecMsg = readSim(false);
	if (!VecMsg->empty())
	{
		std::cout << "GroundStation " << ModuleId << " from SIM: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			//std::cout << "Type=" << type << std::endl;
			switch (type)
			{
				case PROT_SIMCMD_RADIO_ROUND_START:
				{
					// Our turn to send messages
					RadioRoundState = RADIO_STATE_ROUND_START;
					WriteToRadio();
					break;
				}
				case PROT_SIMCMD_RADIO_ROUND_BROADCAST_MSG:
				{
					// Received messages from other uavs
					RadioRoundState = RADIO_STATE_ROUND_SENDRECEIVE;
					RadioMsg bmsg;
					it = FromCont(bmsg, it, VecMsg->end());
					std::cout << "GroundStation " << ModuleId << " received bmsg: " << bmsg << std::endl;
					ReceiveBuffer.push_back(bmsg);
					break;
				}
				case PROT_SIMCMD_RADIO_ROUND_END:
				{
					//RadioRoundState = RADIO_STATE_ROUND_END; // At this state we wait for other parts to update before going to IDLE
					ReadReceiveBuffer();
					RadioRoundState = RADIO_STATE_ROUND_IDLE; // At this state the write buffer can be filled with new msgs.

					VecMsgType vecMsg;
					vecMsg.push_back(PROT_SIMSTAT_ACK);
					std::cout << "to sim: ";
					dobots::print(vecMsg.begin(), vecMsg.end());
					writeToSim(vecMsg);

					// Fill outbuffer with "new" msg
					WriteToOutBuffer(CmdMsg);

					break;
				}
			}
		}
		VecMsg->clear();

		// Send ack when we received the msgs from other uavs
		if (RadioRoundState == RADIO_STATE_ROUND_SENDRECEIVE)
		{
			VecMsgType vecMsg;
			vecMsg.push_back(PROT_SIMSTAT_ACK);
			std::cout << "to sim: ";
			dobots::print(vecMsg.begin(), vecMsg.end());
			writeToSim(vecMsg);
		}
	}

	usleep(config.TickTime);
}


void CGroundStationSim::ReadReceiveBuffer()
{
	UavStruct uav;
	while (!ReceiveBuffer.empty())
	{
		for (int i=0; i<RADIO_NUM_RELAY_PER_MSG; ++i)
		{
			switch (ReceiveBuffer.front().Data.Data[i].MessageType)
			{
				case RADIO_MSG_RELAY_POS:
				{
					uav.UavId = ReceiveBuffer.front().Data.Data[i].Pos.UavId -1;

					// Ignore invalid uav IDs and own messages
					if ((uav.UavId > -1) && (uav.UavId != UavId))
					{
						uav.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Pos);
						std::cout << "GroundStation " << ModuleId << " received: " << ReceiveBuffer.front().Data.Data[i] << " === " << uav << std::endl;
						VecMsgType vecMsg;
						vecMsg.push_back(PROT_RADIO_MSG_RELAY);
						ToCont(ReceiveBuffer.front().Data.Data[i], vecMsg);
						writeToMapUavs(vecMsg);

						vecMsg.clear();
						vecMsg.push_back(PROT_RADIO_MSG_RELAY_POS);
						ToCont(ReceiveBuffer.front().Data.Data[i].Pos, vecMsg);
						writeToGuiInterface(vecMsg);
					}
					break;
				}
				case RADIO_MSG_RELAY_FIRE:
				{
					// Add to fire map
					break;
				}
				case RADIO_MSG_RELAY_CMD:
				{
					// Ignore
					break;
				}
			}
		}
		ReceiveBuffer.pop_front();
	}
}

void CGroundStationSim::WriteToRadio()
{
	VecMsgType vecMsg;
	vecMsg.push_back(PROT_SIMSTAT_ACK);
	vecMsg.push_back(PROT_SIMSTAT_BROADCAST_MSG); // The radio msg to be broadcasted has to go last (after the ACK)!
	if (!SendBuffer.empty())
	{
		ToCont(SendBuffer.front(), vecMsg);
		SendBuffer.pop_front();
	}
	std::cout << "to sim: ";
	dobots::print(vecMsg.begin(), vecMsg.end());
	writeToSim(vecMsg);
}

void CGroundStationSim::WriteToOutBuffer(RadioMsg& msg)
{
	std::cout << "Requested msg to be sent: " << msg << std::endl;

	SendBuffer.push_back(msg);
}
