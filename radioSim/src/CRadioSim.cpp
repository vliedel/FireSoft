/**
 * @brief Radio class: can receive and send messages via the virtual radio.
 * @file CRadioSim.cpp
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
 * @date          Apr 24, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#include "CRadioSim.h"
#include "Protocol.h"
#include "Print.hpp"
//#include <cstdlib>

using namespace rur;
//using namespace std;

CRadioSim::~CRadioSim()
{
}

void CRadioSim::Init(std::string module_id)
{
	radiosim::Init(module_id);
	config.load("config.json");

	ModuleId = module_id;
	UavId = atoi(module_id.c_str());
}

void CRadioSim::Tick()
{
	IntMsg = readCommand(false);
	if (IntMsg != NULL)
	{

	}

	VecMsg = readSimCommand(false);
	if (!VecMsg->empty())
	{
		std::cout << "RADIO " << ModuleId << " from SIM: ";
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
					//ReadFromRadio();
					RadioRoundState = RADIO_STATE_ROUND_SENDRECEIVE;
					RadioMsg bmsg;
					it = FromCont(bmsg, it, VecMsg->end());
					std::cout << "Radio " << ModuleId << " received bmsg: " << bmsg << std::endl;
					ReceiveBuffer.push_back(bmsg);
					break;
				}
				case PROT_SIMCMD_RADIO_ROUND_END:
				{
					RadioRoundState = RADIO_STATE_ROUND_END; // At this state, we update UAV maps, need to wait for reply of MapUavs
					//RadioRoundState = RADIO_STATE_ROUND_IDLE; // end and idle are kinda the same here?

					// Send receive buffer to other modules, let MapUavs update first before we let the msgplanner do its work
					if (!ReadReceiveBuffer())
					{
						RadioRoundState = RADIO_STATE_ROUND_IDLE;
						writeToMsgPlanner(PROT_RADIOSTATUS_IDLE);
					}

					VecMsgType vecMsg;
					vecMsg.push_back(PROT_SIMSTAT_ACK);
					writeSimState(vecMsg);

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
			writeSimState(vecMsg);
		}
	}

	IntMsg = readFromMapUAVs(false);
	if (IntMsg != NULL)
	{
		switch(*IntMsg)
		{
			case PROT_MAPUAV_STATUS_UPDATED:
			{
				if (RadioRoundState == RADIO_STATE_ROUND_END)
				{
					RadioRoundState = RADIO_STATE_ROUND_IDLE;
					writeToMsgPlanner(PROT_RADIOSTATUS_IDLE);
				}
				break;
			}
		}
	}

	VecMsg = readFromMsgPlanner(false);
	if (!VecMsg->empty())
	{
		std::cout << "RADIO " << ModuleId << " from MsgPlanner: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		// Should have more protocol here?
		WriteToOutBuffer(VecMsg);
		VecMsg->clear();
	}
	usleep(config.TickTime);
}

bool CRadioSim::ReadReceiveBuffer()
{
	VecMsgType vecMsgUavs;
	//VecMsgType vecMsgSelf;
	VecMsgType vecMsgFire;
	UavStruct uav;
	while (!ReceiveBuffer.empty())
	{
		for (int i=0; i<RADIO_NUM_RELAY_PER_MSG; ++i)
		{
			switch (ReceiveBuffer.front().Data.Data[i].MessageType)
			{
				case RADIO_MSG_RELAY_POS:
				{
//					uav.UavId = ReceiveBuffer.front().Data.Data[i].Pos.UavId -1;
					uav.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Pos);

					// Ignore invalid uav IDs and own messages
					if ((uav.UavId > -1) && (uav.UavId != UavId))
					{
//						std::cout << ReceiveBuffer.front().Data.Data[i] << " === " << uav << std::endl;
						vecMsgUavs.push_back(PROT_RADIO_MSG_RELAY);
						ToCont(ReceiveBuffer.front().Data.Data[i], vecMsgUavs);
					}
					break;
				}
				case RADIO_MSG_RELAY_FIRE:
				{
					FireStruct fire;

					fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[0]);
					// Ignore invalid uav IDs and own messages
					if ((fire.UavId > -1) && (fire.UavId != UavId))
					{
//						std::cout << fire << std::endl;
						vecMsgFire.push_back(PROT_FIRE_STRUCT);
						ToCont(fire, vecMsgFire);
					}

					fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[1]);
					// Ignore invalid uav IDs and own messages
					if ((fire.UavId > -1) && (fire.UavId != UavId))
					{
//						std::cout << fire << std::endl;
						vecMsgFire.push_back(PROT_FIRE_STRUCT);
						ToCont(fire, vecMsgFire);
					}
					break;
				}
				case RADIO_MSG_RELAY_CMD:
				{
					int id = ReceiveBuffer.front().Data.Data[i].Cmd.UavId -1;
					if (id < 0)
						break;

					// If message is meant for this uav, execute command
					if (id == UavId || id == 14) // TODO: magic number
					{

					}
					// If message is meant for other uav, relay command
					if (id != UavId || id == 14) // TODO: magic number
					{

					}

					VecMsgType vecMsgSelf;
					ToCont(ReceiveBuffer.front().Data.Data[i].Cmd, vecMsgSelf);
					vecMsgSelf.push_back(PROT_MAPSELF_GS_CMD);
					writeToMapSelf(vecMsgSelf);

					break;
				}
			}
		}
		ReceiveBuffer.pop_front();
	}

	if (!vecMsgFire.empty())
	{
		writeToMapFire(vecMsgFire);
	}

	if (!vecMsgUavs.empty())
	{
		writeToMapUAVs(vecMsgUavs);
		return true;
	}

	return false;
}

void CRadioSim::WriteToRadio()
{
	VecMsgType vecMsg;
	vecMsg.push_back(PROT_SIMSTAT_ACK);
	vecMsg.push_back(PROT_SIMSTAT_BROADCAST_MSG); // The radio msg to be broadcasted has to go last (after the ACK)!
	if (!SendBuffer.empty())
	{
		ToCont(SendBuffer.front(), vecMsg);
		SendBuffer.pop_front();
	}
	writeSimState(vecMsg);

//	printf("sending %li msgs\n", SendBuffer.size());
////	std::vector<int> length(1, SendBuffer.size());
//
//	for (int i=0; (i<RADIO_NUM_RELAY_PER_MSG && !SendBuffer.empty()); ++i) // Only 4 relay messages allowed
//	{
//		RadioMsgVec msg = SendBuffer.front();
//		printf("Writing to radio: ");
//		dobots::print(msg.begin(), msg.end());
//		//for (RadioMsgVec::iterator it=msg.begin(); it != msg.end(); ++it)
//		//{
//		//	printf("%i ", *it);
//		//}
//		//printf("\n");
//
//		writeToUAVs(SendBuffer.front());
//		SendBuffer.pop_front();
//	}
}

void CRadioSim::WriteToOutBuffer(VecMsgType* vecMsg)
{
//	std::cout << "Requested msg to be sent: ";
//	dobots::print(vecMsg->begin(), vecMsg->end());

	RadioMsg msg;
	FromCont(msg, vecMsg->begin(), vecMsg->end());
	SendBuffer.push_back(msg);
}

void CRadioSim::UpdateState()
{

}
