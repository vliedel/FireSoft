/**
 * @brief 
 * @file CMsgPlanner.cpp
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

#include "CMsgPlanner.h"
#include "Protocol.h"
#include "Print.hpp"
#include <climits>
#include "CTime.h"

using namespace rur;

CMsgPlanner::~CMsgPlanner()
{
	delete ShMemUavs;
	delete ShMemSelf;
	delete ShMemFire;
}

void CMsgPlanner::Init(std::string module_id)
{
	msgPlanner::Init(module_id);
	ModuleId = module_id;
	config.load("config.json");

	ShMemNameUavs = "mapUAV_" + module_id;
	ShMemNameSelf = "mapSelf_" + module_id;
	ShMemNameFire = "mapFire_" + module_id;

	try
	{
		// Open the shared memory
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());
		ShMemFire = new MapShMemType(boost::interprocess::open_only, ShMemNameFire.c_str());

		// Find the maps and mutexes using the c-string name
		MapUavs = ShMemUavs->find<MapUavType>("Map").first;
		MapSelf = ShMemSelf->find<MapSelfStruct>("Map").first;
		MapFire = ShMemFire->find<MapFireType>("Map").first;

		MutexUavs = ShMemUavs->find<MapMutexType>("Mutex").first;
		MutexSelf = ShMemSelf->find<MapMutexType>("Mutex").first;
		MutexFire = ShMemFire->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		MapUavs = NULL;
		MapSelf = NULL;
		MapFire = NULL;
		MutexUavs = NULL;
		MutexSelf = NULL;
		MutexFire = NULL;
		delete ShMemUavs;
		delete ShMemSelf;
		delete ShMemFire;
		throw;
	}
	SendOwnPos = true;
	RelayPos = true;

	// Init with invalid msgs
	SelectedRadioMsg.MessageType = RADIO_MSG_2POS;
	RadioMsgRelay invalidMsg;
	invalidMsg.MessageType = RADIO_MSG_RELAY_POS;
	invalidMsg.Pos.UavId = 0;
	for (int i=0; i<RADIO_NUM_RELAY_PER_MSG; ++i)
		SelectedRadioMsg.Data.Data[i+1] = invalidMsg;

}

void CMsgPlanner::Tick()
{
//	IntMsg = readCommand(false);
//	if (IntMsg != NULL)
//	{
//
//	}

	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		if (config.Debug > 0)
		{
			std::cout << get_cur_1ms() << " MSGPLNR " << ModuleId << " from Radio: ";
			dobots::print(VecMsg->begin(), VecMsg->end());
		}

		std::vector<int>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_RADIOSTATUS_IDLE: // Time to fill up the radio buffer: deprecated
					SelectMsgs();
					SendMsgs();
					break;
				case PROT_RADIO_STATUS_BUF_SIZE:
					int BufSize = *it++;
					if (BufSize < 1) // TODO: magic number, might want to keep more than 1 in buffer
					{
						SelectMsgs();
						SendMsgs();
					}
					break;
			}
		}
		VecMsg->clear();
	}
	usleep(config.TickTime);
}

void CMsgPlanner::SelectMsgs()
{
	if (config.Debug > 1)
		std::cout << "Selecting msgs" << std::endl;

	bool relayCmdMsg = false;
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);

		// Relay command message if there is any not relayed yet
		// Of each 2 messages, at least 1 will send the own position (the other can be a command msg)
		if (!SendOwnPos)
		{
//			std::cout << "selecting cmd msg.." << std::endl;
			// Only loop half of the history
			// TODO: magic number
			for (int j=0, i=MapSelf->LastGsCmdsIndex; j<MAPSELF_GS_CMDS_HIST/2; ++j, i=(i+1)%MAPSELF_GS_CMDS_HIST)
			{
				// Check if the msg has to be relayed (id is valid and not ours, msg hasn't been relayed too often yet)
				if ((MapSelf->LastGsCmds[i].Msg.UavId != 0)
						&& (MapSelf->LastGsCmds[i].Msg.UavId != MapSelf->UavData.UavId + 1)
						&& (MapSelf->LastGsCmds[i].TimesSent < config.RelayNumCmdMsg))
				{
					// Relay the cmd msg
//					std::cout << "Selected cmd msg " << MapSelf->LastGsCmds[i].Msg << std::endl;
					SelectedRadioMsg.Data.Data[0].MessageType = RADIO_MSG_RELAY_CMD;
					SelectedRadioMsg.Data.Data[0].Cmd = MapSelf->LastGsCmds[i].Msg;
					MapSelf->LastGsCmds[i].TimesSent++;
					relayCmdMsg = true;
					break;
				}

			}
			if (!relayCmdMsg)
			{
				SendOwnPos = true;
//				std::cout << "No cmd msg to send" << std::endl;
			}
		}

		// Send own position
		// Of each 2 messages, at least 1 will send the own position (the other can be a command msg)
		if (SendOwnPos)
		{
//			std::cout << "Selected own pos" << std::endl;
			//SelectedMsgs[0].MessageType = RADIO_MSG_RELAY_POS;
			SelectedRadioMsg.Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
			//MapSelf->UavData.ToRadioMsg(SelectedMsgs[0].Pos);
			MapSelf->UavData.ToRadioMsg(SelectedRadioMsg.Data.Data[0].Pos);
			SendOwnPos = false;
		}
	}


	// Send fire msg if there are any not sent yet
	// Of each 2 messages, at least 1 will relay a pos (the other can be a fire msg)
//	std::cout << "RelayPos=" << RelayPos << " relayCmdMsg=" << relayCmdMsg << std::endl;
	bool sendFire = false;
	if (!RelayPos && !relayCmdMsg) // There is no combination GsCmd + Fire
	{
		boost::interprocess::scoped_lock<MapMutexType> lockFire(*MutexFire);

//		std::cout << "selecting fire msg.." << std::endl;
		// Loop over all fires
		FireMapIterXType itX;
		FireMapIterYType itY;
		int f=0;
		for (itX=MapFire->MapX.begin(); itX != MapFire->MapX.end(); ++itX)
		{
			for (itY=itX->second.begin(); itY!=itX->second.end(); ++itY)
			{
				//if (!itY->second.Sent && itY->second.Seen)
				if (!itY->second.Sent)
				{
//					std::cout << "Selected fire msg: " << itY->second << std::endl;
					SelectedRadioMsg.Data.Data[1].MessageType = RADIO_MSG_RELAY_FIRE;
					itY->second.Fire.ToRadioMsg(SelectedRadioMsg.Data.Data[1].Fires.Fire[f]);
					++f;
					sendFire = true;
					itY->second.Sent = true;
				}
				if (f > 1)
					break;
			}
			if (f > 1)
				break;
		}
		if (f < 2)
		{
			// Only 1 fire msg has been selected, fill up the 2nd with an invalid fire msg
			SelectedRadioMsg.Data.Data[1].Fires.Fire[f].UavId = 0;
		}
		if (!sendFire)
		{
			RelayPos = true;
//			std::cout << "No fire msg to send" << std::endl;
		}
	}

	// Relay a pos msg
	// Of each 2 messages, at least 1 will relay a pos (the other can be a fire msg)
	if (RelayPos || relayCmdMsg) // There is no combination GsCmd + Fire
	{
		boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);

		if (config.Debug > 1)
			std::cout << "Selecting relay pos msg" << std::endl;
		RelayPos = false;
		// Find the uavs that have been last sent, and send their last state
		long lastSentTimes[RADIO_NUM_RELAY_PER_MSG-1] = {LONG_MAX};
		MapUavIterType iters[RADIO_NUM_RELAY_PER_MSG-1];
		for (int k=0; k<RADIO_NUM_RELAY_PER_MSG-1; ++k)
			iters[k] = MapUavs->end();

		MapUavIterType it;
		for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
		{
			if (config.Debug > 1)
				std::cout << "checking uav " << it->second.data.UavId << std::endl;

			// If last sent happened later than last receive, there is nothing new to tell
			if (it->second.LastRadioSentTime > it->second.LastRadioReceiveTime)
				continue;

			// Check if last sent time is smaller than the current smallest N values
			for (int j=0; j<RADIO_NUM_RELAY_PER_MSG-1; ++j)
			{
				if (config.Debug > 1)
					std::cout << j << " Comparing " << it->second.LastRadioSentTime << " with " << lastSentTimes[j] << std::endl;

				if (it->second.LastRadioSentTime < lastSentTimes[j])
				{
					// If so: shift the array
					for (int k=RADIO_NUM_RELAY_PER_MSG-2; k>j; --k)
					{
						lastSentTimes[k] = lastSentTimes[k-1];
						iters[k] = iters[k-1];
					}
					lastSentTimes[j] = it->second.LastRadioSentTime;
					iters[j] = it;
					dobots::print(lastSentTimes, lastSentTimes+RADIO_NUM_RELAY_PER_MSG-1);
					break;
				}
			}
		}

		// Fill up selected msgs
		RadioMsgRelay invalidMsg;
		invalidMsg.MessageType = RADIO_MSG_RELAY_POS;
		invalidMsg.Pos.UavId = 0;
		for (int i=0; i<RADIO_NUM_RELAY_PER_MSG-1; ++i)
		{
			if (lastSentTimes[i] == LONG_MAX)
			{
				if (config.Debug > 1)
					std::cout << "Selected invalid pos msg" << std::endl;
				SelectedRadioMsg.Data.Data[i+1] = invalidMsg;
			}
			else
			{
				if (config.Debug > 1)
					std::cout << "Selected:" << iters[i]->second.LastRadioMsgs[iters[i]->second.LastRadioMsgsIndex] << std::endl;
				SelectedRadioMsg.Data.Data[i+1] = iters[i]->second.LastRadioMsgs[iters[i]->second.LastRadioMsgsIndex];
				iters[i]->second.LastRadioSentTime = get_cur_1ms();
			}
		}
	}

	// If a cmd msg has been relayed, next radio message should send own pos
	if (relayCmdMsg)
		SendOwnPos=true;
	// If a fire msg has been sent, next radio message should relay a pos
	if (sendFire)
		RelayPos = true;
//	std::cout << "End of message selection. SendOwnPos=" << SendOwnPos << " RelayPos=" << RelayPos << std::endl;

}

void CMsgPlanner::SendMsgs()
{
	if (SelectedRadioMsg.Data.Data[0].MessageType == RADIO_MSG_RELAY_POS)
	{
		if (SelectedRadioMsg.Data.Data[1].MessageType == RADIO_MSG_RELAY_POS)
			SelectedRadioMsg.MessageType = RADIO_MSG_2POS;
		else
			SelectedRadioMsg.MessageType = RADIO_MSG_POS_2FIRE;
	}
	if (SelectedRadioMsg.Data.Data[0].MessageType == RADIO_MSG_RELAY_CMD)
	{
		if (SelectedRadioMsg.Data.Data[1].MessageType != RADIO_MSG_RELAY_POS)
			std::cout << "Error: can't combine cmd msg and fire msg!" << std::endl;
		RadioMsgRelay tmp = SelectedRadioMsg.Data.Data[1];
		SelectedRadioMsg.Data.Data[1] = SelectedRadioMsg.Data.Data[0];
		SelectedRadioMsg.Data.Data[0] = tmp;
		SelectedRadioMsg.MessageType = RADIO_MSG_POS_CMD;
	}

	VecMsgType vecMsg;
	//vecMsg.push_back(PROT_SIMSTAT_BROADCAST_MSG);
	ToCont(SelectedRadioMsg, vecMsg);
	writeToRadio(vecMsg);
}
