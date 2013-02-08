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
}

void CMsgPlanner::Init(std::string module_id)
{
	msgPlanner::Init(module_id);
	ModuleId = module_id;
	config.load("config.json");

	ShMemNameUavs = "mapUAV_" + module_id;
	ShMemNameSelf = "mapSelf_" + module_id;

	try
	{
		// Open the shared memory
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());

		// Find the maps and mutexes using the c-string name
		MapUavs = ShMemUavs->find<MapUavType>("Map").first;
		MapSelf = ShMemSelf->find<MapSelfStruct>("Map").first;

		MutexUavs = ShMemUavs->find<MapMutexType>("Mutex").first;
		MutexSelf = ShMemSelf->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		MapUavs = NULL;
		MapSelf = NULL;
		MutexUavs = NULL;
		MutexSelf = NULL;
		delete ShMemUavs;
		delete ShMemSelf;
		throw;
	}
}

void CMsgPlanner::Tick()
{
	IntMsg = readCommand(false);
	if (IntMsg != NULL)
	{

	}

	IntMsg = readFromRadio(false);
	if (IntMsg != NULL)
	{
		std::cout << "MSGPLNR " << ModuleId << " from Radio: " << *IntMsg << std::endl;
		switch (*IntMsg)
		{
		case PROT_RADIOSTATUS_IDLE: // Time to fill up the radio buffer
			SelectMsgs();
			SendMsgs();
			break;
		case PROT_MSGPLANNER_MSG_CMD:

			break;
		}
	}
	usleep(config.TickTime);
}

void CMsgPlanner::SelectMsgs()
{
	std::cout << "selecting msgs" << std::endl;
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);

		SelectedMsgs[0].MessageType = RADIO_MSG_RELAY_POS;
		MapSelf->UavData.ToRadioMsg(SelectedMsgs[0].Pos);
	}

	{
		boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);

		// Find the uavs that have been last sent, and send their last state
		long lastSentTimes[RADIO_NUM_RELAY_PER_MSG-1] = {LONG_MAX};
		MapUavIterType iters[RADIO_NUM_RELAY_PER_MSG-1];
		for (int k=0; k<RADIO_NUM_RELAY_PER_MSG-1; ++k)
			iters[k] = MapUavs->end();

		MapUavIterType it;
		for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
		{
			std::cout << "checking uav " << it->second.data.UavId << std::endl;

			// If last sent happened later than last receive, there is nothing new to tell
			if (it->second.LastRadioSentTime > it->second.LastRadioReceiveTime)
				continue;

			// Check if last sent time is smaller than the current smallest N values
			for (int j=0; j<RADIO_NUM_RELAY_PER_MSG-1; ++j)
			{
				//std::cout << j << " Comparing " << it->second.LastRadioSentTime << " with " << lastSentTimes[j] << std::endl;

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
				SelectedMsgs[i+1] = invalidMsg;
			else
			{
				SelectedMsgs[i+1] = iters[i]->second.LastRadioMsgs[iters[i]->second.LastRadioMsgsIndex];
				iters[i]->second.LastRadioSentTime = get_cur_1ms();
			}
		}
	}

}

void CMsgPlanner::SendMsgs()
{
	RadioMsg bmsg;
	if (SelectedMsgs[0].MessageType == RADIO_MSG_RELAY_POS)
	{
		if (SelectedMsgs[1].MessageType == RADIO_MSG_RELAY_POS)
			bmsg.MessageType = RADIO_MSG_2POS;
		else
			bmsg.MessageType = RADIO_MSG_POS_2FIRE;
	}
	for (int i=0; i<RADIO_NUM_RELAY_PER_MSG; ++i)
	{
		bmsg.Data.Data[i] = SelectedMsgs[i];
		//ToCont(SelectedMsgs[i], vec);
		//vec = SelectedMsgs[i].ToVec();
	}
	VecMsgType vecMsg;
	//vecMsg.push_back(PROT_SIMSTAT_BROADCAST_MSG);
	ToCont(bmsg, vecMsg);
	writeToRadio(vecMsg);
}
