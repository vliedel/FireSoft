/**
 * @brief 
 * @file CMapUAVs.cpp
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

#include "CMapUAVs.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"

using namespace rur;

CMapUAVs::~CMapUAVs()
{
	Map = NULL;
	delete MapAllocator;
	delete ShMem;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
}

void CMapUAVs::Init(std::string module_id)
{
	mapUAVs::Init(module_id);
	config.load("config.json");

	ModuleId = module_id;

	//Shared memory front-end that is able to construct objects
	//associated with a c-string. Erase previous shared memory with the name
	//to be used and create the memory segment at the specified address and initialize resources
	ShMemName = "mapUAV_" + module_id;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());

	std::cout << "sizeof(MapUavStruct) = " << sizeof(MapUavStruct) << std::endl;

	try
	{
		ShMem = new MapShMemType(boost::interprocess::create_only, ShMemName.c_str(), config.MapSize);

		//Initialize the shared memory STL-compatible allocator
		MapAllocator = new MapUavAllocatorType(ShMem->get_segment_manager());

		//Construct a shared memory map.
		Map =
			ShMem->construct<MapUavType>("Map")	//object name
			(std::less<MapUavKeyType>()		//first  ctor parameter: comparison function
			,*MapAllocator);		//second ctor parameter: allocator

		//Mutex = new MapMutexType();
		Mutex = ShMem->construct<MapMutexType>("Mutex") ();
		//Mutex->lock();
		//std::cout << "map uavs locked the map" << std::endl;
		//Mutex->unlock();
	}
	catch(...)
	{
		Map = NULL;
		Mutex = NULL;
		delete ShMem;
		delete MapAllocator;
		boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
		std::cout << "Error in " << ShMemName << std::endl;
		throw;
	}
}

void CMapUAVs::Tick()
{
//	IntMsg = readCommand(false);
//	if (IntMsg != NULL)
//	{
//
//	}

	// All messages that will access the shared memory go in here, as we need a lock
	//if (Mutex->try_lock())
//	{
//		boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex, boost::interprocess::try_to_lock);
//		if (lock)
		//if (true)
//		{
	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		if (config.Debug > 0)
		{
			std::cout << get_cur_1ms() << " MAPUAVS " << ModuleId << " from Radio: ";
			dobots::print(VecMsg->begin(), VecMsg->end());
		}

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
//				case PROT_MAPUAV_DATAIN_UAV:
//				{
//					UavStruct uav;
//					it = FromCont(uav, it, VecMsg->end());
//					AddUAV(&uav);
//					break;
//				}
				case PROT_RADIO_MSG_RELAY:
				{
					RadioMsgRelay msg;
					it = FromCont(msg, it, VecMsg->end());
					UpdateUav(msg);
					break;
				}
			}
		}
		// Let the radio know we updated the uav map
		writeToRadio(PROT_MAPUAV_STATUS_UPDATED);
		VecMsg->clear();
		if (config.Debug > 1)
			std::cout << *this << std::endl;
	}
			//Mutex->unlock();
			//lock.unlock();
//		}
//		else
//			std::cout << "mapuavs has to wait for lock" << std::endl;
//		// Lock gets released here
//	}
	usleep(config.TickTime);
}

//void CMapUAVs::AddUAV(std::vector<float>* vec)
//{
//	std::cout << "Don't use this anymore" << std::endl;
//	return;
//
//	boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
//	printf("VecMsg size = %li. Contents: ", vec->size());
//	dobots::print(vec->begin(), vec->end());
//	UavStruct uav;
//	if (vec->end() == FromCont(uav, vec->begin(), vec->end()))
//		AddUAV(&uav);
//	else
//		printf("Invalid vector to create an UAVStruct!\n");
//}

//void CMapUAVs::AddUAV(UavStruct* uav)
//{
//	std::cout << "Don't use this anymore" << std::endl;
//	return;
//
//	boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
//	MapUavStruct uavOnMap;
//	uavOnMap.data = *uav;
//	uavOnMap.LastRadioReceiveTime = get_cur_1ms();
//	Map->erase(uav->UavId);
//	Map->insert(MapUavValueType(uav->UavId, uavOnMap));
//	//printf("Added/updated uav to shared mem\n");
//	//printf("Map size: %li\n", UAVMap->size());
//	//std::cout << *this << std::endl;
//}

void CMapUAVs::UpdateUav(RadioMsgRelay& msg)
{
//	int uavId;
//	switch (msg.MessageType)
//	{
//		case RADIO_MSG_RELAY_POS:
//		{
//			Uav.FromRadioMsg(msg.Pos);
//			uavId = Uav.UavId;
//			break;
//		}
//		case RADIO_MSG_RELAY_FIRE:
//		{
//			// TODO: handle this..
//			uavId = msg.Fires.Fire[0].UavId;
//			break;
//		}
//		case RADIO_MSG_RELAY_CMD:
//		{
//			// TODO: what to do?
//			break;
//		}
//	}

	if (msg.MessageType != RADIO_MSG_RELAY_POS)
		return;
	Uav.FromRadioMsg(msg.Pos);

	boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
	MapUavIterType it = Map->find(Uav.UavId);
	if (it == Map->end())
	{
		MapUavStruct uavOnMap;
		uavOnMap.data = Uav;
		uavOnMap.LastRadioMsgs[0] = msg; // LastRadioMsgs are initialized such that its filled with UavIds of 0 (invalid)
		uavOnMap.LastRadioMsgsIndex = 0;
		uavOnMap.LastRadioReceiveTime = get_cur_1ms();
		uavOnMap.LastRadioSentTime = get_cur_1ms() - 1000*3600; // One hour ago should do the trick..
		Map->insert(MapUavValueType(Uav.UavId, uavOnMap));
		if (config.Debug > 0)
			std::cout << "Added uav" << std::endl;
	}
	else
	{
		//bool found = false;
		for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
		{
			// TODO: this check is incomplete, but sufficient?
			if ((it->second.LastRadioMsgs[i].Pos.UavId != 0)
					&& (it->second.LastRadioMsgs[i].Pos.X == msg.Pos.X)
					&& (it->second.LastRadioMsgs[i].Pos.Y == msg.Pos.Y)
					&& (it->second.LastRadioMsgs[i].Pos.State == msg.Pos.State))
			{
				if (config.Debug > 0)
					std::cout << "Radio msg already received, ignoring update!" << std::endl;
				return;
				//found = true;
				//break;
			}
		}

		it->second.LastRadioMsgsIndex = (it->second.LastRadioMsgsIndex+1) % MAPUAV_RADIOMSG_HIST;
		it->second.LastRadioMsgs[it->second.LastRadioMsgsIndex] = msg;
		it->second.LastRadioReceiveTime = get_cur_1ms();
		it->second.data = Uav;
		if (config.Debug > 0)
			std::cout << "Updated uav" << std::endl;
	}
}



namespace rur
{
std::ostream& operator<<(std::ostream& os, CMapUAVs& map)
{
	os << "Currently " << map.Map->size() << " UAVs on the map:" << std::endl;
	//CMapUAVs::UAVMapIterType it;
	MapUavIterType it;
	for (it = map.Map->begin(); it != map.Map->end(); ++it)
	{
		os << it->second << std::endl;
	}

	return os;
}
}
