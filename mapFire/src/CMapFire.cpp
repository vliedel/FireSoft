/**
 * @brief 
 * @file CMapFire.cpp
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

#include "CMapFire.h"
#include "Protocol.h"
#include "StructToFromCont.h"
#include "Print.hpp"
#include "CTime.h"

using namespace rur;

CMapFire::~CMapFire()
{
	Map = NULL;
//	delete MapAllocator;
	delete VoidAllocator;
	delete ShMem;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
}

void CMapFire::Init(std::string module_id)
{
	mapFire::Init(module_id);
	config.load("config.json");

	ModuleId = module_id;
	UavId = atoi(module_id.c_str());

	//Shared memory front-end that is able to construct objects
	//associated with a c-string. Erase previous shared memory with the name
	//to be used and create the memory segment at the specified address and initialize resources
	ShMemName = "mapFire_" + module_id;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());

	//std::cout << "sizeof(MapFireValueType) = " << sizeof(MapFireValueType) << std::endl;
	std::cout << "sizeof(MapFireStruct) = " << sizeof(MapFireStruct) << std::endl;

	try
	{
		ShMem = new MapShMemType(boost::interprocess::create_only, ShMemName.c_str(), config.MapSize);

		//Initialize the shared memory STL-compatible allocator
		//MapAllocator = new MapFireAllocatorType(ShMem->get_segment_manager());
		VoidAllocator = new MapVoidAllocatorType(ShMem->get_segment_manager());

		//Construct a shared memory map.
//		Map =
//			ShMem->construct<MapFireType>("Map")	//object name
//			(std::less<MapFireKeyType>()		//first  ctor parameter: comparison function
//			,*MapAllocator);		//second ctor parameter: allocator
		Map = ShMem->construct<MapFireType>("Map")		(*VoidAllocator);

		Mutex = ShMem->construct<MapMutexType>("Mutex") ();
	}
	catch(...)
	{
		Map = NULL;
		Mutex = NULL;
		delete ShMem;
		//delete MapAllocator;
		delete VoidAllocator;
		boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
		std::cout << "Error in " << ShMemName << std::endl;
		throw;
	}

	MapFireStruct fire;
	fire.Seen = true;
	fire.Sent = false;
	fire.Fire.Center.x() = 1500;
	fire.Fire.Center.y() = 2000;
	fire.Fire.Center.z() = 0;

	fire.Fire.Probability[FIRE_SRC_CAM] = 0.1;
	fire.Fire.Probability[FIRE_SRC_TPA] = 0.2;
	fire.Fire.Probability[FIRE_SRC_CO] = 0.3;
	fire.Fire.Init(10, 15, M_PI/8);
	fire.Fire.Height = 100;
	fire.Fire.UavId = UavId;
	fire.Fire.Amplitude = 1-(1-fire.Fire.Probability[FIRE_SRC_CAM])*(1-fire.Fire.Probability[FIRE_SRC_TPA])*(1-fire.Fire.Probability[FIRE_SRC_CO]);
	AddFire(fire);

	fire.Fire.Center.x() = 1600;
	fire.Fire.Center.y() = 2100;
	AddFire(fire);

	fire.Fire.Center.x() = 1700;
	fire.Fire.Center.y() = 2200;
	AddFire(fire);

	fire.Fire.Center.x() = 1800;
	fire.Fire.Center.y() = 2300;
	AddFire(fire);

	fire.Fire.Center.x() = 1900;
	fire.Fire.Center.y() = 2400;
	AddFire(fire);

	LastTestFireGenTime = get_cur_1ms();
}

void CMapFire::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}


	VecMsg = readFromFireDetector(false);
	if (!VecMsg->empty())
	{
		std::cout << "MapFire " << ModuleId << " from Detector: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_FIRE_STRUCT:
				{
					//Position pos(0,0,0);
					//MapFireStruct fire(pos, 1, 1, 1, 0);
					MapFireStruct fire;
					FromCont(fire.Fire, it, VecMsg->end());
					fire.Seen = true;
					fire.Sent = false;
					AddFire(fire);
					break;
				}
			}
		}
		VecMsg->clear();
	}

	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		std::cout << "MapFire " << ModuleId << " from Radio: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_FIRE_STRUCT:
				{
					MapFireStruct fire;
					it = FromCont(fire.Fire, it, VecMsg->end());
					fire.Seen = false;
					fire.Sent = false;
//					std::cout << "Adding: " << fire.Fire << std::endl;
					AddFire(fire);
					break;
				}
			}
		}
		VecMsg->clear();
	}

	// For test purposes: add random fires
	if (get_duration(LastTestFireGenTime, get_cur_1ms()) > 10000)
	{
		MapFireStruct fire;
		fire.Seen = true;
		fire.Sent = false;
		fire.Fire.Center.x() = rand() % 1000 + 1000;
		fire.Fire.Center.y() = rand() % 1000 + 1000;
		fire.Fire.Center.z() = 0;

		fire.Fire.Probability[FIRE_SRC_CAM] = 0.1;
		fire.Fire.Probability[FIRE_SRC_TPA] = 0.2;
		fire.Fire.Probability[FIRE_SRC_CO] = 0.3;
		fire.Fire.Init(10, 15, M_PI/8);
		fire.Fire.Height = 100;
		fire.Fire.UavId = UavId;
		fire.Fire.Amplitude = 1-(1-fire.Fire.Probability[FIRE_SRC_CAM])*(1-fire.Fire.Probability[FIRE_SRC_TPA])*(1-fire.Fire.Probability[FIRE_SRC_CO]);
		AddFire(fire);

		LastTestFireGenTime = get_cur_1ms();
	}

	usleep(config.TickTime);
}

void CMapFire::AddFire(MapFireStruct& fire)
{
	boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
	// TODO: Should do more than this, merge?
	Map->AddGaussian(fire);
}
