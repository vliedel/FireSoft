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

using namespace rur;

CMapFire::~CMapFire()
{
	Map = NULL;
	delete MapAllocator;
	delete ShMem;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
}

void CMapFire::Init(std::string module_id)
{
	mapFire::Init(module_id);
	config.load("config.json");

	ModuleId = module_id;

	//Shared memory front-end that is able to construct objects
	//associated with a c-string. Erase previous shared memory with the name
	//to be used and create the memory segment at the specified address and initialize resources
	ShMemName = "mapFire_" + module_id;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());

	std::cout << "sizeof(MapFireValueType) = " << sizeof(MapFireValueType) << std::endl;

	try
	{
		ShMem = new MapShMemType(boost::interprocess::create_only, ShMemName.c_str(), config.MapSize);

		//Initialize the shared memory STL-compatible allocator
		MapAllocator = new MapFireAllocatorType(ShMem->get_segment_manager());

		//Construct a shared memory map.
		Map =
			ShMem->construct<MapFireType>("Map")	//object name
			(std::less<MapFireKeyType>()		//first  ctor parameter: comparison function
			,*MapAllocator);		//second ctor parameter: allocator

		Mutex = ShMem->construct<MapMutexType>("Mutex") ();
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

void CMapFire::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}
	usleep(config.TickTime);
}
