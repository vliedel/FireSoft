/**
 * @brief 
 * @file CMapFitness.cpp
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

#include "CMapFitness.h"

using namespace rur;

CMapFitness::~CMapFitness()
{
	delete ShMem;
	//delete MapAllocator;
	delete VoidAllocator;
	delete FitnessAllocator;
	delete FitnessWallAllocator;
	delete Fitness3DAllocator;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
}

void CMapFitness::Init(std::string module_id)
{
	mapFitness::Init(module_id);
	config.load("config.json");

	ShMemName = "mapFitness_" + module_id;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());

	std::cout << "sizeof(FitnessGaussian2D) = " << sizeof(FitnessGaussian2D) << std::endl;

	try
	{
		ShMem = new MapShMemType(boost::interprocess::create_only, ShMemName.c_str(), config.MapSize);

		//Initialize the shared memory STL-compatible allocator
		FitnessAllocator = new FitnessAllocatorType(ShMem->get_segment_manager());
		FitnessWallAllocator = new FitnessWallAllocatorType(ShMem->get_segment_manager());
		Fitness3DAllocator = new Fitness3DAllocatorType(ShMem->get_segment_manager());
		//MapAllocator = new FitnessMapAllocatorType(ShMem->get_segment_manager());
		VoidAllocator = new MapVoidAllocatorType(ShMem->get_segment_manager());


		//Construct a shared memory map.
		FitnessCollision =		ShMem->construct<Fitness3DVecType>("Collision")		(*Fitness3DAllocator);
		FitnessBattery =		ShMem->construct<FitnessVecType>("Battery")			(*FitnessAllocator);
		FitnessConnectivity =	ShMem->construct<FitnessVecType>("Connectivity")	(*FitnessAllocator);
		FitnessFires =			ShMem->construct<FitnessVecType>("Fires")			(*FitnessAllocator);
		FitnessStatic =			ShMem->construct<FitnessVecType>("Static")			(*FitnessAllocator);
		FitnessWall =			ShMem->construct<FitnessWallVecType>("Wall")		(*FitnessWallAllocator);
		//FitnessCoverage =		ShMem->construct<FitnessMapType>("Coverage")		(std::less<FitnessMapKeyType>(), *MapAllocator);
		//FitnessCoverage =		ShMem->construct<FitnessMapXType>("Coverage")		(std::less<FitnessMapKeyType>(), *VoidAllocator);
		FitnessCoverage =		ShMem->construct<FitnessMapType>("Coverage")		(*VoidAllocator);


		MutexCollision = ShMem->construct<MapMutexType>("MutexCollision") ();
		MutexBattery = ShMem->construct<MapMutexType>("MutexBattery") ();
		MutexConnectivity = ShMem->construct<MapMutexType>("MutexConnectivity") ();
		MutexFires = ShMem->construct<MapMutexType>("MutexFires") ();
		MutexStatic = ShMem->construct<MapMutexType>("MutexStatic") ();
		MutexWall = ShMem->construct<MapMutexType>("MutexWall") ();
		MutexCoverage = ShMem->construct<MapMutexType>("MutexCoverage") ();
	}
	catch(...)
	{
		FitnessCollision = NULL;
		FitnessBattery = NULL;
		FitnessConnectivity = NULL;
		FitnessFires = NULL;
		FitnessStatic = NULL;
		FitnessWall = NULL;
		FitnessCoverage = NULL;

		MutexCollision = NULL;
		MutexBattery = NULL;
		MutexConnectivity = NULL;
		MutexFires = NULL;
		MutexStatic = NULL;
		MutexWall = NULL;
		MutexCoverage = NULL;

		delete ShMem;
		//delete MapAllocator;
		delete VoidAllocator;
		delete FitnessAllocator;
		delete FitnessWallAllocator;
		delete Fitness3DAllocator;

		boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
		std::cout << "Error in " << ShMemName << std::endl;
		throw;
	}
}

void CMapFitness::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}
	usleep(config.TickTime);
}
