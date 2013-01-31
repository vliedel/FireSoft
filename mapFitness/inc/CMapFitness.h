/**
 * @brief 
 * @file CMapFitness.h
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

#ifndef CMAPFITNESS_H_
#define CMAPFITNESS_H_

#include "mapFitness.h"
#include "Defs.h"
//#include "Fitness.h"
#include "FitnessMap.h"
#include "ShMemTypedefs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// 2d Fitness function is 1(amplitude) + 2(center) + 3(var) + 2(min&max) +1(type) ~= 9*4 = 36B
// 3d Fitness function is 1(amplitude) + 3(center) + 6(var) + 2(min&max) +1(type) ~= 13*4 = 52B
//#define MAP_FITNESS_SIZE 32*1024*1024 // 10 * 3600 * 44B for 10 uavs that leave a 2d fitness each second for an hour

namespace rur {

struct MapFitnessConfig
{
	long TickTime; // us
	bool Debug;
	long MapSize; // bytes

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("mapFitness.TickTime");
		Debug = pt.get<bool>("mapFitness.Debug");
		MapSize = pt.get<long>("mapFitness.MapSize");
	}
};

class CMapFitness : public mapFitness
{
	public:
		~CMapFitness();
		void Init(std::string module_id);
		void Tick();

		// Functions that it should have:
		// Find_in_range(x, y, z, radius, [type])
		// Find_all_type(type)
		// Insert(x, y, z, type)
		// -bundle close points of same type together?
		// Fitness(x,y,z, [type])

	private:
		MapShMemType* ShMem;
		MapMutexType* MutexCollision;
		MapMutexType* MutexBattery;
		MapMutexType* MutexConnectivity;
		MapMutexType* MutexFires;
		MapMutexType* MutexStatic;
		MapMutexType* MutexWall;
		MapMutexType* MutexCoverage;

		FitnessAllocatorType*		FitnessAllocator;
		FitnessWallAllocatorType*	FitnessWallAllocator;
		Fitness3DAllocatorType*		Fitness3DAllocator;
		//FitnessMapAllocatorType*	MapAllocator;
		MapVoidAllocatorType*		VoidAllocator;

		Fitness3DVecType*	FitnessCollision;
		FitnessVecType*		FitnessBattery;
		FitnessVecType*		FitnessConnectivity;
		FitnessVecType*		FitnessFires;
		FitnessVecType*		FitnessStatic;
		FitnessWallVecType*	FitnessWall;
		//FitnessMapXType*	FitnessCoverage;
		FitnessMapType*		FitnessCoverage;

		std::string ShMemName;

		MapFitnessConfig config;
		std::string ModuleId;

		int* IntMsg;
		std::vector<float>* VecMsg;
};

}
#endif // CMAPFITNESS_H_
