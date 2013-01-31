/**
 * @brief 
 * @file CFitnessGenCoverage.cpp
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

#include "CFitnessGenCoverage.h"
#include "CTime.h"

using namespace rur;

CFitnessGenCoverage::~CFitnessGenCoverage()
{
	delete ShMemFitness;
	delete ShMemUavs;
	delete ShMemSelf;
}

void CFitnessGenCoverage::Init(std::string module_id)
{
	fitnessGenCoverage::Init(module_id);
	ShMemNameFitness = "mapFitness_" + module_id;
	ShMemNameUavs = "mapUAV_" + module_id;
	ShMemNameSelf = "mapSelf_" + module_id;
	UavId = atoi(module_id.c_str());
	config.load("config.json");
	//srand(UavId+2); // seed of 0 and 1 are special cases


	try
	{
		// Open the shared memory
		ShMemFitness = new MapShMemType(boost::interprocess::open_only, ShMemNameFitness.c_str());
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());

		// Find the maps and mutexes using the c-string name
		FitnessMap =		ShMemFitness->find<FitnessMapType>("Coverage").first;
		MutexFitness = 		ShMemFitness->find<MapMutexType>("MutexCoverage").first;

		MapUavs =			ShMemUavs->find<MapUavType>("Map").first;
		MutexUavs = 		ShMemUavs->find<MapMutexType>("Mutex").first;

		MapSelf =			ShMemSelf->find<MapSelfStruct>("Map").first;
		MutexSelf = 		ShMemSelf->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		FitnessMap = NULL;
		MutexFitness = NULL;
		MapUavs = NULL;
		MutexUavs = NULL;
		MapSelf = NULL;
		MutexSelf = NULL;
		delete ShMemFitness;
		delete ShMemUavs;
		delete ShMemSelf;
		std::cout << "Error in " << ShMemNameFitness << " or " << ShMemNameSelf << " or " << ShMemNameUavs << std::endl;
		throw;
	}



//	// Test to see if range query works
//	float amplitude = 1.0;
//	float sigmaX = 10.0;
//	float sigmaY = 10.0;
//	float rotation = 0.0;
//	Position pos;
//	FitnessGaussian2D gaussian(0, FITSRC_COLLISION, pos, amplitude, sigmaX, sigmaY, rotation);
//
//	for (int x=0; x<1000; x+=10)
//	{
//		for (int y=0; y<1000; y+=10)
//		{
//			gaussian.Center.x() = x;
//			gaussian.Center.y() = y;
//			FitnessMap->AddGaussian(gaussian);
//		}
//	}
//
//	std::vector<FitnessGaussian2D> gaussians;
//	FitnessMap->Get(gaussians, 60, 160, 575, 675);
//	std::vector<FitnessGaussian2D>::iterator it;
//	for (it = gaussians.begin(); it != gaussians.end(); ++it)
//	{
//		std::cout << "got [" << it->Center.x() << " " << it->Center.y() << "]" << std::endl;
//	}


}

void CFitnessGenCoverage::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}

	if (get_cur_1ms() - LastGenTime > config.IntervalTime)
	{
		LastGenTime = get_cur_1ms();
		GenFitness();
	}
	usleep(config.TickTime);
}


void CFitnessGenCoverage::GenFitness()
{
	// TODO: set the correct rotation

	std::cout << get_cur_1ms() << std::endl;
	float rotation = 0.0;
	Position pos;
//	FitnessGaussian2D gaussian(0, FITSRC_COVERAGE, pos, amplitude, sigmaX, sigmaY, rotation);
	FitnessGaussian2D gaussian(0, FITSRC_COVERAGE, pos, config.CoverageAmplitude, config.CoverageSigmaX, config.CoverageSigmaY, rotation);
	{
		boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);
		boost::interprocess::scoped_lock<MapMutexType> lockFitness(*MutexFitness);

		long time = get_cur_1ms();
		MapUavIterType it;
		for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
		{
			if (lastAddedTime[it->second.data.UavId] + config.CoverageIntervalTime < time)
			{
				gaussian.Center = it->second.data.Geom.Pos;
				std::cout << get_cur_1ms() << " ";
				FitnessMap->AddGaussian(gaussian);
				lastAddedTime[it->second.data.UavId] = time;
			}
		}
	}
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
		boost::interprocess::scoped_lock<MapMutexType> lockFitness(*MutexFitness);

		long time = get_cur_1ms();
		if (lastAddedTime[MapSelf->UavData.UavId] + config.CoverageIntervalTime < time)
		{
			CoverageSelfTrace cur;
			cur.Pos = MapSelf->UavData.Geom.Pos;
			cur.Heading = MapSelf->UavData.Geom.Heading.angle();
			cur.Time = time;
			Trace.push_back(cur);

			lastAddedTime[MapSelf->UavData.UavId] = time;
		}
	}

	long time = get_cur_1ms();
	std::deque<CoverageSelfTrace>::iterator it;
	while (!Trace.empty())
	{
		// Add items from the trace with a delay
		if (Trace.front().Time + config.CoverageSelfDelayTime > time)
			break;

		gaussian.Center = Trace.front().Pos;
		std::cout << get_cur_1ms() << " ";
		FitnessMap->AddGaussian(gaussian);

		Trace.pop_front();
	}
}
