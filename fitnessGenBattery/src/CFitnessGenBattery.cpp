/**
 * @brief 
 * @file CFitnessGenBattery.cpp
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

#include "CFitnessGenBattery.h"
#include "CTime.h"

using namespace rur;

CFitnessGenBattery::~CFitnessGenBattery()
{
	delete ShMemFitness;
	delete ShMemSelf;
}

void CFitnessGenBattery::Init(std::string module_id)
{
//	std::cout << "Initializing..." << std::endl;
	fitnessGenBattery::Init(module_id);
	ShMemNameFitness = "mapFitness_" + module_id;
	ShMemNameSelf = "mapSelf_" + module_id;
	config.load("config.json");
		//UavId = atoi(module_id.c_str());

	try
	{
		// Open the shared memory
		ShMemFitness = new MapShMemType(boost::interprocess::open_only, ShMemNameFitness.c_str());
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());

		// Find the maps and mutexes using the c-string name
		FitnessVec =		ShMemFitness->find<FitnessVecType>("Battery").first;
		MutexFitness = 		ShMemFitness->find<MapMutexType>("MutexBattery").first;

		MapSelf =			ShMemSelf->find<MapSelfStruct>("Map").first;
		MutexSelf = 		ShMemSelf->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		FitnessVec = NULL;
		MutexFitness = NULL;
		MapSelf = NULL;
		MutexSelf = NULL;
		delete ShMemFitness;
		delete ShMemSelf;
		std::cout << "Error in " << ShMemNameFitness << " or " << ShMemNameSelf << std::endl;
		throw;
	}

	LastGenTime = get_cur_1ms();

//	std::cout << "Initialized" << std::endl;
}

void CFitnessGenBattery::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}

//	std::cout << "cur=" << get_cur_1ms() << " last=" << LastGenTime << std::endl;
	if (get_duration(LastGenTime, get_cur_1ms()) > config.IntervalTime)
	{
		LastGenTime = get_cur_1ms();
		GenFitness();
	}
	usleep(config.TickTime);
}


void CFitnessGenBattery::GenFitness()
{
	std::vector<FitnessGaussian2D> gaussians;

	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);

		// Estimated time to fly to home is (x+y) * cruisespeed
//		float timeNeeded = config.BatteryLowTime + (config.MaxX+config.MaxY)*config.CruiseSpeed;
		float distLeft = MapSelf->UavData.Geom.Pos.x() - MapSelf->GsCmd.Landing.Pos.x();
		distLeft += MapSelf->UavData.Geom.Pos.y() - MapSelf->GsCmd.Landing.Pos.y();

		std::cout << "BatteryTimeLeft=" << MapSelf->UavData.BatteryTimeLeft << " distLeft=" << distLeft << std::endl;

		float timeNeeded = config.BatteryLowTime + distLeft/config.CruiseSpeed;
		if (MapSelf->UavData.BatteryTimeLeft < timeNeeded)
		{
			//FitnessGaussian2D(int id, FitnessSource source, Position& center, float amplitude, float sigmaX, float sigmaY, float rotation, float minVal=0, float maxVal=0)
			Position c;
			c << MapSelf->GsCmd.Landing.Pos.x(), MapSelf->GsCmd.Landing.Pos.y(), 0;
			// Amplitude = (Tlow - Tleft)/Tlow * Amax
			float amplitude = (timeNeeded - MapSelf->UavData.BatteryTimeLeft) / timeNeeded * config.BatteryAmplitude;
			//float sigma = std::max(config.MaxX, config.MaxY); // Same sigma for both
			float sigma = std::max(MapSelf->GsCmd.AreaSize.x(), MapSelf->GsCmd.AreaSize.y()); // Same sigma for both
			FitnessGaussian2D gauss(0, FITSRC_BATTERY, c, amplitude, sigma, sigma, 0.0);
			std::cout << "Adding Guassian: " << gauss << std::endl;
			gaussians.push_back(gauss);
		}
	}

	// Update fitness map, do this outside the mapself lock!
	{
		boost::interprocess::scoped_lock<MapMutexType> lockFitness(*MutexFitness);
		FitnessVec->clear();
		for (std::vector<FitnessGaussian2D>::iterator it=gaussians.begin(); it!=gaussians.end(); ++it)
			FitnessVec->push_back(*it);
	}
}
