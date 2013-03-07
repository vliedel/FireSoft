/**
 * @brief 
 * @file CFitnessGenCollision.cpp
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

#include "CFitnessGenCollision.h"
#include "CTime.h"

using namespace rur;

CFitnessGenCollision::~CFitnessGenCollision()
{
	delete ShMemFitness;
	delete ShMemUavs;
}

void CFitnessGenCollision::Init(std::string module_id)
{
	fitnessGenCollision::Init(module_id);

	ShMemNameFitness = "mapFitness_" + module_id;
	ShMemNameUavs = "mapUAV_" + module_id;
	config.load("config.json");
	//UavId = atoi(module_id.c_str());
	//srand(UavId+2); // seed of 0 and 1 are special cases


	try
	{
		// Open the shared memory
		ShMemFitness = new MapShMemType(boost::interprocess::open_only, ShMemNameFitness.c_str());
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());

		// Find the maps and mutexes using the c-string name
		FitnessVec =		ShMemFitness->find<Fitness3DVecType>("Collision").first;
		MutexFitness = 		ShMemFitness->find<MapMutexType>("MutexCollision").first;

		MapUavs =			ShMemUavs->find<MapUavType>("Map").first;
		MutexUavs = 		ShMemUavs->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		FitnessVec = NULL;
		MutexFitness = NULL;
		MapUavs = NULL;
		MutexUavs = NULL;
		delete ShMemFitness;
		delete ShMemUavs;
		std::cout << "Error in " << ShMemNameFitness << " or " << ShMemNameUavs << std::endl;
		throw;
	}

	LastGenTime = get_cur_1ms();
}

void CFitnessGenCollision::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}

	if (get_duration(LastGenTime, get_cur_1ms()) > config.IntervalTime)
	{
		LastGenTime = get_cur_1ms();
		GenFitness();
	}
	usleep(config.TickTime);
}


void CFitnessGenCollision::GenFitness()
{
	float rotation = 0.0;
	Position pos;
	std::vector<Position> checkPos;
	float distanceLeft;
	float stepDist = 2*config.CollisionSigmaX;
	std::vector<Position>::iterator itPos;
	std::vector<FitnessGaussian3D> gaussians;
	FitnessGaussian3D gaussian(0, FITSRC_COLLISION, pos, config.CollisionAmplitude,
			config.CollisionSigmaX, config.CollisionSigmaY, config.CollisionSigmaZ, rotation);

	{
		boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);

		MapUavIterType it;
		for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
		{
			if (it->second.data.State == UAVSTATE_LANDED)
				continue;

			gaussian.ID = it->second.data.UavId;

			if (config.Debug)
				std::cout << it->second.data.UavId << " Adding gaussians with center:";

			//FitnessGaussian3D gaussian(it->second.data.UavId, FITSRC_COLLISION, pos, amplitude, sigmaX, sigmaY, sigmaZ, rotation);
			gaussian.Center = it->second.data.Geom.Pos;
			gaussians.push_back(gaussian);

			if (config.Debug)
				std::cout << " [" << gaussian.Center.transpose() << "]";

			// The future path of the UAV should also be avoided
			//pos = it->second.data.Geom.Pos;
			checkPos.clear();
			if (it->second.data.WpNext[0].wpMode != WP_INVALID)
				it->second.data.WpNext[0].GetPath(checkPos, distanceLeft, stepDist, &(gaussian.Center)); // TODO: magic number
			if (checkPos.size() > 5 && it->second.data.WpNext[1].wpMode != WP_INVALID)
				it->second.data.WpNext[1].GetPath(checkPos, distanceLeft, stepDist);
			for (itPos = checkPos.begin(); itPos != checkPos.end(); ++itPos)
			{
				gaussian.Center = *itPos;
				gaussians.push_back(gaussian);
				if (config.Debug)
					std::cout << " [" << gaussian.Center.transpose() << "]";
			}


			// To be sure: also put a gaussian in the direction the uav is flying now
			gaussian.Center = it->second.data.Geom.Pos;
			gaussian.Center.x() += config.PredictAheadTime * it->second.data.Geom.GroundSpeed * cos(it->second.data.Geom.Heading.angle());
			gaussian.Center.y() += config.PredictAheadTime * it->second.data.Geom.GroundSpeed * sin(it->second.data.Geom.Heading.angle());
			gaussian.Center.z() -= config.PredictAheadTime * it->second.data.Geom.VerticalSpeed;
			gaussians.push_back(gaussian);
			if (config.Debug)
			{
				std::cout << " [" << gaussian.Center.transpose() << "]";
				std::cout << std::endl;
			}
		}
	}

	// Add gaussians to fitness map, outside uav map lock!
	{
		boost::interprocess::scoped_lock<MapMutexType> lockFitness(*MutexFitness);
		FitnessVec->clear();
		for (std::vector<FitnessGaussian3D>::iterator it=gaussians.begin(); it != gaussians.end(); ++it)
			FitnessVec->push_back(*it);
	}
}
