/**
 * @brief 
 * @file CFitnessGenStatic.cpp
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

#include "CFitnessGenStatic.h"
#include "CTime.h"

using namespace rur;

CFitnessGenStatic::~CFitnessGenStatic()
{
	delete ShMemFitness;
	delete ShMemSelf;
}

void CFitnessGenStatic::Init(std::string module_id)
{
	fitnessGenStatic::Init(module_id);

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
		FitnessStaticVec =		ShMemFitness->find<FitnessVecType>("Static").first;
		MutexFitnessStatic = 	ShMemFitness->find<MapMutexType>("MutexStatic").first;
		FitnessWallVec =		ShMemFitness->find<FitnessWallVecType>("Wall").first;
		MutexFitnessWall = 		ShMemFitness->find<MapMutexType>("MutexWall").first;
		MapSelf =			ShMemSelf->find<MapSelfStruct>("Map").first;
		MutexSelf = 		ShMemSelf->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		FitnessStaticVec = NULL;
		MutexFitnessStatic = NULL;
		FitnessWallVec = NULL;
		MutexFitnessWall = NULL;
		MapSelf = NULL;
		MutexSelf = NULL;
		delete ShMemFitness;
		delete ShMemSelf;
		std::cout << "Error in " << ShMemNameFitness << " or " << ShMemNameSelf << std::endl;
		throw;
	}
//	std::cout << "opened shared memory" << std::endl;
//	if (FitnessStaticVec == NULL)
//		std::cout << "FitnessStaticVec=NULL" << std::endl;
//	if (MutexFitnessStatic == NULL)
//		std::cout << "MutexFitnessStatic=NULL" << std::endl;
//	if (FitnessWallVec == NULL)
//		std::cout << "FitnessWallVec=NULL" << std::endl;
//	if (MutexFitnessWall == NULL)
//		std::cout << "MutexFitnessWall=NULL" << std::endl;

	GenFitness();
	LastTimeGen = get_cur_1ms();
}

void CFitnessGenStatic::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}

	if (get_cur_1ms() - LastTimeGen > config.IntervalTime)
	{
		GenFitness();
		LastTimeGen = get_cur_1ms();
	}

	usleep(config.TickTime);
}

void CFitnessGenStatic::GenFitness()
{
	// Add walls on the edges of the map, so the UAV won't fly outside the map
	// Offset the walls <WallSigma> to the inside, so that the value at the edge is correct
	Position p0, p1, p2, p3;
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
		p0 = MapSelf->AreaZero;
		p0.x() += config.WallSigma;
		p0.y() += config.WallSigma;

		float dx = MapSelf->AreaSize.x() - 2*config.WallSigma;
		float dy = MapSelf->AreaSize.y() - 2*config.WallSigma;

		p1 = p0;
		p1.x() += dx * cos(MapSelf->AreaRotation.angle());
		p1.y() += dx * sin(MapSelf->AreaRotation.angle());

		p2 = p1;
		p2.x() += dy * cos(MapSelf->AreaRotation.angle() + 0.5*M_PI);
		p2.y() += dy * sin(MapSelf->AreaRotation.angle() + 0.5*M_PI);

		p3 = p0;
		p3.x() += dy * cos(MapSelf->AreaRotation.angle() + 0.5*M_PI);
		p3.y() += dy * sin(MapSelf->AreaRotation.angle() + 0.5*M_PI);

//		p0 << config.WallSigma, 				config.WallSigma, 0;
//		p1 << config.MaxX-config.WallSigma, 	config.WallSigma, 0;
//		p2 << config.MaxX-config.WallSigma, 	config.MaxY-config.WallSigma, 0;
//		p3 << config.WallSigma, 				config.MaxY-config.WallSigma, 0;
	}

	{
		boost::interprocess::scoped_lock<MapMutexType> lockWall(*MutexFitnessWall);

		FitnessWallVec->clear();

		float amplitude = 10.0/(3*config.WallSigma*3*config.WallSigma);

		FitnessQuadraticWall wall(0, FITSRC_EDGE, p0, p1, amplitude);
		FitnessWallVec->push_back(wall);
		std::cout << "wall=[" << wall.StartPoint[0].transpose() << "] [" << wall.StartPoint[1].transpose() << "] " << wall.Amplitude << std::endl;
//		Position checkPos;
//		checkPos << config.MaxX/2, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX/2, 0, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX/2, -config.WallSigma, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		std::cout<<std::endl;

		wall.StartPoint[0] = p1;
		wall.StartPoint[1] = p2;
		FitnessWallVec->push_back(wall);
		std::cout << "wall=[" << wall.StartPoint[0].transpose() << "] [" << wall.StartPoint[1].transpose() << "] " << wall.Amplitude << std::endl;
//		checkPos << config.MaxX/2, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX+config.WallSigma, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		std::cout<<std::endl;

		wall.StartPoint[0] = p2;
		wall.StartPoint[1] = p3;
		FitnessWallVec->push_back(wall);
		std::cout << "wall=[" << wall.StartPoint[0].transpose() << "] [" << wall.StartPoint[1].transpose() << "] " << wall.Amplitude << std::endl;
//		checkPos << config.MaxX/2, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX/2, config.MaxY, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << config.MaxX/2, config.MaxY+config.WallSigma, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		std::cout<<std::endl;

		wall.StartPoint[0] = p3;
		wall.StartPoint[1] = p0;
		FitnessWallVec->push_back(wall);
		std::cout << "wall=[" << wall.StartPoint[0].transpose() << "] [" << wall.StartPoint[1].transpose() << "] " << wall.Amplitude << std::endl;
//		checkPos << config.MaxX/2, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << 0, config.MaxX/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		checkPos << -config.WallSigma, config.MaxY/2, 0;
//		std::cout << "wall at [" << checkPos.transpose() <<"]=" << wall.GetValue(checkPos) << std::endl;
//		std::cout<<std::endl;
	}

/*
	{
		boost::interprocess::scoped_lock<MapMutexType> lockStatic(*MutexFitnessStatic);
		// Add a gausian that makes the center attractive
		float amplitude = -2.0;
		float sigmaX = 100.0;
		float sigmaY = 100.0;
		float rotation = 0.0;
		Position center;
		center << fieldSize/4, fieldSize*3/4, 0; // left top quarter of the map
		//center << (max_x + min_x)/2, (max_y + min_y)/2, 0; // Center of the map
		// Max val of -1, makes it a nice plateau
		FitnessGaussian2D gaussianCenter(0, FITSRC_STATIC, center, amplitude, sigmaX, sigmaY, rotation, 0.0, -1.0);
		FitnessStaticVec->push_back(gaussianCenter);
	}
*/
}
