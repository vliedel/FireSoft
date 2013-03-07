/**
 * @brief 
 * @file CGsVisualizer.cpp
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

#include "CGsVisualizer.h"
#include "Protocol.h"
#include "CTime.h"

using namespace rur;

CGsVisualizer::~CGsVisualizer()
{
	delete ShMemUavs;
}

void CGsVisualizer::Init(std::string module_id)
{
	gsVisualizer::Init(module_id);
	ShMemNameUavs = "mapUAV_" + module_id;
	config.load("config.json");

	try
	{
		// Open the shared memory
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());

		// Find the maps and mutexes using the c-string name
		MapUavs =			ShMemUavs->find<MapUavType>("Map").first;
		MutexUavs = 		ShMemUavs->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		MapUavs = NULL;
		MutexUavs = NULL;
		delete ShMemUavs;
		std::cout << "Error in " << ShMemNameUavs << std::endl;
		throw;
	}

	FileOutPos.open(config.OutputFilePosName.c_str());
	LastOutputPosTime = get_cur_1ms();
}

void CGsVisualizer::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//	}

	if (get_duration(LastOutputPosTime, get_cur_1ms()) > config.OutputPosIntervalTime)
	{
		LastOutputPosTime = get_cur_1ms();

		boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);

		if (MapUavs->size() < 1)
			return;

		FileOutPos << get_cur_1ms() << " ";
		MapUavIterType it;
		for (it=MapUavs->begin(); it != MapUavs->end(); ++it)
		{
			FileOutPos << it->second.data.UavId << " " \
					<< it->second.data.Geom.Pos.x() << " " << it->second.data.Geom.Pos.y() << " " << it->second.data.Geom.Pos.z() << " " \
					<< it->second.data.Geom.Heading.angle() << " " \
					<< "0 0 0 " \
					<< "0 ";

		}
		FileOutPos << std::endl;
	}

	usleep(config.TickTime);
}
