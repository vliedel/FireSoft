/**
 * @brief 
 * @file CGsVisualizer.h
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

#ifndef CGSVISUALIZER_H_
#define CGSVISUALIZER_H_

#include "gsVisualizer.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "ShMemTypedefs.h"

namespace rur {

struct GsVisualizerConfig
{
	long TickTime;
	bool Debug;

	std::string OutputFilePosName;
	long OutputPosIntervalTime;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("gsVisualizer.TickTime");
		Debug = pt.get<bool>("gsVisualizer.Debug");

		OutputFilePosName = pt.get<std::string>("gsVisualizer.OutputFilePosName");
		OutputPosIntervalTime = pt.get<long>("gsVisualizer.OutputPosIntervalTime");
	}
};

class CGsVisualizer : public gsVisualizer
{
	private:
		std::ofstream FileOutPos;
		long LastOutputPosTime;

		std::string ShMemNameUavs;
		MapShMemType* ShMemUavs;
		MapUavType* MapUavs;
		MapMutexType* MutexUavs;

	public:
		GsVisualizerConfig config;

		~CGsVisualizer();
		void Init(std::string module_id);
		void Tick();
};

}
#endif // CGSVISUALIZER_H_
