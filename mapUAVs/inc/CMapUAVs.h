/**
 * @brief 
 * @file CMapUAVs.h
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

#ifndef CMAPUAVS_H_
#define CMAPUAVS_H_

// Size of shared memory in bytes
//#define MAP_UAV_MEMSIZE 1024*1024 // uav is about 784 B data

#include "mapUAVs.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include "ShMemTypedefs.h"
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct MapUAVsConfig
{
	long TickTime; // us
	bool Debug;
	long MapSize; // bytes

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("mapUAVs.TickTime");
		Debug = pt.get<bool>("mapUAVs.Debug");
		MapSize = pt.get<long>("mapUAVs.MapSize");
	}
};

class CMapUAVs : public mapUAVs
{
	public:
		// Constructors
		~CMapUAVs();

		// Functions
		void Init(std::string module_id);
		void Tick();
		void AddUAV(std::vector<float>* vec);
		void AddUAV(UavStruct* uav);
		void UpdateUav(RadioMsgRelay& msg);
		//void RemoveUAV(int id);
		friend std::ostream& operator<<(std::ostream& os, CMapUAVs& map);

	private:
		// Data
		MapShMemType* ShMem;
		MapMutexType* Mutex;
		MapUavType* Map;
		MapUavAllocatorType* MapAllocator;
		std::string ShMemName;

		MapUAVsConfig config;
		std::string ModuleId;

		int* IntMsg;
		std::vector<float>* VecMsg;

		UavStruct Uav;

		// Functions
};

}
#endif /* CMAPUAVS_H_ */
