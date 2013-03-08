/**
 * @brief 
 * @file CMapFire.h
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

#ifndef CMAPFIRE_H_
#define CMAPFIRE_H_

#include "mapFire.h"
//#include "StructToFromCont.h"
#include "Defs.h"
#include "ShMemTypedefs.h"
#include "FireMap.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

struct MapFireConfig
{
	long TickTime; // us
	int Debug;
	long MapSize; // bytes

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("mapFire.TickTime");
		Debug = pt.get<int>("mapFire.Debug");
		MapSize = pt.get<long>("mapFire.MapSize");
	}
};

namespace rur {

class CMapFire : public mapFire
{
	public:
		~CMapFire();
		void Init(std::string module_id);
		void Tick();

	private:
		// Data
		MapShMemType* ShMem;
		MapMutexType* Mutex;
		MapFireType* Map;
		MapVoidAllocatorType* VoidAllocator;
		//MapFireAllocatorType* MapAllocator;
		std::string ShMemName;

		MapFireConfig config;
		std::string ModuleId;
		int UavId;

		long LastTestFireGenTime;

		int* IntMsg;
		std::vector<float>* VecMsg;

		// Functions
		void AddFire(MapFireStruct& fire);
};

}
#endif // CMAPFIRE_H_
