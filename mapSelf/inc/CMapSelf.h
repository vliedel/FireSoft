/**
 * @brief 
 * @file CMapSelf.h
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

#ifndef CMAPSELF_H_
#define CMAPSELF_H_

// Size of shared memory in bytes, sizeof(MapSelfStruct) = 1012 at this time of writing
//#define MAP_SELF_MEMSIZE 1024*16

#include "mapSelf.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include "ShMemTypedefs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct MapSelfConfig
{
	long TickTime; // us
	int Debug;
	long MapSize; // bytes
	float BatteryTime; // seconds

//	float OriginX; // Mercator coordinates
//	float OriginY; // Mercator coordinates
	float AreaOriginX;
	float AreaOriginY;
	float AreaSizeX;
	float AreaSizeY;
	float AreaRotation;
	float LandPointX;
	float LandPointY;
	float LandHeading;
	bool LandLeftTurn;
	float LandLength;
	float LandRadius;
	float MinHeight;
	float MaxHeight;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("mapSelf.TickTime");
		Debug = pt.get<int>("mapSelf.Debug");
		MapSize = pt.get<long>("mapSelf.MapSize");
		BatteryTime = pt.get<float>("UAV.BatteryTime");

//		OriginX = pt.get<float>("field.OriginX");
//		OriginY = pt.get<float>("field.OriginY");
		AreaOriginX = pt.get<float>("field.AreaOriginX");
		AreaOriginY = pt.get<float>("field.AreaOriginY");
		AreaSizeX = pt.get<float>("field.AreaSizeX");
		AreaSizeY = pt.get<float>("field.AreaSizeY");
		AreaRotation = pt.get<float>("field.AreaRotation");
		LandPointX = pt.get<float>("field.LandPointX");
		LandPointY = pt.get<float>("field.LandPointY");
		LandHeading = pt.get<float>("field.LandHeading");
		LandLeftTurn = pt.get<bool>("field.LandLeftTurn");
		LandLength = pt.get<float>("field.LandLength");
		LandRadius = pt.get<float>("field.LandRadius");
		MinHeight = pt.get<float>("UAV.MinHeight");
		MaxHeight = pt.get<float>("UAV.MaxHeight");
	}
};

class CMapSelf : public mapSelf
{
	public:
		~CMapSelf();
		void Init(std::string module_id);
		void Tick();

	private:
		// Data
		MapShMemType* ShMem;
		MapSelfStruct* Map;
		MapMutexType* Mutex;
		std::string ShMemName;

		MapSelfConfig config;
		std::string ModuleId;

		int* IntMsg;
		std::vector<float>* VecMsg;

		// Functions
};

}
#endif // CMAPSELF_H_
