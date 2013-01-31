/**
 * @brief 
 * @file CFitnessGenCoverage.h
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

#ifndef CFITNESSGENCOVERAGE_H_
#define CFITNESSGENCOVERAGE_H_

#include "fitnessGenCoverage.h"
#include "Defs.h"
#include "FitnessMap.h"
#include "ShMemTypedefs.h"
#include <deque>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct CoverageSelfTrace
{
	Position Pos;
	float Heading;
	long Time;
};

struct FitnessGenCoverageConfig
{
	long TickTime; // us
	bool Debug;
	long IntervalTime; // ms
	float CoverageAmplitude;
	float CoverageSigmaX;
	float CoverageSigmaY;
	long CoverageIntervalTime; // Time (ms) in between adding 2 gaussians for some uav
	long CoverageSelfDelayTime; // Time (ms) to wait adding gaussians on own position
	int GridSizeX; // Size of the grid with gaussians
	int GridSizeY;
//	float MaxX;
//	float MaxY;


	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("fitnessGenCoverage.TickTime");
		Debug = pt.get<bool>("fitnessGenCoverage.Debug");
		IntervalTime = pt.get<long>("fitnessGenCoverage.IntervalTime");
		CoverageAmplitude	= pt.get<float>("fitness.CoverageAmplitude");
		CoverageSigmaX		= pt.get<float>("fitness.CoverageSigmaX");
		CoverageSigmaY		= pt.get<float>("fitness.CoverageSigmaY");
		CoverageIntervalTime	= pt.get<long>("fitnessGenCoverage.CoverageIntervalTime");
		CoverageSelfDelayTime	= pt.get<long>("fitnessGenCoverage.CoverageSelfDelayTime");
		GridSizeX	= pt.get<int>("fitnessGenCoverage.GridSizeX");
		GridSizeY	= pt.get<int>("fitnessGenCoverage.GridSizeY");
//		MaxX	= pt.get<float>("field.MaxX");
//		MaxY	= pt.get<float>("field.MaxY");
	}
};

class CFitnessGenCoverage : public fitnessGenCoverage
{
	public:
		~CFitnessGenCoverage();
		void Init(std::string module_id);
		void Tick();

	private:
		std::string ShMemNameFitness;
		MapShMemType* ShMemFitness;
		FitnessMapType* FitnessMap;
		MapMutexType* MutexFitness;

		std::string ShMemNameUavs;
		MapShMemType* ShMemUavs;
		MapUavType* MapUavs;
		MapMutexType* MutexUavs;

		std::string ShMemNameSelf;
		MapShMemType* ShMemSelf;
		MapSelfStruct* MapSelf;
		MapMutexType* MutexSelf;

		FitnessGenCoverageConfig config;
		std::string ModuleId;
		int UavId;

		int* IntMsg;
		std::vector<float>* VecMsg;

		long lastAddedTime[UAVS_NUM];
		std::deque<CoverageSelfTrace> Trace;

		long LastGenTime;

		void GenFitness();
};

}
#endif // CFITNESSGENCOVERAGE_H_
