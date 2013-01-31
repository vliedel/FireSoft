/**
 * @brief 
 * @file CFitnessGenBattery.h
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

#ifndef CFITNESSGENBATTERY_H_
#define CFITNESSGENBATTERY_H_

#include "fitnessGenBattery.h"
#include "Defs.h"
#include "Fitness.h"
#include "ShMemTypedefs.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct FitnessGenBatteryConfig
{
	long TickTime; // us
	bool Debug;
	long IntervalTime;
	float CruiseSpeed;
	float BatteryLowTime;
//	float MaxX, MaxY;
//	float HomeX, HomeY;
	float BatteryAmplitude;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime		= pt.get<long>("fitnessGenBattery.TickTime");
		Debug			= pt.get<bool>("fitnessGenBattery.Debug");
		IntervalTime	= pt.get<long>("fitnessGenBattery.IntervalTime");
		CruiseSpeed			= pt.get<float>("UAV.CruiseSpeed");
		BatteryLowTime		= pt.get<float>("UAV.BatteryLowTime");
//		MaxX	= pt.get<float>("field.MaxX");
//		MaxY	= pt.get<float>("field.MaxY");
//		HomeX	= pt.get<float>("field.HomeX");
//		HomeY	= pt.get<float>("field.HomeY");
		BatteryAmplitude	= pt.get<float>("fitness.BatteryAmplitude");
	}
};

class CFitnessGenBattery : public fitnessGenBattery
{
	public:
		~CFitnessGenBattery();
		void Init(std::string module_id);
		void Tick();

		std::string			ShMemNameFitness;
		MapShMemType*		ShMemFitness;
		FitnessVecType*		FitnessVec;
		MapMutexType*		MutexFitness;

		std::string			ShMemNameSelf;
		MapShMemType*		ShMemSelf;
		MapSelfStruct*		MapSelf;
		MapMutexType*		MutexSelf;

		FitnessGenBatteryConfig config;

		long LastGenTime;

		void GenFitness();
};

}
#endif // CFITNESSGENBATTERY_H_
