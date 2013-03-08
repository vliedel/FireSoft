/**
 * @brief 
 * @file CFitnessGenCollision.h
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

#ifndef CFITNESSGENCOLLISION_H_
#define CFITNESSGENCOLLISION_H_

#include "fitnessGenCollision.h"
#include "Defs.h"
#include "Fitness.h"
#include "ShMemTypedefs.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct FitnessGenCollisionConfig
{
	long TickTime; // us
	int Debug;
	long IntervalTime;
	float CollisionAmplitude;
	float CollisionSigmaX;
	float CollisionSigmaY;
	float CollisionSigmaZ;
	float PredictAheadTime; // Time ahead in the current heading where we put a guassian
//	float CruiseSpeed;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("fitnessGenCollision.TickTime");
		Debug = pt.get<int>("fitnessGenCollision.Debug");
		IntervalTime = pt.get<long>("fitnessGenCollision.IntervalTime");
		CollisionAmplitude = pt.get<float>("fitness.CollisionAmplitude");
		CollisionSigmaX = pt.get<float>("fitness.CollisionSigmaX");
		CollisionSigmaY = pt.get<float>("fitness.CollisionSigmaY");
		CollisionSigmaZ = pt.get<float>("fitness.CollisionSigmaZ");
		PredictAheadTime = pt.get<float>("fitnessGenCollision.PredictAheadTime");
//		CruiseSpeed = pt.get<float>("UAV.CruiseSpeed");
	}
};

class CFitnessGenCollision : public fitnessGenCollision
{
	public:
		~CFitnessGenCollision();
		void Init(std::string module_id);
		void Tick();

	private:
		std::string ShMemNameFitness;
		MapShMemType* ShMemFitness;
		Fitness3DVecType* FitnessVec;
		MapMutexType* MutexFitness;

		std::string ShMemNameUavs;
		MapShMemType* ShMemUavs;
		MapUavType* MapUavs;
		MapMutexType* MutexUavs;

		//std::string ModuleId;
		//int UavId;

		FitnessGenCollisionConfig config;
		int* IntMsg;
		std::vector<float>* VecMsg;

		long LastGenTime;

		void GenFitness();
};

}
#endif // CFITNESSGENCOLLISION_H_
