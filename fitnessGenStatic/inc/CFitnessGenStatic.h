/**
 * @brief 
 * @file CFitnessGenStatic.h
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

#ifndef CFITNESSGENSTATIC_H_
#define CFITNESSGENSTATIC_H_

#include "fitnessGenStatic.h"
#include "Defs.h"
#include "Fitness.h"
#include "ShMemTypedefs.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct FitnessGenStaticConfig
{
	long TickTime; // us
	bool Debug;
	long IntervalTime;
//	float MaxX, MaxY; // Size of the field
	float WallAmplitude; // Value of wall at 3 sigma
	float WallSigma;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("fitnessGenStatic.TickTime");
		Debug = pt.get<bool>("fitnessGenStatic.Debug");
		IntervalTime = pt.get<long>("fitnessGenStatic.IntervalTime");
//		MaxX = pt.get<float>("field.MaxX");
//		MaxY = pt.get<float>("field.MaxY");
		WallAmplitude = pt.get<float>("fitnessGenStatic.WallAmplitude");
		WallSigma = pt.get<float>("fitnessGenStatic.WallSigma");
	}
};

class CFitnessGenStatic : public fitnessGenStatic
{
	public:
		~CFitnessGenStatic();
		void Init(std::string module_id);
		void Tick();

	private:
		std::string			ShMemNameFitness;
		MapShMemType*		ShMemFitness;
		FitnessVecType*		FitnessStaticVec;
		FitnessWallVecType*	FitnessWallVec;
		MapMutexType*		MutexFitnessStatic;
		MapMutexType*		MutexFitnessWall;

		std::string			ShMemNameSelf;
		MapShMemType*		ShMemSelf;
		MapSelfStruct*		MapSelf;
		MapMutexType*		MutexSelf;

		long LastTimeGen;
		FitnessGenStaticConfig config;

		void GenFitness();
};

}
#endif // CFITNESSGENSTATIC_H_
