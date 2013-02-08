/**
 * @brief 
 * @file CSim.h
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

#ifndef CSIM_H_
#define CSIM_H_

#include "sim.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct SimConfig
{
	long TickTime; // us
	bool Debug;
	float TimeStep;
	float RadioRoundTime;
	float RadioRange;
	std::string OutputFileName;
	float GroundStationX;
	float GroundStationY;


	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("sim.TickTime");
		Debug = pt.get<bool>("sim.Debug");
		TimeStep = pt.get<float>("sim.TimeStep");
		RadioRoundTime = pt.get<float>("sim.RadioRoundTime");
		RadioRange = pt.get<float>("UAV.RadioRange");
		OutputFileName = pt.get<std::string>("sim.OutputFileName");
		GroundStationX = pt.get<float>("field.GroundStationX");
		GroundStationY = pt.get<float>("field.GroundStationY");
	}
};

class CSim : public sim
{
	typedef std::vector<SimUavStruct>::iterator UavIterType;

	public:
		std::vector<SimUavStruct> Uavs;

		~CSim();
		void Init(std::string module_id, int numUavsAP, int numUavsRadio, int simTime);
		void Tick();

	private:
		int* IntMsg;
		VecMsgType* VecMsg;
		SimConfig config;

		bool Running;
		std::ofstream FileOut;
		float SimTime;
//		float TimeStep;
//		float RadioRoundTime;
//		float RadioDistance;
		float LastRadioRoundStartTime;
		ERadioRoundState RadioRoundState;
		//int RadioRoundLastUav;
		float CurTime;
		long StartTimeStamp;
		long LastTimeStepTimeStamp;

		VecMsgType BroadcastMsgs[UAVS_NUM+1]; // 1 for ground station

		void CmdInit();
		void CmdStart();
		void CmdStop();
		void CmdLand();

		// Message read/write functions with uav ID added
		void writeRadioCommand(int uavId, const VecMsgType& vecMsg);
		VecMsgType* readRadioState(int uavId, bool blocking=true);
		void writeAutoPilotCommand(int uavId, const VecMsgType& vecMsg);
		VecMsgType* readAutoPilotState(int uavId, bool blocking=true);

};

}
#endif // CSIM_H_
