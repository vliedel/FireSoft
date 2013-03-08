/**
 * @brief 
 * @file CMsgPlanner.h
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

#ifndef CMSGPLANNER_H_
#define CMSGPLANNER_H_

#include "msgPlanner.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include "ShMemTypedefs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "FireMap.h"

namespace rur {

struct MsgPlannerConfig
{
	long TickTime; // us
	int Debug;
	int RelayNumCmdMsg; // Number of times to relay a command message

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("msgPlanner.TickTime");
		Debug = pt.get<int>("msgPlanner.Debug");
		RelayNumCmdMsg = pt.get<int>("msgPlanner.RelayNumCmdMsg");
	}
};

class CMsgPlanner : public msgPlanner
{
	//typedef boost::interprocess::interprocess_mutex UAVMapMutexType;

	public:
		~CMsgPlanner();
		void Init(std::string module_id);
		void Tick();

	private:
		// Data
		MapShMemType* ShMemUavs;
		MapUavType* MapUavs;
		MapMutexType* MutexUavs;
		std::string ShMemNameUavs;

		MapShMemType* ShMemSelf;
		MapSelfStruct* MapSelf;
		MapMutexType* MutexSelf;
		std::string ShMemNameSelf;

		MapShMemType* ShMemFire;
		MapFireType* MapFire;
		MapMutexType* MutexFire;
		std::string ShMemNameFire;

		MsgPlannerConfig config;
		std::string ModuleId;

		//UavStruct SelectedMsgs[RADIO_NUM_RELAY_PER_MSG];
		//RadioMsgRelay SelectedMsgs[RADIO_NUM_RELAY_PER_MSG];
		RadioMsg SelectedRadioMsg;

		int* IntMsg;
		std::vector<int>* VecMsg;

		bool SendOwnPos;
		bool RelayPos;
		RadioMsgRelayCmd GsCmds[MAPSELF_GS_CMDS_HIST];

		// Functions
		//void OpenShMem();
		void SelectMsgs();
		void SendMsgs();
};

}
#endif // CMSGPLANNER_H_
