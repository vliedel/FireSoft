/**
 * @brief Radio class: can receive and send messages via the radio.
 * @file CRadio.h
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
 * @date          Apr 24, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef CRADIOSIM_H_
#define CRADIOSIM_H_

#include <radiosim.h>
#include "StructToFromCont.h"
#include "Defs.h"
#include <deque>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

//typedef std::vector<float> RadioMsgVec;
typedef std::deque<RadioMsg> RadioMsgBuf;

struct RadioSimConfig
{
	long TickTime; // us
	int Debug;
//	long MsgPlannerTickTime; // us
	long BufStatusIntervalTime; // ms

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("radioSim.TickTime");
		Debug = pt.get<int>("radioSim.Debug");
//		MsgPlannerTickTime = pt.get<long>("msgPlanner.TickTime");
		BufStatusIntervalTime = pt.get<long>("radio.BufStatusIntervalTime");
	}
};

class CRadioSim : public radiosim
{
	private:
		// Data
		//std::deque<RadioMsgVec> SendBuffer;
		//std::deque<RadioMsgVec> ReceiveBuffer;
		RadioMsgBuf SendBuffer;
		RadioMsgBuf ReceiveBuffer;

		RadioSimConfig config;
		std::string ModuleId;
		int UavId;

		ERadioRoundState RadioRoundState;

		int* IntMsg;
		VecMsgType* VecMsg;
//		long LastSentBufStatusTime; // us
		long LastSentBufStatusTime; // ms


		// Functions

		// Reads msgs from the buffer, relays it to other modules, returns true if anything was read
		bool ReadReceiveBuffer();

		// Writes a msg from buffer to the radio chip, msgs other modules that it sent the msg
		void WriteToRadio();

		// Writes a msg to the outgoing buffer
		void WriteToOutBuffer(VecMsgType* vecMsg);

		// updates state (start/end of radio round), msgs state to other modules
		// Who triggers this function? should be radio chip? or clock?
		void UpdateState();
	public:
		// Data

		// Constructors
		~CRadioSim();

		// Functions
		void Init(std::string module_id);
		void Tick();

		//void SendRelayMsg(int uavID, RadioMsgRelay& msg, bool pos=true);
		//void GetBufferState();
};

}

#endif // CRADIOSIM_H_
