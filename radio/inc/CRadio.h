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

#ifndef CRADIO_H_
#define CRADIO_H_

//#include "RadioStructs.h"
#include <radio.h>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct RadioConfig
{
	long TickTime; // us
	bool Debug;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("radio.TickTime");
		Debug = pt.get<bool>("radio.Debug");
	}
};

typedef std::vector<int> RadioMsgVec;

class CRadio : public radio
{
	public:
		// Data
		std::vector<RadioMsgVec> sendBuffer;
		std::vector<RadioMsgVec> receiveBuffer;
		
		// Constructors
		~CRadio();

		// Functions
		void Tick();

		// Reads (a) msg(s) from the radio chip, relays it to other modules
		void ReadFromRadio();
		
		// Writes a msg from buffer to the radio chip, msgs other modules that it sent the msg
		void WriteToRadio();
		
		// updates state (start/end of radio round), msgs state to other modules
		// Who triggers this function? should be radio chip? or clock?
		void UpdateState();
		
		//void SendRelayMsg(int uavID, RadioMsgRelay& msg, bool pos=true);
		//void GetBufferState();
};

}

#endif // CRADIO_H_
