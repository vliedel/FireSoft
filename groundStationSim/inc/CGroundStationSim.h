/**
 * @brief 
 * @file CGroundStationSim.h
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

#ifndef CGROUNDSTATIONSIM_H_
#define CGROUNDSTATIONSIM_H_

#include "groundStationSim.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <deque>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "AutoPilotProt.h"

namespace rur {

typedef std::deque<RadioMsg> RadioMsgBuf;

struct GroundStationSimConfig
{
	long TickTime;
	bool Debug;

	float OriginX; // Mercator coordinates
	float OriginY; // Mercator coordinates
	float AreaOriginX;
	float AreaOriginY;
	float AreaSizeX;
	float AreaSizeY;
	float AreaRotation;
	float LandPointX;
	float LandPointY;
	float LandHeading;
	bool LandLeftTurn;
	float MinHeight;
	float MaxHeight;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("groundStationSim.TickTime");
		Debug = pt.get<bool>("groundStationSim.Debug");

		OriginX = pt.get<float>("field.OriginX");
		OriginY = pt.get<float>("field.OriginY");
		AreaOriginX = pt.get<float>("field.AreaOriginX");
		AreaOriginY = pt.get<float>("field.AreaOriginY");
		AreaSizeX = pt.get<float>("field.AreaSizeX");
		AreaSizeY = pt.get<float>("field.AreaSizeY");
		AreaRotation = pt.get<float>("field.AreaRotation");
		LandPointX = pt.get<float>("field.LandPointX");
		LandPointY = pt.get<float>("field.LandPointY");
		LandHeading = pt.get<float>("field.LandHeading");
		LandLeftTurn = pt.get<bool>("field.LandLeftTurn");
		MinHeight = pt.get<float>("UAV.MinHeight");
		MaxHeight = pt.get<float>("UAV.MaxHeight");
	}
};

class CGroundStationSim : public groundStationSim
{
	private:
		ERadioRoundState RadioRoundState;
		GroundStationSimConfig config;
		std::string ModuleId;
		int UavId;
		VecMsgType* VecMsg;
		RadioMsgBuf SendBuffer;
		RadioMsgBuf ReceiveBuffer;
		RadioMsg CmdMsg;

		// Reads msgs from the buffer, relays it to other modules, returns true if anything was read
		void ReadReceiveBuffer();

		// Writes a msg from buffer to the radio chip, msgs other modules that it sent the msg
		void WriteToRadio();

		// Writes a msg to the outgoing buffer
		void WriteToOutBuffer(RadioMsg& msg);

	public:
		~CGroundStationSim();
		void Init(std::string module_id);
		void Tick();
};

}
#endif // CGROUNDSTATIONSIM_H_
