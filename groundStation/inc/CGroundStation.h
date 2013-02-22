/**
 * @brief 
 * @file CGroundStation.h
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

#ifndef CGROUNDSTATION_H_
#define CGROUNDSTATION_H_

#include "groundStation.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <deque>
#include "BufferedAsyncSerial.h"
#include "AutoPilotProt.h"
#include "RadioStructs.h"

namespace rur {

struct GroundStationConfig
{
	long TickTime;
	bool Debug;
	std::string PortName;
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
		TickTime = pt.get<long>("groundStation.TickTime");
		Debug = pt.get<bool>("groundStation.Debug");
		PortName = pt.get<std::string>("groundStation.PortName");
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

class CGroundStation : public groundStation
{
	private:
		std::string ModuleId;
		int UavId;
		GroundStationConfig config;
		int* IntMsg;
		VecMsgType* VecMsg;

		BufferedAsyncSerial *Serial;
		int fd_cts;

		//! Flag to indicate that synchronization is required (not to indicate that it is done)
		bool Synchronize;

		//! Last message header
		RadioMsgHeader LastReadHeader;

		//! True when new header should be read, false if we should use LastReadHeader
		bool LastReadHeaderUsed;

		//! Checksum (including RSSI value)
		uint16_t CheckSum;

		//! Stop byte
		uint8_t StopByte;

		//! Signal value
		uint8_t RSSI;

		long LastWriteTime;

		RadioMsg CmdMsg;
		std::deque<RadioMsg> SendBuffer;
		std::deque<RadioMsg> ReceiveBuffer;

	protected:
		// Calculate the checksum
		uint16_t CRC(const char *data, const int length, const uint8_t *precession, const int p_length,
				const uint8_t *succession, const int s_length);

		bool SynchronizeUart(RadioMsgHeader& msgHdr);

		void ReadUart();

		bool ReadData(char* data, size_t size);

		bool ReadReceiveBuffer();

		void WriteToOutBuffer(RadioMsg& msg);

		void WriteData(const char* data, ssize_t length);

	public:
		CGroundStation();
		~CGroundStation();
		void Init(std::string module_id);
		void Tick();

		// Reads (a) msg(s) from the radio chip to buffer
		void ReadFromRadio();

		// Writes a msg from buffer to the radio chip
		void WriteToRadio();
};

}
#endif // CGROUNDSTATION_H_
