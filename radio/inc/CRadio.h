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

#include "RadioStructs.h"
#include <iosfwd>
#include <sstream>
#include <iostream>
#include "Print.hpp"
#include "CTime.h"

#include "Protocol.h"
#include "StructToFromCont.h"
#include "Defs.h"

#include <radio.h>
#include <deque>
#include "BufferedAsyncSerial.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

/**
 * Serial settings for the Myrianed radio are:
 * 115200 baud
 * 8 data bits
 * no parity
 * 1 stop bit
 */



struct RadioConfig
{
	long TickTime; // us
	bool Debug;
	std::string PortName;
//	long MsgPlannerTickTime; // us
	long BufStatusIntervalTime; // ms

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("radio.TickTime");
		Debug = pt.get<bool>("radio.Debug");
		PortName = pt.get<std::string>("radio.PortName");
//		MsgPlannerTickTime = pt.get<long>("msgPlanner.TickTime");
		BufStatusIntervalTime = pt.get<long>("radio.BufStatusIntervalTime");
	}
};

class CRadio : public radio
{
	private:
		std::string ModuleId;

		int UavId;

		int* IntMsg;

		VecMsgType* VecMsg;

//		long LastSentBufStatusTime; // us
		long LastSentBufStatusTime; // ms
		long LastWriteTime; // ms

//		ERadioRoundState RadioRoundState;

		BufferedAsyncSerial *Serial;

		RadioConfig config;

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

	protected:
		// Calculate the checksum
		uint16_t CRC(const char *data, const int length, const uint8_t *precession, const int p_length,
				const uint8_t *succession, const int s_length);

		bool SynchronizeUart(RadioMsgHeader& msgHdr);

		// Reads and parses uart, returns true when whole message was read.
		bool ReadUart();

		//!
		bool ReadData(char* data, size_t size);

		bool ReadReceiveBuffer();

		void WriteToOutBuffer(VecMsgType* vecMsg);

		void WriteData(const char* data, ssize_t length);
	public:
		// Data
		std::deque<RadioMsg> SendBuffer;
		std::deque<RadioMsg> ReceiveBuffer;
		
		// Constructors
		CRadio();

		~CRadio();

		void Init(std::string &module_id);

		void Power(bool enable);

		// Functions
		void Tick();

		// Reads a (part of a) msg from the radio chip to buffer, returns true when whole message was read
		bool ReadFromRadio();
		
		// Writes a msg from buffer to the radio chip
		void WriteToRadio();
};

}

#endif // CRADIO_H_
