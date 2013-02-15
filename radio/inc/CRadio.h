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

// Serial speed (75 110 300 1200 2400 4800 9600 19200 38400 57600 115200)
#define MYRIANED_SERIAL_SPEED 115200

#define MYRIANED_HEADER 0xAA

//! Checksum polynomial
#define CRC16_POLY 0x8005

//! Checksum seed value
#define CRC_INIT 0xFFFF

struct RadioConfig
{
	long TickTime; // us
	bool Debug;
	std::string PortName;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("radio.TickTime");
		Debug = pt.get<bool>("radio.Debug");
		PortName = pt.get<std::string>("radio.PortName");
	}
};

typedef uint8_t	RadioMsgHeaderType;		// Type used for the magic number

#pragma pack(1)

struct RadioMsgHeader
{
	RadioMsgHeaderType			Header;		// Set this to some magic number: 0xAA
	uint8_t 					DataSize; 	// Length of the data following in bytes (can be 0, checksum not included)
	friend std::ostream& operator<<(std::ostream& os, const RadioMsgHeader& struc)
	{
		os << "Header=" << struc.Header << ", DataSize=" << struc.DataSize;
		return os;
	}
};

#pragma pack()

//typedef std::vector<int> RadioMsgVec;

class CRadio : public radio
{
	private:
		std::string ModuleId;

		int* IntMsg;

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

		void ReadUart();

		//!
		bool ReadData(char* data, size_t size);

		bool ReadReceiveBuffer();
	public:
		// Data
//		std::deque<RadioMsg> sendBuffer;
		std::deque<RadioMsg> ReceiveBuffer;
		
		// Constructors
		CRadio();

		~CRadio();

		void Init(std::string &module_id);

		void Power(bool enable);

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
