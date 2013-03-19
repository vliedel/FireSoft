/**
 * @brief 
 * @file CComSalland.h
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

#ifndef CCOMSALLAND_H_
#define CCOMSALLAND_H_

#include "comSalland.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "SallandProt.h"
#include "ShMemTypedefs.h"
#include "BufferedAsyncSerial.h"


namespace rur {

struct ComSallandConfig
{
	long TickTime;
	int Debug;
	std::string PortName;
	long IntervalTimeUis; // ms

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("comSalland.TickTime");
		Debug = pt.get<int>("comSalland.Debug");
		PortName = pt.get<std::string>("comSalland.PortName");
		IntervalTimeUis = pt.get<long>("comSalland.IntervalTimeUis");
	}
};

class CComSalland : public comSalland
{
	private:
		std::string ModuleId;
		int UavId;
		std::vector<float>* VecMsg;

		std::string ShMemNameSelf;
		MapShMemType* ShMemSelf;
		MapMutexType* MutexSelf;
		MapSelfStruct* MapSelf;

		BufferedAsyncSerial *Serial;

		//! Flag to indicate that synchronization is required (not to indicate that it is done)
		bool Synchronize;

		//! Last message header
		SallandHeader LastReadHeader;

		//! True when new header should be read, false if we should use LastReadHeader
		bool LastReadHeaderUsed;

		long LastTimeWriteUis;
		ESallandState State;

		void Power(bool enable);
		bool SynchronizeUart(SallandHeader& header);
		// Reads and parses uart, returns true when whole message was read.
		bool ReadUart();
		bool ReadData(char* data, size_t size);
		void WriteData(SallandHeader& header, char* data, size_t size);

	public:
		ComSallandConfig config;

		CComSalland();
		~CComSalland();
		void Init(std::string module_id);
		void Tick();
};

}
#endif // CCOMSALLAND_H_
