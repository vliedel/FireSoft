/**
 * @brief 
 * @file CSensorCO.h
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

#ifndef CSENSORCO_H_
#define CSENSORCO_H_

#include "sensorCO.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "BufferedAsyncSerial.h"

#define SENSOR_CO_BAUDRATE 9600
#define SENSOR_CO_READBUF_SIZE 64

enum ESensorCOState {
	SENSOR_CO_STATE_OFF=0,
	SENSOR_CO_STATE_COLD,
	SENSOR_CO_STATE_HEATING,
	SENSOR_CO_STATE_READY,
	SENSOR_CO_STATE_PRINT
};


namespace rur {

struct SensorCOConfig
{
	long TickTime;
	int Debug;
	std::string PortName;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("sensorCO.TickTime");
		Debug = pt.get<int>("sensorCO.Debug");
		PortName = pt.get<std::string>("sensorCO.PortName");
	}
};

class CSensorCO : public sensorCO
{
	private:

	SensorCOConfig config;
	BufferedAsyncSerial *Serial;
	char ReadBuf[SENSOR_CO_READBUF_SIZE];
	ESensorCOState State;

	void Write(const char c);

	public:
		~CSensorCO();
		void Init(std::string module_id);
		void Tick();
		void Power(bool enable);
};

}
#endif // CSENSORCO_H_
