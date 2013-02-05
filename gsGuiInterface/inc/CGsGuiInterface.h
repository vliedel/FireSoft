/**
 * @brief 
 * @file CGsGuiInterface.h
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

#ifndef CGSGUIINTERFACE_H_
#define CGSGUIINTERFACE_H_

#include "gsGuiInterface.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/asio.hpp>

namespace rur {

struct GsGuiInterfaceConfig
{
	long TickTime;
	bool Debug;
	std::string Host;
	std::string Port;
	std::string DataType;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("gsGuiInterface.TickTime");
		Debug = pt.get<bool>("gsGuiInterface.Debug");
		Host = pt.get<std::string>("gsGuiInterface.Host");
		Port = pt.get<std::string>("gsGuiInterface.Port");
		DataType = pt.get<std::string>("gsGuiInterface.DataType");
	}
};

class CGsGuiInterface : public gsGuiInterface
{
	private:
		boost::property_tree::ptree PropertyTreePos;
		boost::property_tree::ptree PropertyTreeFire;
		boost::asio::ip::tcp::socket* Socket;
		//boost::asio::ip::tcp::iostream* socket;

		VecMsgType* VecMsg;
		RadioMsgRelayPos PosMsg;
		UavStruct Uav;
		RadioMsgRelayFire FireMsg;



	public:
		GsGuiInterfaceConfig config;

		~CGsGuiInterface();
		void Init(std::string module_id);
		void Tick();
};

}
#endif // CGSGUIINTERFACE_H_
