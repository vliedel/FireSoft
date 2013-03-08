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

#define SIZE_SIZE 9
#define HEADER_SIZE 4

#define JSON_SIZE_MAX 1024

namespace rur {

enum EReadState {
	READSTATE_SIZE=0,
	READSTATE_HEADER,
	READSTATE_JSON
};

struct GsGuiInterfaceConfig
{
	long TickTime;
	int Debug;
	std::string Host;
	std::string Port;
	std::string DataType;
	float OriginX; // Mercator coordinates
	float OriginY; // Mercator coordinates

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("gsGuiInterface.TickTime");
		Debug = pt.get<int>("gsGuiInterface.Debug");
		Host = pt.get<std::string>("gsGuiInterface.Host");
		Port = pt.get<std::string>("gsGuiInterface.Port");
		DataType = pt.get<std::string>("gsGuiInterface.DataType");
		OriginX = pt.get<float>("field.OriginX");
		OriginY = pt.get<float>("field.OriginY");
	}
};

class CGsGuiInterface : public gsGuiInterface
{
	private:
		boost::property_tree::ptree PropertyTreePos;
		boost::property_tree::ptree PropertyTreeFire;
		boost::asio::ip::tcp::socket* Socket;

//		boost::asio::io_service IoService;
//		boost::asio::ip::tcp::resolver* Resolver;
//		boost::asio::ip::tcp::resolver::query* Query;
//		boost::asio::ip::tcp::resolver::iterator ResolveIt;

		//boost::asio::ip::tcp::iostream* socket;

		VecMsgType* VecMsg;
		RadioMsgRelayPos PosMsg;
		UavStruct Uav;
		RadioMsgRelayFire FireMsg;


		char Header[HEADER_SIZE];
		char Size[SIZE_SIZE+1];
		char Json[JSON_SIZE_MAX];
		uint8_t ReadState;
		size_t BytesToRead;
		size_t JsonSize;

		// Functions
		bool Connect();
		bool Read(size_t& bytesRead, const boost::asio::mutable_buffers_1& buffer);
		bool Write(const boost::asio::const_buffers_1& buffer);
		bool Available(size_t& numBytes);
		void ReadGui();
//		void HandleRead(const boost::system::error_code& error, size_t bytesRead);


	public:
		GsGuiInterfaceConfig config;

		~CGsGuiInterface();
		void Init(std::string module_id);
		void Tick();
};

}
#endif // CGSGUIINTERFACE_H_
