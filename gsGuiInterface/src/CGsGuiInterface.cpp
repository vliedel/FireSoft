/**
 * @brief 
 * @file CGsGuiInterface.cpp
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

#include "CGsGuiInterface.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"
#include <ctime>

using namespace rur;

CGsGuiInterface::~CGsGuiInterface()
{
	delete Socket;
}

void CGsGuiInterface::Init(std::string module_id)
{
	gsGuiInterface::Init(module_id);
	config.load("config.json");

	boost::asio::io_service io_service;
	boost::asio::ip::tcp::resolver resolver(io_service);
	boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), config.Host, config.Port);
	boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);
	Socket = new boost::asio::ip::tcp::socket(io_service);
	Socket->connect(*iterator);
	std::cout << "Connected" << std::endl;

	ReadState = READSTATE_SIZE;
	BytesToRead = SIZE_SIZE;
	Size[SIZE_SIZE] = 0; // This char array is passed to atoi() so it needs an end
//	Header[HEADER_SIZE] = 0; // To make it a nice string

}

void CGsGuiInterface::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}

	// Read from GUI
	size_t availableRead = Socket->available();
	switch (ReadState)
	{
		case READSTATE_SIZE:
		{
			if (availableRead >= BytesToRead)
			{
				size_t bytesRead = Socket->read_some(boost::asio::buffer(Size, BytesToRead));
				BytesToRead -= bytesRead;
				if (BytesToRead == 0)
				{
					std::cout << "Size = " << Size << std::endl;
					JsonSize = atoi(Size) - HEADER_SIZE;
					std::cout << "Json size: " << JsonSize << std::endl;
					BytesToRead = HEADER_SIZE;
					ReadState = READSTATE_HEADER;
				}
			}
			break;
		}
		case READSTATE_HEADER:
		{
			if (availableRead >= BytesToRead)
			{
				size_t bytesRead = Socket->read_some(boost::asio::buffer(Header, BytesToRead));
				BytesToRead -= bytesRead;
				if (BytesToRead == 0)
				{
					std::cout << "Header = ";
					std::cout.write(Header, HEADER_SIZE);
					std::cout << std::endl;
					if (!strcmp(Header, "0001"))
						std::cout << "Error wrong msg type" << std::endl;
					BytesToRead = JsonSize;
					ReadState = READSTATE_JSON;
				}
			}
			break;
		}
		case READSTATE_JSON:
		{
			if (availableRead >= BytesToRead)
			{
				size_t bytesRead = Socket->read_some(boost::asio::buffer(Json, BytesToRead));
				BytesToRead -= bytesRead;
				if (BytesToRead == 0)
				{
					std::cout << "Json:" << std::endl;
					std::cout.write(Json, JsonSize);
					std::cout << std::endl;
					BytesToRead = SIZE_SIZE;
					ReadState = READSTATE_SIZE;


					boost::property_tree::ptree pt;
					std::stringstream ss;
					ss.write(Json, JsonSize);
					read_json(ss, pt);

					GsCmdStruct gsCmd;
					if (pt.get<std::string>("message_type") != "cmd")
					{
						std::cout << "Error: wrong json type" << std::endl;
						return;
					}
					std::string timeStamp = pt.get<std::string>("timestamp");
					int version = pt.get<int>("version");
					std::cout << "cmd_type=cmd" << " timestamp=" << timeStamp << " version=" << version << std::endl;

					gsCmd.UavId = pt.get<int>("uav_id");
					gsCmd.MsgId = pt.get<int>("message_id");
					gsCmd.HeightMin = pt.get<float>("min_height");
					gsCmd.HeightMax = pt.get<float>("max_height");
					gsCmd.AreaZero.x() = pt.get<float>("area_min_x") - config.OriginX; // Mercator to local coordinates
					gsCmd.AreaZero.y() = pt.get<float>("area_min_y") - config.OriginY; // Mercator to local coordinates
					gsCmd.AreaSize.x() = pt.get<float>("area_dx");
					gsCmd.AreaSize.y() = pt.get<float>("area_dy");
					gsCmd.AreaRotation.angle() = pt.get<float>("area_rotation");
					gsCmd.Landing.Pos.x() = pt.get<float>("land_x") - config.OriginX; // Mercator to local coordinates
					gsCmd.Landing.Pos.y() = pt.get<float>("land_y") - config.OriginY; // Mercator to local coordinates
					gsCmd.Landing.Heading.angle() = pt.get<float>("land_heading");
					gsCmd.Landing.LeftTurn = pt.get<bool>("turn");
					gsCmd.Mode = pt.get<int>("auto_pilot_mode");
					gsCmd.EnablePlanner = pt.get<bool>("enable_planner");

					RadioMsgRelayCmd cmdMsg;
					gsCmd.toMsg(cmdMsg);
					std::cout << "gsCmd:" << gsCmd << " = " << cmdMsg;

					VecMsgType vecMsg;
					vecMsg.push_back(RADIO_MSG_RELAY_CMD);
					ToCont(cmdMsg, vecMsg);
					writeToRadio(vecMsg);
				}
			}
			break;
		}
	}

	// Read radio
	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		std::cout << "GsGuiInterface from GroundStation: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		//std::string header;
		std::stringstream ssJson;
		std::stringstream ssOutput;

		// Variables to get formatted time
		std::stringstream ssTime;
		struct timespec tp;
		struct tm * ptm;
		char bufTime[64];

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			// Clear string streams
			ssJson.str().clear();
			ssTime.str().clear();
			ssOutput.str().clear();

			// Get current time
			clock_gettime(CLOCK_REALTIME, &tp);
			ptm = gmtime(&tp.tv_sec);
			// %Y-%m-%d %H:%M:%S.%f
			// 2008-09-03T20:56:35.450686Z
			size_t tsl = strftime(bufTime, 64, "%Y-%m-%dT%H:%M:%S", ptm);
			ssTime << bufTime << "." << tp.tv_nsec/1000 << "Z";

			int type = *it++;
			//std::cout << "Type=" << type << std::endl;
			switch (type)
			{
				case PROT_RADIO_MSG_RELAY_POS:
				{
					it = FromCont(PosMsg, it, VecMsg->end());
					Uav.FromRadioMsg(PosMsg);

					PropertyTreePos.put("message_type", "pos");
					PropertyTreePos.put("version", 1);
					PropertyTreePos.put("timestamp", ssTime.str());
					PropertyTreePos.put("uav_id", Uav.UavId);
					PropertyTreePos.put("uav_x", Uav.Geom.Pos.x() + config.OriginX); // Local to mercator coordinates
					PropertyTreePos.put("uav_y", Uav.Geom.Pos.y() + config.OriginY); // Local to mercator coordinates
					PropertyTreePos.put("uav_z", Uav.Geom.Pos.z());
					PropertyTreePos.put("heading", Uav.Geom.Heading.angle());
					PropertyTreePos.put("ground_speed", Uav.Geom.GroundSpeed);
					switch (Uav.State)
					{
					case UAVSTATE_LANDED:				PropertyTreePos.put("state", "Landed"); break;
					case UAVSTATE_TAKING_OFF:			PropertyTreePos.put("state", "Taking off"); break;
					case UAVSTATE_FLYING:				PropertyTreePos.put("state", "Flying"); break;
					case UAVSTATE_GOING_HOME:			PropertyTreePos.put("state", "Going home"); break;
					case UAVSTATE_WAITING_TO_LAND:		PropertyTreePos.put("state", "Waiting to land"); break;
					case UAVSTATE_COLLISION_AVOIDING:	PropertyTreePos.put("state", "Avoiding collision"); break;
					case UAVSTATE_CHECK_FIRE:			PropertyTreePos.put("state", "Checking fire"); break;
					case UAVSTATE_FOLLOW_FIRE:			PropertyTreePos.put("state", "Following fire"); break;
					case UAVSTATE_STAYING:				PropertyTreePos.put("state", "Staying"); break;
					}

					PropertyTreePos.put("sensed_radius", 0); // TODO: must be something hard coded or calculated
					PropertyTreePos.put("roll_angle", Uav.Geom.Roll.angle());
					PropertyTreePos.put("dx1", Uav.WpNext[0].to.x() + config.OriginX); // Local to mercator coordinates
					PropertyTreePos.put("dy1", Uav.WpNext[0].to.y() + config.OriginY); // Local to mercator coordinates
					PropertyTreePos.put("dz1", Uav.WpNext[0].to.z());
					PropertyTreePos.put("dx2", Uav.WpNext[1].to.x() + config.OriginX); // Local to mercator coordinates
					PropertyTreePos.put("dy2", Uav.WpNext[1].to.y() + config.OriginY); // Local to mercator coordinates
					PropertyTreePos.put("dz2", Uav.WpNext[1].to.z());
					PropertyTreePos.put("dx3", Uav.WpNext[2].to.x() + config.OriginX); // Local to mercator coordinates
					PropertyTreePos.put("dy3", Uav.WpNext[2].to.y() + config.OriginY); // Local to mercator coordinates
					PropertyTreePos.put("dz3", Uav.WpNext[2].to.z());
					PropertyTreePos.put("battery_left", Uav.BatteryTimeLeft);

					PropertyTreePos.put("autopilot_status", "ok"); // TODO: fill this by checking status bits
					//PropertyTreePos.put("autopilot_status", "gps error, engine error");

					write_json(ssJson, PropertyTreePos, false); // no pretty output
					break;
				}
				case PROT_RADIO_MSG_RELAY_FIRE:
				{
					it = FromCont(FireMsg, it, VecMsg->end());

					// TODO: convert to fire struct (which does the translation to floats)
					PropertyTreeFire.put("message_type", "fire");
					PropertyTreePos.put("version", 1);
					PropertyTreePos.put("timestamp", ssTime.str());
					PropertyTreeFire.put("uav_id", FireMsg.UavId);
					PropertyTreeFire.put("p_f_c", FireMsg.PCam);
					PropertyTreeFire.put("p_f_t", FireMsg.PTPA);
					PropertyTreeFire.put("p_f_g", FireMsg.PGas);
					PropertyTreeFire.put("g_x", FireMsg.X + config.OriginX); // Local to mercator coordinates
					PropertyTreeFire.put("g_y", FireMsg.Y + config.OriginY); // Local to mercator coordinates
					PropertyTreeFire.put("g_var_x", FireMsg.VarX);
					PropertyTreeFire.put("g_var_y", FireMsg.VarY);
					PropertyTreeFire.put("g_var_rot", FireMsg.Rot);
					PropertyTreeFire.put("uav_z", FireMsg.Z);

					write_json(ssJson, PropertyTreeFire, false); // no pretty output
					break;
				}
			}

			if (ssJson.str().size() > 0)
			{
				ssOutput << std::setw(SIZE_SIZE) << std::setfill('0') << ssJson.str().size() + HEADER_SIZE;
				ssOutput << config.DataType << ssJson.str();
				boost::asio::write(*Socket, boost::asio::buffer(ssOutput.str(), ssOutput.str().size()));
			}
		}
		VecMsg->clear();
	}

	usleep(config.TickTime);
}
