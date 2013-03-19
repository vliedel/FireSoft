/**
 * @brief 
 * @file CMapSelf.cpp
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

#include "CMapSelf.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"
#include "AutoPilotProt.h"

using namespace rur;

CMapSelf::~CMapSelf()
{
	delete ShMem;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
}

void CMapSelf::Init(std::string module_id)
{
	mapSelf::Init(module_id);
	config.load("config.json");

	ModuleId = module_id;
	int id;

	//Shared memory front-end that is able to construct objects
	//associated with a c-string. Erase previous shared memory with the name
	//to be used and create the memory segment at the specified address and initialize resources
	ShMemName = "mapSelf_" + module_id;
	boost::interprocess::shared_memory_object::remove(ShMemName.c_str());

	std::cout << "sizeof(MapSelfStruct) = " << sizeof(MapSelfStruct) << std::endl;

	try
	{
		ShMem = new MapShMemType(boost::interprocess::create_only, ShMemName.c_str(), config.MapSize);

		//Construct a shared memory map.
		Map =
			ShMem->construct<MapSelfStruct>("Map")	//object name
			();		//first ctor parameter

		//Mutex = new MapMutexType();
		Mutex = ShMem->construct<MapMutexType>("Mutex") ();
		id = atoi(module_id.c_str());
	}
	catch(...)
	{
		Map = NULL;
		Mutex = NULL;
		delete ShMem;
		boost::interprocess::shared_memory_object::remove(ShMemName.c_str());
		std::cout << "Error in " << ShMemName << std::endl;
		throw;
	}

	// Init data
	{
		boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
		Map->UavData.State = UAVSTATE_LANDED;
		Map->UavData.Geom.Init();
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			Map->UavData.WpNext[i].Init();
		Map->UavData.UavId = id;
		Map->PreviousState = UAVSTATE_LANDED;
		Map->UavData.BatteryTimeLeft = config.BatteryTime;
		// Init ground station command messages with invalid ids
		for (int i=0; i<MAPSELF_GS_CMDS_HIST; ++i)
			Map->LastGsCmds[i].Msg.UavId = 0; // Invalid msg
		Map->LastGsCmdsIndex = 0;

		Map->GsCmd.HeightMin = config.MinHeight;
		Map->GsCmd.HeightMax = config.MaxHeight;
		Map->GsCmd.AreaZero << config.AreaOriginX, config.AreaOriginY, 0;
		Map->GsCmd.AreaSize << config.AreaSizeX, config.AreaSizeY, 0;
		Map->GsCmd.AreaRotation.angle() = config.AreaRotation;
		Map->GsCmd.Landing.Pos << config.LandPointX, config.LandPointY, 0;
		Map->GsCmd.Landing.Heading.angle() = config.LandHeading;
		Map->GsCmd.Landing.LeftTurn = config.LandLeftTurn;
		Map->GsCmd.Landing.Length = config.LandLength;
		Map->GsCmd.Landing.Radius = config.LandRadius;
		Map->GsCmd.Mode = AP_PROT_MODE_WP;
		Map->GsCmd.EnablePlanner = true;
	}
}

void CMapSelf::Tick()
{
//	IntMsg = readCommand(false);
//	if (IntMsg != NULL)
//	{
//
//	}

	// All messages that will access the shared memory go in here, as we need a lock
//	if (Mutex->try_lock())
//	{
//		boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex, boost::interprocess::try_to_lock);
//		if (lock)
//		{
	VecMsg = readFromAutoPilot(false);
	if (!VecMsg->empty())
	{
		if (config.Debug > 0)
		{
			std::cout << get_cur_1ms() << " MAPSELF " << ModuleId << " from AP: ";
			dobots::print(VecMsg->begin(), VecMsg->end());
		}
		int type = VecMsg->back();
		VecMsg->pop_back();
		switch (type)
		{
			case PROT_MAPSELF_DATAIN_GEOM:
			{
				UavGeomStruct geom;
				if (VecMsg->end() == FromCont(geom, VecMsg->begin(), VecMsg->end()))
				{
					{
						boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
						Map->UavData.Geom = geom;
					}
					if (config.Debug > 1)
						std::cout << "Set UavData.Geom to: " << geom << std::endl;
					//Map->Heading = atan2(geom.Speed.x(), geom.Speed.y());
				}
				else
					std::cout << "Error: invalid vector to create an UavGeomStruct" << std::endl;
					//printf("Invalid vector to create an UAVStruct!\n");
				break;
			}
			case PROT_MAPSELF_DATAIN_STATE:
			{
				boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
				UAVState newState = (UAVState) VecMsg->at(0);

				// If uav is in collision avoidance, stay in collision avoidance
				// but only when ap is flying normally
				if ((newState == UAVSTATE_GOING_HOME || UAVSTATE_STAYING || UAVSTATE_FLYING)
						&& Map->UavData.State == UAVSTATE_COLLISION_AVOIDING)
					Map->PreviousState = newState;
				else
					Map->UavData.State = newState;
				break;
			}
			case PROT_MAPSELF_DATAIN_WAYPOINTS:
			{
				WayPointsStruct wps;
				if (VecMsg->end() == FromCont(wps, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
					Map->WayPoints = wps;

					for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
					{
						if (i < wps.WayPointsNum)
							Map->UavData.WpNext[i] = wps[i];
						else
							Map->UavData.WpNext[i].wpMode = WP_INVALID;
					}
				}
				else
					std::cout << "Error: invalid vector to create WayPoints" << std::endl;
				break;
			}
			case PROT_MAPSELF_DATAIN_BATTERY:
			{
				boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
				Map->UavData.BatteryTimeLeft = VecMsg->at(0);
				break;
			}
//			case PROT_MAPSELF_DATAIN_HOME:
//			{
//				Position home;
//				if (VecMsg->end() == FromCont(home, VecMsg->begin(), VecMsg->end()))
//				{
//					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
//					Map->Home = home;
//				}
//				else
//					std::cout << "Invalid vector to create Home" << std::endl;
//				break;
//			}
			case PROT_MAPSELF_DATAIN_NEIGHBOURS:
			{
				MapSelfNeighboursStruct nb;
				if (VecMsg->end() == FromCont(nb, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
					Map->NeighBours = nb;
				}
				else
					std::cout << "Error: invalid vector to create Neighbours" << std::endl;
				break;
			}
			case PROT_MAPSELF_DATAIN_AP_STATUS:
			{
				APStatusStruct apStatus;
				if (VecMsg->end() == FromCont(apStatus, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
					Map->UavData.APStatus = apStatus;
				}
				else
					std::cout << "Error: invalid vector to create ap status" << std::endl;
				break;
			}
		}
		VecMsg->clear();
	}

	// Read radio msgs, the variables should be scaled and translated here
	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		if (config.Debug > 0)
		{
			std::cout << get_cur_1ms() << " MAPSELF " << ModuleId << " from RADIO: ";
			dobots::print(VecMsg->begin(), VecMsg->end());
		}
		int type = VecMsg->back();
		VecMsg->pop_back();
		switch (type)
		{
			case PROT_MAPSELF_GS_CMD:
			{
				RadioMsgRelayCmd gsCmdMsg;
				if (VecMsg->end() == FromCont(gsCmdMsg, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);

					if (config.Debug > 0)
						std::cout << "gsCmdMsg=" << gsCmdMsg << std::endl;

					// Check if the msg is a valid msg
					if (gsCmdMsg.UavId == 0)
						break;

					// Check if the msg has been seen already
					bool seen=false;
					for (int i=0; i<MAPSELF_GS_CMDS_HIST; ++i)
					{
						// TODO: incomplete check?
						if ((Map->LastGsCmds[i].Msg.UavId == gsCmdMsg.UavId)
								&& (Map->LastGsCmds[i].Msg.MsgId == gsCmdMsg.MsgId))
						{
							seen = true;
							break;
						}
					}
					if (seen)
						break;

					// Update current variables if the msg is for us
					if (gsCmdMsg.UavId == UAVS_NUM+1 || gsCmdMsg.UavId == Map->UavData.UavId + 1)
					{
						GsCmdStruct gsCmd;
						gsCmd.FromRadioMsg(gsCmdMsg);

						// If landing changed, update auto pilot
						if (gsCmd.Landing.Pos.x() != Map->GsCmd.Landing.Pos.x() ||
							gsCmd.Landing.Pos.y() != Map->GsCmd.Landing.Pos.y() ||
							gsCmd.Landing.Heading.angle() != Map->GsCmd.Landing.Heading.angle() ||
							gsCmd.Landing.LeftTurn != Map->GsCmd.Landing.LeftTurn
							)
						{
							VecMsgType vecMsg;
							vecMsg.push_back(PROT_AP_SET_LAND);
							ToCont(gsCmd.Landing, vecMsg);
							writeToAutoPilot(vecMsg);
						}

						// If AP mode changed, update auto pilot
						if (Map->GsCmd.Mode != gsCmd.Mode)
						{
							VecMsgType vecMsg;
							vecMsg.push_back(PROT_AP_SET_MODE);
							// If mode is WP, let the WP set the mode
							if (gsCmd.Mode == AP_PROT_MODE_WP)
								vecMsg.push_back(Map->RequestedAPModeByWP);
							else
								vecMsg.push_back(gsCmd.Mode);
							writeToAutoPilot(vecMsg);
						}
						Map->GsCmd = gsCmd;
						Map->GsCmd.Landing.Length = config.LandLength; // This is not included in the cmd msg!
						Map->GsCmd.Landing.Radius = config.LandRadius; // This is not included in the cmd msg!

						// Check mode to update state
						switch (gsCmd.Mode)
						{
							case AP_PROT_MODE_HOME:
								if (Map->UavData.State == UAVSTATE_COLLISION_AVOIDING)
									Map->PreviousState = UAVSTATE_GOING_HOME;
								else
									Map->UavData.State = UAVSTATE_GOING_HOME;
								break;
							case AP_PROT_MODE_LAND:
								Map->UavData.State = UAVSTATE_LANDING;
								break;
							case AP_PROT_MODE_STAY:
								Map->GsCmd.EnablePlanner = false;
								break;
							case AP_PROT_MODE_WP:
								//Map->UavData.State = UAVSTATE_FLYING;
								Map->GsCmd.EnablePlanner = true;
								break;
						}
					}

					// Add message to history
					Map->LastGsCmdsIndex = (Map->LastGsCmdsIndex+1) % MAPSELF_GS_CMDS_HIST;
					Map->LastGsCmds[Map->LastGsCmdsIndex].Msg = gsCmdMsg;
					Map->LastGsCmds[Map->LastGsCmdsIndex].TimesSent = 0;
				}
				else
					std::cout << "Error: invalid vector to create RadioMsgRelayCmd" << std::endl;

				break;
			}
		}
		VecMsg->clear();
	}

	usleep(config.TickTime);
}
