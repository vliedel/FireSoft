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
		Map->UavData.WpNext.Init();
		Map->UavData.WpFar.Init();
		Map->UavData.UavId = id;
		Map->PreviousState = UAVSTATE_LANDED;
		Map->BatteryTimeLeft = config.BatteryTime;
		// Init ground station command messages with invalid ids
		for (int i=0; i<MAPSELF_GS_CMDS_HIST; ++i)
			Map->LastGsCmds[i].MsgId = 255; // TODO: magic number

		Map->AreaZero.x() = config.AreaOriginX;
		Map->AreaZero.y() = config.AreaOriginY;
		Map->AreaSize.x() = config.AreaSizeX;
		Map->AreaSize.y() = config.AreaSizeY;
		Map->AreaRotation.angle() = config.AreaRotation;
		Map->Landing.Pos.x() = config.LandPointX;
		Map->Landing.Pos.y() = config.LandPointY;
		Map->Landing.Heading.angle() = config.LandHeading;
		Map->Landing.LeftTurn = config.LandLeftTurn;
		Map->HeightMin = config.MinHeight;
		Map->HeightMax = config.MaxHeight;
		Map->RequestedAPModeByGS = AP_PROT_MODE_WP;
		Map->EnablePlanner = true;
	}
}

void CMapSelf::Tick()
{
	IntMsg = readCommand(false);
	if (IntMsg != NULL)
	{

	}

	// All messages that will access the shared memory go in here, as we need a lock
//	if (Mutex->try_lock())
//	{
//		boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex, boost::interprocess::try_to_lock);
//		if (lock)
//		{
	VecMsg = readAutoPilot(false);
	if (!VecMsg->empty())
	{
		std::cout << get_cur_1ms() << " MAPSELF " << ModuleId << " from AP: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		int type = VecMsg->back();
		VecMsg->pop_back();
		switch (type)
		{
			case PROT_MAPSELF_DATAIN_GEOM:
			{
				UavGeomStruct geom;
				if (VecMsg->end() == FromCont(geom, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
					Map->UavData.Geom = geom;
					//Map->Heading = atan2(geom.Speed.x(), geom.Speed.y());
				}
				else
					std::cout << "Invalid vector to create a UAVStruct" << std::endl;
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

					std::cout << get_cur_1ms();
					if (wps.WayPointsNum > 0)
					{
						// Set wp Next
						if (wps[0].wpMode == WP_CIRCLE)
						{
							Map->UavData.WpNext = wps[0];
						}
						else if (wps.WayPointsNum > 1)
						{
							//Map->UavData.WpNext = wps[1];
							// Let's pretend we fly in a straight line to the end point
							Map->UavData.WpNext.wpMode = WP_LINE;
							wps[1].GetEndPos(Map->UavData.WpNext.to);
							Map->UavData.WpNext.Radius = 0;
						}
						else
							Map->UavData.WpNext = wps[0];

						// Set wp Far
						if (wps[0].wpMode == WP_CIRCLE)
						{
							Map->UavData.WpFar = wps[0];
//							Map->UavData.WpFar.wpMode = WP_CIRCLE;
//							Map->UavData.WpFar.to = wps[0].to;
//							Map->UavData.WpFar.Radius = wps[0].Radius;
						}
						else
						{
							//Map->UavData.WpFar = wps[wps.WayPointsNum-1];
							// Instead of copying the wp, we are only interested in the last point of the wp path
							// TODO: make this less ugly
							Map->UavData.WpFar.wpMode = WP_LINE;
							wps[wps.WayPointsNum-1].GetEndPos(Map->UavData.WpFar.to);
						}

					}
					else
					{
						std::cout << "No waypoints!" << std::endl;
						// TODO: What if there are NO waypoints????
						Map->UavData.WpNext.wpMode = WP_LINE;
						Map->UavData.WpNext.Radius = 0;
						Map->UavData.WpNext.to = Map->UavData.Geom.Pos;

						Map->UavData.WpFar.wpMode = WP_LINE;
						Map->UavData.WpFar.to = Map->UavData.Geom.Pos;
					}
					std::cout << " wpNext=" << Map->UavData.WpNext;
					std::cout << " wpFar=" << Map->UavData.WpFar.to.transpose() << std::endl;

				}
				else
					std::cout << "Invalid vector to create WayPoints" << std::endl;
				break;
			}
			case PROT_MAPSELF_DATAIN_BATTERY:
			{
				boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
				Map->BatteryTimeLeft = VecMsg->at(0);
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
					std::cout << "Invalid vector to create Neighbours" << std::endl;
				break;
			}
			case PROT_MAPSELF_DATAIN_AP_STATUS:
			{
				MapSelfAPStatusStruct apStatus;
				if (VecMsg->end() == FromCont(apStatus, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);
					Map->APStatus = apStatus;
				}
				else
					std::cout << "Invalid vector to create ap status" << std::endl;
				break;
			}
		}
		VecMsg->clear();
	}

	// Read radio msgs, the variables should be scaled and translated here
	VecMsg = readFromRadio(false);
	if (!VecMsg->empty())
	{
		std::cout << get_cur_1ms() << " MAPSELF " << ModuleId << " from RADIO: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		int type = VecMsg->back();
		VecMsg->pop_back();
		switch (type)
		{
			case PROT_MAPSELF_GS_CMD:
			{
				RadioMsgRelayCmd gsCmd;
				if (VecMsg->end() == FromCont(gsCmd, VecMsg->begin(), VecMsg->end()))
				{
					boost::interprocess::scoped_lock<MapMutexType> lock(*Mutex);

					// Shift history and add new msg at back
					for (int i=0; i<MAPSELF_GS_CMDS_HIST-1; ++i)
						Map->LastGsCmds[i] = Map->LastGsCmds[i+1];
					Map->LastGsCmds[MAPSELF_GS_CMDS_HIST-1] = gsCmd;

					// Update current variables
					// Translate to meters and radians
					// TODO: MAGIC NUMBERS SO MANY

					if (gsCmd.UavId == 15 || gsCmd.UavId == Map->UavData.UavId)
					{
						// Height is 0 to 255, translate to 50 to 305
						Map->HeightMin = gsCmd.HeightMin + 50;
						Map->HeightMax = gsCmd.HeightMax + 50;
						// Area has precision of 5 meter
						Map->AreaZero.x() = gsCmd.AreaMinX * 5;
						Map->AreaZero.y() = gsCmd.AreaMinY * 5;
						Map->AreaSize.x() = gsCmd.AreaDX * 5;
						Map->AreaSize.y() = gsCmd.AreaDY * 5;
						// Rotation is 0 to 255, translate to 0 to 0.5*pi
						Map->AreaRotation.angle() = 0.5*M_PI* gsCmd.AreaRotation / 255;

						// In meters
						LandingStruct land;
						land.Pos.x() = gsCmd.LandX;
						land.Pos.y() = gsCmd.LandY;
						// Heading is 0 to 255, translate to 0 to 2*pi
						land.Heading.angle() = 2.0*M_PI* gsCmd.LandHeading /255;
						land.LeftTurn = gsCmd.LandLeftTurn;

						// If landing changed, update auto pilot
						if (land.Pos.x() != Map->Landing.Pos.x() ||
							land.Pos.y() != Map->Landing.Pos.y() ||
							land.Heading.angle() != Map->Landing.Heading.angle() ||
							land.LeftTurn != Map->Landing.LeftTurn
							)
						{
							VecMsgType vecMsg;
							vecMsg.push_back(PROT_AP_SET_LAND);
							ToCont(land, vecMsg);
							writeAutoPilot(vecMsg);
						}
						Map->Landing = land;

						// If AP mode changed, update auto pilot
						if (Map->RequestedAPModeByGS != gsCmd.Mode)
						{
							VecMsgType vecMsg;
							vecMsg.push_back(PROT_AP_SET_MODE);
							// If mode is WP, let the WP set the mode
							if (gsCmd.Mode == AP_PROT_MODE_WP)
								vecMsg.push_back(Map->RequestedAPModeByWP);
							else
								vecMsg.push_back(gsCmd.Mode);
							writeAutoPilot(vecMsg);
						}
						Map->RequestedAPModeByGS = gsCmd.Mode;

						Map->EnablePlanner = gsCmd.EnablePlanner;

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
								Map->EnablePlanner = false;
								break;
							case AP_PROT_MODE_WP:
								//Map->UavData.State = UAVSTATE_FLYING;
								Map->EnablePlanner = true;
								break;
						}
					}
				}
				else
					std::cout << "Invalid vector to create RadioMsgRelayCmd" << std::endl;

				break;
			}
		}
		VecMsg->clear();
	}

	usleep(config.TickTime);
}
