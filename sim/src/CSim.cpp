/**
 * @brief 
 * @file CSim.cpp
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

#include "CSim.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"
#include <unistd.h> // Needed for usleep()
#include <cmath>

#define SIM_REALTIME

using namespace rur;

CSim::~CSim()
{
	FileOut.close();
}

void CSim::Init(std::string module_id, int numUavsAP, int numUavsRadio, int simTime)
{
	sim::Init(module_id);
	config.load("config.json");
	if (numUavsAP < 1 && numUavsRadio < 1)
	{
		std::cout << "Number of UAVs should be a number larger than 0!" << std::endl;
		assert(false);
	}

//	TimeStep = timeStep;
//	RadioRoundTime = radioRoundTime;
//	RadioDistance = radioDistance;
	CurTime = 0;
	SimTime = simTime;
	Running = false;
	SimUavStruct uav;

	for (int i=0; i<std::max(numUavsAP,numUavsRadio); ++i)
	{
		uav.UavData.UavData.UavId = i;
		uav.AutoPilotSim = true;
		uav.RadioSim = true;
		Uavs.push_back(uav);
	}

	if (numUavsAP > numUavsRadio)
		for (int i=0; i<numUavsAP-numUavsRadio; ++i)
			Uavs[i].RadioSim = false;
	else
		for (int i=0; i<numUavsRadio-numUavsAP; ++i)
			Uavs[i].AutoPilotSim = false;

	// Add ground station as uav, makes life easier
	uav.UavData.UavData.UavId = UAVS_NUM;
	uav.AutoPilotSim = false;
	uav.RadioSim = true;
	uav.RadioWorks = true;
	uav.WaitForReplyRadio = false;
	uav.WaitForReplyAutoPilot = false;
	uav.UavData.UavData.State = UAVSTATE_FLYING; // So that this node isn't ignored?
	Uavs.push_back(uav);

	FileOut.open(config.OutputFileName.c_str());
	//{'numUavs':3, 'timeStep':0.1, 'radioRange':100}

	FileOut << "{'numUavs':" << numUavsAP << ", 'timeStep':" << config.TimeStep << ", 'radioRange':" << config.RadioRange << "}" << std::endl;
}

void CSim::Tick()
{
	IntMsg = readCommand(false);
	if (IntMsg != NULL)
	{
		switch(*IntMsg)
		{
			case CMD_INIT:
				CmdInit();
				break;
			case CMD_START:
				CmdStart();
				break;
			case CMD_STOP:
				CmdLand();
				break;
		}
	}

	// ------------------------------------------------------------------------------
	// Read radio
	// ------------------------------------------------------------------------------
	UavIterType itUav;
	for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
	{
		int id = itUav->UavData.UavData.UavId;
		VecMsg = readRadioState(id, false);
		if (!VecMsg->empty())
		{
			std::cout << "SIM from Radio " << id << ": ";
			dobots::print(VecMsg->begin(), VecMsg->end());

			VecMsgType::iterator it = VecMsg->begin();
			while (it != VecMsg->end())
			{
				int type = *it++;
				//std::cout << "Type=" << type << std::endl;
				switch (type)
				{
					case PROT_SIMSTAT_ACK:
					{
						itUav->WaitForReplyRadio = false;
						break;
					}
					case PROT_SIMSTAT_BROADCAST_MSG:
					{
						// This only works when we assume that the broadcast msg is at the end of vecmsg
						//UavIterType itUav2;
						if (it != VecMsg->end() && itUav->RadioWorks)
						{
							// Only add the msg to connected neighbours
							//for (itUav2 = Uavs.begin(); itUav2 != Uavs.end(); ++itUav2)
							for (int i = 0; i < itUav->UavData.NeighBours.ConnectedNeighboursNum; ++i)
							{
								//int id2 = itUav2->UavData.UavData.UavId;
								int id2 = itUav->UavData.NeighBours.ConnectedNeighbours[i];
								BroadcastMsgs[id2].push_back(PROT_SIMCMD_RADIO_ROUND_BROADCAST_MSG);
								BroadcastMsgs[id2].insert(BroadcastMsgs[id2].end(), it, VecMsg->end());
							}

							// Debug test
							RadioMsg bmsg;
							it = FromCont(bmsg, it, VecMsg->end());
							if (it != VecMsg->end())
								std::cout << "ERROR! " << bmsg << std::endl;
							std::cout << "Adding bmsg: " << bmsg << std::endl;
						}
						break;
					}
				}
			}
			VecMsg->clear();
		}
	}

	// ------------------------------------------------------------------------------
	// Read new auto pilot state (uav position etc)
	// ------------------------------------------------------------------------------
	for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
	{
		if (!itUav->AutoPilotSim)
			continue;
		int id = itUav->UavData.UavData.UavId;
		VecMsg = readAutoPilotState(id, false);
		if (!VecMsg->empty())
		{
			std::cout << "SIM from AP " << id << ": ";
			dobots::print(VecMsg->begin(), VecMsg->end());

			//			if (Uavs[i].UavData.UavData.State == UAVSTATE_LANDED)
			//				return; // Nothing to read here :)

			VecMsgType::iterator it = VecMsg->begin();
			while (it != VecMsg->end())
			{
				int type = *it++;
				//std::cout << "Type=" << type << std::endl;
				switch (type)
				{
				case PROT_SIMSTAT_ACK:
				{
					itUav->WaitForReplyAutoPilot = false;
					break;
				}
				case PROT_SIMSTAT_GEOM:
				{
					//UavGeomStruct geom;
					//FromCont(geom, it, VecMsg->end());
					it = FromCont(itUav->UavData.UavData.Geom, it, VecMsg->end());
					break;
				}
				//					case PROT_SIMSTAT_GEOM_EXTRA:
				//					{
				//						itUav->UavData.Heading = *it++;
				//						itUav->UavData.RollAngle = *it++;
				//						break;
				//					}
				case PROT_SIMSTAT_BATTERY:
				{
					itUav->UavData.UavData.BatteryTimeLeft = *it++;
					break;
				}
				case PROT_SIMSTAT_WP:
				{
					it = FromCont(itUav->UavData.WayPoints[0], it, VecMsg->end());
					break;
				}
				}
			}
			VecMsg->clear();
		}
	}


	if (Running)
	{

		// --------------------------------------------------------------------------
		// Check if can take the next step in the radio round
		// --------------------------------------------------------------------------
		for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
		{
			if (itUav->WaitForReplyRadio)
				return;
		}

		// One radio round
		// state = idle, waiting for time
		// ==> start --- state = idle -> start

		// <== bmsg, ack --- state = start -> send/receive
		// ==> bmsgs --- state = send/receive

		// <== ack --- state = send/receive -> end
		// ==> end --- state = end

		// <== ack --- state = end -> idle, wait for time
		if (RadioRoundState != RADIO_STATE_ROUND_IDLE)
			std::cout << "RadioRoundState=" << RadioRoundState << std::endl;
		switch (RadioRoundState)
		{
			case RADIO_STATE_ROUND_START:
			{
				// Radios received the start command and replied with their msg
				// Send msgs to all uavs with working radio
				for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
				{
					int id = itUav->UavData.UavData.UavId;
					if (itUav->RadioWorks && !BroadcastMsgs[id].empty())
					{
						itUav->WaitForReplyRadio = true;
						writeRadioCommand(id, BroadcastMsgs[id]);
					}
					BroadcastMsgs[id].clear();
				}
				RadioRoundState = RADIO_STATE_ROUND_SENDRECEIVE;
				return; // return, not break, since we have to wait for replies
			}
			case RADIO_STATE_ROUND_SENDRECEIVE:
			{
				// Tell all uavs that the radio round is over
				VecMsgType vecMsg;
				vecMsg.push_back(PROT_SIMCMD_RADIO_ROUND_END);
				for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
				{
					itUav->WaitForReplyRadio = true;
					writeRadioCommand(itUav->UavData.UavData.UavId, vecMsg);
				}

				RadioRoundState = RADIO_STATE_ROUND_END;
				return; // return, not break, since we have to wait for replies
			}
			case RADIO_STATE_ROUND_END:
			{
				// All uavs received the end of the round, radio goes idle
				RadioRoundState = RADIO_STATE_ROUND_IDLE;
				break;
			}
		}

		VecMsgType vecMsg;

		// --------------------------------------------------------------------------
		// After some time waiting it's time for a new radio round
		// --------------------------------------------------------------------------
		if (LastRadioRoundStartTime+config.RadioRoundTime < CurTime)
		{
			LastRadioRoundStartTime = CurTime;
			RadioRoundState = RADIO_STATE_ROUND_START;
			vecMsg.push_back(PROT_SIMCMD_RADIO_ROUND_START);
			for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
			{
				itUav->WaitForReplyRadio = true;
				writeRadioCommand(itUav->UavData.UavData.UavId, vecMsg);
			}
			return; // return, since we have to wait for replies
		}

		// --------------------------------------------------------------------------
		// Check if we can take the next time step: shouldn't be waiting for any ACK (radio or autopilot)
		// --------------------------------------------------------------------------
		for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
		{
			if (itUav->WaitForReplyAutoPilot || itUav->WaitForReplyRadio)
				return;
		}

#ifdef SIM_REALTIME
		// Simulation should go real time
		if (get_cur_1ms() - LastTimeStepTimeStamp < 1000*config.TimeStep)
			return;
		LastTimeStepTimeStamp = get_cur_1ms();
#endif

		// If we arrived here, all uavs have taken their time step and reported back the new state

		// Update the neighbours of each UAV
		for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
		{
			itUav->UavData.NeighBours.clear();
//			if (!itUav->AutoPilotSim)
//			{
//				// UAV has unknown position, let's just say it's connected to everything
//				UavIterType itUav2;
//				for (itUav2=Uavs.begin(); itUav2 != Uavs.end(); ++itUav2)
//					if (itUav != itUav2)
//						itUav->UavData.NeighBours.push_back(itUav2->UavData.UavData.UavId);
//				continue;
//			}
			Position p1(itUav->UavData.UavData.Geom.Pos);
			if (itUav->UavData.UavData.UavId == UAVS_NUM)
				p1 << config.GroundStationX, config.GroundStationY, 0;
			Position p2, diff;
			UavIterType itUav2;
			for (itUav2=Uavs.begin(); itUav2 != Uavs.end(); ++itUav2)
			{
				// Uav can't listen to it's own msgs
				if (itUav == itUav2)
					continue;

				// Position unknown of one of the UAVs: assume connected
				if ((!itUav->AutoPilotSim && itUav->UavData.UavData.UavId != UAVS_NUM)
					|| (!itUav2->AutoPilotSim && itUav2->UavData.UavData.UavId != UAVS_NUM))
				{
					itUav->UavData.NeighBours.push_back(itUav2->UavData.UavData.UavId);
				}
				// Check distance
				else
				{
					if (itUav2->UavData.UavData.UavId == UAVS_NUM)
						p2 << config.GroundStationX, config.GroundStationY, 0;
					else
						p2 = itUav2->UavData.UavData.Geom.Pos;
					diff = p1 - p2;
					if (diff.dot(diff) <= config.RadioRange*config.RadioRange)
					{
						itUav->UavData.NeighBours.push_back(itUav2->UavData.UavData.UavId);
					}
				}
			}
			//std::cout << "uav " << it->UavData.UavData.UavId << " neighbours: ";
			//dobots::print(it->UavData.NeighBours.ConnectedNeighbours, it->UavData.NeighBours.ConnectedNeighbours + it->UavData.NeighBours.ConnectedNeighboursNum);
		}


		// Write current states of uavs to output file
		FileOut << get_cur_1ms() << " ";
		for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
		{
			if (itUav->UavData.UavData.UavId == UAVS_NUM)
			{
				itUav->UavData.UavData.Geom.Pos << config.GroundStationX, config.GroundStationY, 0;
				itUav->UavData.WayPoints[0].to << config.GroundStationX, config.GroundStationY, 0;
			}
			else if (!itUav->AutoPilotSim)
				continue;

			FileOut << itUav->UavData.UavData.UavId << " ";
			FileOut << itUav->UavData.UavData.Geom.Pos.x() << " ";
			FileOut << itUav->UavData.UavData.Geom.Pos.y() << " ";
			FileOut << itUav->UavData.UavData.Geom.Pos.z() << " ";
			FileOut << itUav->UavData.UavData.Geom.Heading.angle() << " ";
			FileOut << itUav->UavData.WayPoints[0].to.x() << " ";
			FileOut << itUav->UavData.WayPoints[0].to.y() << " ";
			FileOut << itUav->UavData.WayPoints[0].to.z() << " ";
			//file << autoPilotSim->RollAngle.angle();

			int neighbours = 0;
			for (int i=0; i<itUav->UavData.NeighBours.ConnectedNeighboursNum; ++i)
				neighbours |= 1 << itUav->UavData.NeighBours.ConnectedNeighbours[i];
			FileOut << neighbours << " ";
		}
		FileOut << std::endl;

		// Check for end of simulation time
		if (CurTime > SimTime)
		{
			std::cout << std::endl << "----- End of simulation -----" << std::endl;
			std::cout << SimTime << "s simulated in " << (get_cur_1ms() - StartTimeStamp)/1000 << "s" << std::endl;
			exit(1);
		}

#ifndef SIM_REALTIME
		// Give UAVs some time to calculate
		// Wp planner takes about 25 ms
		// Should have planning at at least 4 Hz, so 1 plan per 25 timesteps
		// So each timestep should at least take 1ms
		usleep(1*1000);
		//usleep(10*1000);	// 10ms = 0.01s = 0.1*timestep
		//usleep(50*1000);	// 50ms = 0.05s = 0.5*timestep
#endif

		// Now that we've written all the latest states, we can go to the next time step
		std::cout << std::endl << "----- Next time step " << CurTime << " -----" << std::endl;

		// Command autopilot to take a time step
		for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
		{
			vecMsg.clear();
			int id = itUav->UavData.UavData.UavId;
			if (itUav->UavData.UavData.State == UAVSTATE_LANDED)
			{
				std::cout << id << " takeofftime=" << itUav->TakeOffTime << std::endl;
				if (CurTime > itUav->TakeOffTime)
				{
					itUav->UavData.UavData.State = UAVSTATE_FLYING;
					itUav->RadioWorks = true;
					vecMsg.push_back(PROT_SIMCMD_SET_STATE);
					vecMsg.push_back(UAVSTATE_FLYING);
				}
			}
			if (!itUav->AutoPilotSim)
				continue;
			vecMsg.push_back(PROT_SIMCMD_TIMESTEP);
			vecMsg.push_back(config.TimeStep);
			itUav->WaitForReplyAutoPilot = true;
			writeAutoPilotCommand(id, vecMsg);
		}

//		for (int i=0; i<Uavs.size(); ++i)
//		{
//			writeAutoPilotCommand(i, vecMsg);
//		}

		// Increase time
		CurTime += config.TimeStep;
	}
	usleep(config.TickTime);
}

void CSim::CmdInit()
{
/*
	VecMsgType vecMsg;
	//vecMsg.clear();
	vecMsg.push_back(PROT_SIMCMD_INIT);

	for (int i=0; i<Uavs.size(); ++i)
	{
		writeRadioCommand(i, vecMsg);
		writeAutoPilotCommand(i, vecMsg);
	}

	Running = false;
*/
}

void CSim::CmdStart()
{
	VecMsgType vecMsg;

	UavIterType it;
	int id=0;
	for (it=Uavs.begin(); it != Uavs.end(); ++it, ++id)
	{
		//it->UavData.UavData.UavId = id; // Should be set at init
		vecMsg.clear();

		it->UavData.UavData.Geom.Init();
		it->UavData.UavData.Geom.Pos.x() = config.GroundStationX;
		it->UavData.UavData.Geom.Pos.y() = config.GroundStationY;

		vecMsg.push_back(PROT_SIMCMD_SET_GEOM);
		//geom.Pos << (id+0)*50, (id+0)*50, (id+0)*10;
		ToCont(it->UavData.UavData.Geom, vecMsg);

		//vecMsg.push_back(PROT_SIMCMD_SET_BATTERY);
		//vecMsg.push_back(40*60 + rand()%5); // 40 - 44 minutes battery time
		//vecMsg.push_back((i+35)*60);

		it->UavData.UavData.State = UAVSTATE_LANDED;
		vecMsg.push_back(PROT_SIMCMD_SET_STATE);
		vecMsg.push_back(it->UavData.UavData.State);

		if (it->AutoPilotSim)
		{
			writeAutoPilotCommand(id, vecMsg);
			it->WaitForReplyAutoPilot = true;
		}

		// Each 2 minutes 2 uavs liftoff (with 20s in between)
		//it->TakeOffTime = int(id/2)*120.0 + id*20 + rand()%10;
		it->TakeOffTime = id*5.0;
		//it->TakeOffTime = 0;
		it->RadioWorks = false;
		it->WaitForReplyRadio = false;

		// Add a dummy wp
		WayPoint wp;
		wp.to << 0, 0, 0;
		it->UavData.WayPoints.push_back(wp);

		BroadcastMsgs[id].clear();
	}

	RadioRoundState = RADIO_STATE_ROUND_IDLE;
	LastRadioRoundStartTime = 0;
	Running = true;
	StartTimeStamp = get_cur_1ms();
	LastTimeStepTimeStamp = get_cur_1ms();
}

void CSim::CmdStop()
{
	Running = false;
}

void CSim::CmdLand()
{
	UavIterType itUav;
	for (itUav=Uavs.begin(); itUav != Uavs.end(); ++itUav)
	{
		int id = itUav->UavData.UavData.UavId;
		VecMsgType vecMsg;
		vecMsg.push_back(PROT_SIMCMD_LAND);
		writeRadioCommand(id, vecMsg);
	}
}

void CSim::writeRadioCommand(int uavId, const VecMsgType& vecMsg)
{
	switch (uavId)
	{
		case 0: writeRadioCommand0(vecMsg); break;
		case 1: writeRadioCommand1(vecMsg); break;
		case 2: writeRadioCommand2(vecMsg); break;
		case 3: writeRadioCommand3(vecMsg); break;
		case 4: writeRadioCommand4(vecMsg); break;
		case 5: writeRadioCommand5(vecMsg); break;
		case 6: writeRadioCommand6(vecMsg); break;
		case 7: writeRadioCommand7(vecMsg); break;
		case 8: writeRadioCommand8(vecMsg); break;
		case 9: writeRadioCommand9(vecMsg); break;
		case 10: writeGroundStation(vecMsg); break;
	}
}

VecMsgType* CSim::readRadioState(int uavId, bool blocking)
{
	switch (uavId)
	{
		case 0: return readRadioState0(blocking);
		case 1: return readRadioState1(blocking);
		case 2: return readRadioState2(blocking);
		case 3: return readRadioState3(blocking);
		case 4: return readRadioState4(blocking);
		case 5: return readRadioState5(blocking);
		case 6: return readRadioState6(blocking);
		case 7: return readRadioState7(blocking);
		case 8: return readRadioState8(blocking);
		case 9: return readRadioState9(blocking);
		case 10: return readFromGroundStation(blocking);
	}
}

void CSim::writeAutoPilotCommand(int uavId, const VecMsgType& vecMsg)
{
	switch (uavId)
	{
		case 0: writeAutoPilotCommand0(vecMsg); break;
		case 1: writeAutoPilotCommand1(vecMsg); break;
		case 2: writeAutoPilotCommand2(vecMsg); break;
		case 3: writeAutoPilotCommand3(vecMsg); break;
		case 4: writeAutoPilotCommand4(vecMsg); break;
		case 5: writeAutoPilotCommand5(vecMsg); break;
		case 6: writeAutoPilotCommand6(vecMsg); break;
		case 7: writeAutoPilotCommand7(vecMsg); break;
		case 8: writeAutoPilotCommand8(vecMsg); break;
		case 9: writeAutoPilotCommand9(vecMsg); break;
	}
}

VecMsgType* CSim::readAutoPilotState(int uavId, bool blocking)
{
	switch (uavId)
	{
		case 0: return readAutoPilotState0(blocking);
		case 1: return readAutoPilotState1(blocking);
		case 2: return readAutoPilotState2(blocking);
		case 3: return readAutoPilotState3(blocking);
		case 4: return readAutoPilotState4(blocking);
		case 5: return readAutoPilotState5(blocking);
		case 6: return readAutoPilotState6(blocking);
		case 7: return readAutoPilotState7(blocking);
		case 8: return readAutoPilotState8(blocking);
		case 9: return readAutoPilotState9(blocking);
	}
}
