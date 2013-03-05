/**
 * @brief 
 * @file CAutoPilot.cpp
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

#include "CAutoPilot.h"
#include "Protocol.h"
#include "CTime.h"
#include "Print.hpp"

using namespace rur;

CAutoPilot::~CAutoPilot()
{
	delete Serial;
	PathOutFile.close();
	PlanOutFile.close();
}

void CAutoPilot::Init(std::string module_id)
{
	autoPilot::Init(module_id);
	UavId = atoi(module_id.c_str());
	config.load("config.json");

	try
	{
		//Serial = new BufferedAsyncSerial(config.PortName, AP_PROT_SERIAL_SPEED);
		Serial = new BufferedAsyncSerial(config.PortName, config.SerialSpeed);
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}
	Synchronize = true;
	LastReadHeaderUsed = true;
	LastReqTimeSensorData = get_cur_1ms();
	LastReqTimeWpStatus = get_cur_1ms();
	LastReqTimeWpBounds = get_cur_1ms();

	LastSetTimeMode = get_cur_1ms();
	LastSetMode.Mode = AP_PROT_MODE_WP;

	LastSetTimeLanding = get_cur_1ms();
	// Convert to mercator coordinates
	LastSetLanding.LandPoint.X = config.LandPointX + config.OriginX; // Local to mercator coordinates
	LastSetLanding.LandPoint.Y = config.LandPointY + config.OriginY; // Local to mercator coordinates
	LastSetLanding.LandPoint.Z = 0;
	LastSetLanding.LandHeading = config.LandHeading;
	LastSetLanding.LandLeftTurn = config.LandLeftTurn;

	LastWpId = 0;

	std::string fileName = "output/path_" + module_id + ".txt";
	PathOutFile.open(fileName.c_str());
	fileName = "output/plan_" + module_id + ".txt";
	PlanOutFile.open(fileName.c_str());
}

void CAutoPilot::Tick()
{
//	IntMsg = readCommand(false);
//	if (IntMsg != NULL)
//	{
//
//	}

	// Check if there is something to read
	ReadUart();

//	// From time to time: request AP for sensor data
//	if (get_duration(LastReqTimeSensorData, get_cur_1ms()) > config.SensorReqIntervalTime)
//	{
//		//std::cout << "Sending sensor request" << std::endl;
//		SendHeader(AP_PROT_REQ_SENSORDATA, 0);
//		LastReqTimeSensorData = get_cur_1ms();
//	}

//	// From time to time: request AP for wp data
//	if (get_duration(LastReqTimeWpStatus, get_cur_1ms()) > config.WPStatustReqIntervalTime)
//	{
//		//std::cout << "Sending wp status request" << std::endl;
//		SendHeader(AP_PROT_REQ_WP_STATUS, 0);
//		LastReqTimeWpStatus = get_cur_1ms();
//	}


	// TODO: set it every time? set on change maybe better, but msg might be missed
	// From time to time: set correct mode
	if (get_duration(LastSetTimeMode, get_cur_1ms()) > 500) // TODO: magic number
	{
		SendHeader(AP_PROT_SET_MODE, sizeof(AutoPilotMsgMode));
		//SendData((char*)&LastSetMode, sizeof(AutoPilotMsgMode));
		AutoPilotMsgMode mode;
		mode.Mode = rand() % 4;
		SendData((char*)&mode, sizeof(AutoPilotMsgMode));
		LastSetTimeMode = get_cur_1ms();
	}

	if (get_duration(LastSetTimeLanding, get_cur_1ms()) > 250) // TODO: magic number
	{
		SendHeader(AP_PROT_SET_LAND, sizeof(AutoPilotMsgLanding));
		SendData((char*)&LastSetLanding, sizeof(AutoPilotMsgLanding));
		LastSetTimeLanding = get_cur_1ms();
	}


	// From WayPoint Planner
	VecMsg = readFromWayPointPlanner(false);
	if (!VecMsg->empty())
	{
		std::cout << "AP " << UavId << " from WpPlanner: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgReply.clear();
		std::vector<float>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_AP_SET_MODE:
				{
					LastSetMode.Mode = *it++;
					SendHeader(AP_PROT_SET_MODE, sizeof(AutoPilotMsgMode));
					SendData((char*)&LastSetMode, sizeof(AutoPilotMsgMode));
					break;
				}
				case PROT_AP_SET_WAYPOINTS:
				{
					WayPointsStruct wayPoints;
					it = FromCont(wayPoints, it, VecMsg->end());
					SetWayPoints(wayPoints);
					if (it != VecMsg->end())
						std::cout << "Error in receiving wps" << std::endl;
					break;
				}
			}
		}
		VecMsg->clear();
	}

	// Read from map self
	VecMsg = readFromMapSelf(false);
	if (!VecMsg->empty())
	{
		std::cout << "AP " << UavId << " from MapSelf: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgReply.clear();
		std::vector<float>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_AP_SET_MODE:
				{
					LastSetMode.Mode = *it++;
					SendHeader(AP_PROT_SET_MODE, sizeof(AutoPilotMsgMode));
					SendData((char*)&LastSetMode, sizeof(AutoPilotMsgMode));
					break;
				}
				case PROT_AP_SET_LAND:
				{
					LandingStruct land;
					it = FromCont(land, it, VecMsg->end());
					LastSetLanding.LandPoint.X = land.Pos.x() + config.OriginX; // Local to mercator coordinates
					LastSetLanding.LandPoint.Y = land.Pos.y() + config.OriginY; // Local to mercator coordinates
					LastSetLanding.LandPoint.Z = land.Pos.z();
					LastSetLanding.LandHeading = -(land.Heading.angle() - 0.5*M_PI);
					if (LastSetLanding.LandHeading > M_PI)
						LastSetLanding.LandHeading -= 2*M_PI;
					if (LastSetLanding.LandHeading < -M_PI)
						LastSetLanding.LandHeading += 2*M_PI;
					LastSetLanding.LandLeftTurn = land.LeftTurn;
					SendHeader(AP_PROT_SET_LAND, sizeof(AutoPilotMsgLanding));
					SendData((char*)&LastSetLanding, sizeof(AutoPilotMsgLanding));
					break;
				}
			}
		}
		VecMsg->clear();
	}

	usleep(config.TickTime);
}


bool CAutoPilot::SynchronizeUart(AutoPilotMsgHeader& msgHdr)
{
	if (Serial->available() < 2*sizeof(AutoPilotMsgHeader)) // TODO: magic number
		return false;

	//std::cout << "Synchronizing..";
	char chr;
	AutoPilotMsgHeaderType header;
	uint8_t msgType;
	Serial->read((char*)&header, sizeof(AutoPilotMsgHeaderType));
	Serial->read((char*)&msgType, sizeof(msgType));
	while (Serial->available() >= sizeof(AutoPilotMsgHeader))
	{
		std::cout << " header=" << header << " msgType=" << +msgType;
		if (header == AP_PROT_HEADER && msgType < AP_PROT_NUM)
		{
			msgHdr.Header = header;
			//Serial->read((char*)&msgHdr+sizeof(AutoPilotMsgHeaderType), sizeof(AutoPilotMsgHeader)-sizeof(AutoPilotMsgHeaderType));
			msgHdr.MsgType = msgType;
			Serial->read((char*)&msgHdr+sizeof(AutoPilotMsgHeaderType)+sizeof(msgType), sizeof(AutoPilotMsgHeader)-sizeof(AutoPilotMsgHeaderType)-sizeof(msgType));
			Synchronize = false;
			LastReadHeaderUsed = false;
			//std::cout << std::endl;
			return true;
		}
		//Serial->read(&chr, 1);
		//header = (header << 8) | chr;
		header = (header << 8) | msgType;
		Serial->read((char*)&msgType, 1);
		std::cout << " header=" << header << " msgType=" << +msgType;
	}
	std::cout << std::endl;
	return false;
}


void CAutoPilot::ReadUart()
{
	//std::cout << get_cur_1ms() << " Read UART " << Serial->available() << std::endl;

	// Check if we need to synchronize (also reads a new header)
	if (Synchronize)
	{
		if (!SynchronizeUart(LastReadHeader))
			return;
		std::cout << "Synched, header=" << LastReadHeader << std::endl;
	}

	// Check if we need to read a new header
	if (LastReadHeaderUsed)
	{
		if (Serial->available() >= sizeof(AutoPilotMsgHeader))
		{
			Serial->read((char*)&LastReadHeader, sizeof(AutoPilotMsgHeader));
			if (LastReadHeader.Header != AP_PROT_HEADER)
			{
				std::cout << "Error header doesn't match, header=" << LastReadHeader << std::endl;
				Synchronize = true;
				return;
			}
		}
		else
			return;
		LastReadHeaderUsed = false;
		std::cout << "Read new header: " << LastReadHeader << std::endl;
	}

	// Header is read successfully, now read the data
	if (Serial->available() < LastReadHeader.DataSize + sizeof(CheckSumOut))
		return;
	LastReadHeaderUsed = true;

	switch (LastReadHeader.MsgType)
	{
	case AP_PROT_SENSORDATA:
	{
		AutoPilotMsgSensorData data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgSensorData)))
			return;
		std::cout << get_cur_1ms() << " Received sensor data:" << data << std::endl;


		// Write to state to mapSelf
		UavGeomStruct geom;
		geom.Pos.x() = data.Position.X - config.OriginX; // Mercator to local coordinates
		geom.Pos.y() = data.Position.Y - config.OriginY; // Mercator to local coordinates
		geom.Pos.z() = data.Position.Z;

		geom.GroundSpeed = data.GroundSpeed;
		geom.VerticalSpeed = data.VerticalSpeed;
		geom.Heading.angle() = -1.0*data.Heading + 0.5*M_PI;
		if (geom.Heading.angle() > M_PI)
			geom.Heading.angle() -= 2*M_PI;
		if (geom.Heading.angle() < -M_PI)
			geom.Heading.angle() += 2*M_PI;

		PathOutFile << geom.Pos.x() << " " << geom.Pos.y() << " " << geom.Heading.angle() << std::endl;

		geom.Yaw.angle() = data.Yaw;
		geom.Pitch.angle() = data.Pitch;
		geom.Roll.angle() = data.Roll;
		std::vector<float> vecMsg;
		vecMsg.clear();
		ToCont(geom, vecMsg);
		vecMsg.push_back(PROT_MAPSELF_DATAIN_GEOM);
		writeToMapSelf(vecMsg);

		APStatusStruct apStatus;
		apStatus.FlyState = data.FlyState;
		apStatus.GPSState = data.GPSState;
		apStatus.ServoState = data.ServoState;
		apStatus.AutoPilotState = data.AutoPilotState;
		apStatus.SensorState = data.SensorState;
		//std::vector<float> vecMsg;
		vecMsg.clear();
		ToCont(apStatus, vecMsg);
		vecMsg.push_back(PROT_MAPSELF_DATAIN_AP_STATUS);
		writeToMapSelf(vecMsg);

		// Set the correct state
		vecMsg.clear();
		switch(apStatus.FlyState)
		{
			case AP_PROT_FLY_STATE_IDLE:
				vecMsg.push_back(UAVSTATE_LANDED);
				break;
			case AP_PROT_FLY_STATE_TAKEOFF:
				vecMsg.push_back(UAVSTATE_TAKING_OFF);
				break;
			case AP_PROT_FLY_STATE_FLYING:
				vecMsg.push_back(UAVSTATE_FLYING);
				break;
			case AP_PROT_FLY_STATE_STAYING:
				vecMsg.push_back(UAVSTATE_STAYING);
				break;
			case AP_PROT_FLY_STATE_GOING_HOME:
				vecMsg.push_back(UAVSTATE_GOING_HOME);
				break;
			case AP_PROT_FLY_STATE_LANDING:
				vecMsg.push_back(UAVSTATE_LANDING);
				break;
		}
		if (!vecMsg.empty())
		{
			vecMsg.push_back(PROT_MAPSELF_DATAIN_STATE);
			writeToMapSelf(vecMsg);
		}

		vecMsg.clear();
		vecMsg.push_back((float)data.BatteryLeft * 10); // TODO: BatteryLeft is in ms?
		vecMsg.push_back(PROT_MAPSELF_DATAIN_BATTERY);
		writeToMapSelf(vecMsg);

		break;
	}
	case AP_PROT_WP_STATUS:
	{
		AutoPilotMsgWpStatus data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgWpStatus)))
			return;
		std::cout << get_cur_1ms() << " Received wp status:" << data << std::endl;


		// List current waypoints from back to front, remove ones that are not in the status msg
		for (int j=CurWayPoints.WayPointsNum-1; j>=0; --j)
		{
			std::cout << "checking " << j << " num=" << CurWayPoints.WayPointsNum << std::endl;
			bool found = false;
			for (int i=0; i<data.NumWaypoints; ++i)
			{
				if (data.ID[i] == CurWayPoints.WayPoints[j].ID)
				{
					found = true;
					break;
				}
			}
			if (!found)
			{
				std::cout << "removing " << j << std::endl;
				CurWayPoints.remove(j);
			}
		}

		// TODO: ugly way of doing this
		if (data.NumWaypoints == 0)
			CurWayPoints.WayPointsNum = 0;

		// Set waypoints at mapself
		VecMsgType vecMsg;
		ToCont(CurWayPoints, vecMsg);
		vecMsg.push_back(PROT_MAPSELF_DATAIN_WAYPOINTS);
		writeToMapSelf(vecMsg);

		break;
	}
	case AP_PROT_WP_BOUNDS:
	{
		AutoPilotMsgWpBounds data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgWpBounds)))
			return;
		std::cout << get_cur_1ms() << " Received wp bounds:" << data << std::endl;
		break;
	}
	case AP_PROT_XBEE_MSG:
	{
		AutoPilotMsgXBeeMsg data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgXBeeMsg)))
			return;
		std::cout << get_cur_1ms() << " Received xbee msg:" << data << std::endl;
		break;
	}
	default:
	{
		Synchronize = true;
		std::cout << get_cur_1ms() << " Received unknown msg type from autopilot: " << LastReadHeader << std::endl;
		break;
	}
	}
}

bool CAutoPilot::ReadData(char* data, size_t size)
{
	if (size != LastReadHeader.DataSize)
	{
		Synchronize = true;
		return false;
	}

	if (size > 0)
		Serial->read(data, size);
	char checkSum1;
	Serial->read(&checkSum1, 1);

	char checkSum2 = 0;
	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		checkSum2 += ((char*)&LastReadHeader)[i];
	for (int i=0; i<size; ++i)
		checkSum2 += data[i];
	if (checkSum1 != checkSum2)
	{
		std::cout << "Read checksum error: " << +checkSum1 << " vs " << +checkSum2 << std::endl;
		return false;
	}
	return true;
}


bool CAutoPilot::SendHeader(EAutoPilotMsgType type, uint8_t dataSize)
{
	CheckSumOut = 0;
	AutoPilotMsgHeader msgHdr;
	msgHdr.Header = AP_PROT_HEADER;
	msgHdr.MsgType = type;
	msgHdr.TimeStamp = get_cur_1ms();
	//msgHdr.TimeStamp = 4919; // hex: 1337
	msgHdr.DataSize = dataSize;

	std::cout << "Writing header: " << msgHdr << std::endl;
	try
	{
		Serial->write((char*)&msgHdr, sizeof(AutoPilotMsgHeader));
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}

	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		CheckSumOut+=((char*)&msgHdr)[i];

	//std::cout << "CheckSum=" << +CheckSumOut << std::endl;
	if (dataSize == 0)
		Serial->write(&CheckSumOut, sizeof(CheckSumOut));

	std::cout << get_cur_1ms() << " Written:";
//	printf("Written:");
	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		std::cout << " " << +((uint8_t*)&msgHdr)[i];
//		printf(" %X", ((uint8_t*)&msgHdr)[i]);
	if (dataSize == 0)
		std::cout << " " << +(uint8_t)CheckSumOut;
//		printf(" %X", (uint8_t)CheckSumOut);
//	printf("\n");
	std::cout << std::endl;

	return true;
}

bool CAutoPilot::SendData(char* data, size_t size)
{
	for (int i=0; i<size; ++i)
		CheckSumOut+=data[i];
	Serial->write(data, size);
	Serial->write(&CheckSumOut, sizeof(CheckSumOut));

	std::cout << get_cur_1ms() << " Written:";
//	printf("Written:");
	for (int i=0; i<size; ++i)
		std::cout << " " << +((uint8_t*)&data)[i];
//		printf(" %X", ((uint8_t*)data)[i]);
//	printf(" %X", (uint8_t)CheckSumOut);
	std::cout << " " << +(uint8_t)CheckSumOut << std::endl;
//	printf("\n");
	return true;
}


void CAutoPilot::SetWayPoints(WayPointsStruct& wps)
{
	std::cout << get_cur_1ms() << " SetWayPoints num=" << wps.WayPointsNum << std::endl;
	if (wps.WayPointsNum < 1)
		return;

	std::cout << wps << std::endl;

	SendHeader(AP_PROT_SET_WAYPOINTS, sizeof(AutoPilotMsgWayPoints));

	AutoPilotMsgWayPoints msgWps;
	msgWps.NumWayPoints = wps.WayPointsNum;

	Position startPos;
	wps.WayPoints[0].GetStartPos(startPos);
	PlanOutFile << startPos.x() << " " << startPos.y() << " ";

	for (int i=0; i<wps.WayPointsNum; ++i)
	{
		LastWpId = (LastWpId+1) % 1024; // TODO: magic number (should be large enough to avoid id collisions)
		// TODO: set this to something
		msgWps.WayPoints[i].Id = LastWpId;
		msgWps.WayPoints[i].GroundSpeed = config.CruiseSpeed;
		msgWps.WayPoints[i].VerticalSpeed = wps.WayPoints[i].VerticalSpeed;

		Position endPos;
		wps.WayPoints[i].GetEndPos(endPos);

		PlanOutFile << endPos.x() << " " << endPos.y() << " ";

		switch(wps.WayPoints[i].wpMode)
		{
			case WP_LINE:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_LINE;
				msgWps.WayPoints[i].Line.From.X = wps.WayPoints[i].from.x() + config.OriginX; // Local to mercator coordinates
				msgWps.WayPoints[i].Line.From.Y = wps.WayPoints[i].from.y() + config.OriginY; // Local to mercator coordinates
				msgWps.WayPoints[i].Line.From.Z = wps.WayPoints[i].from.z();
				msgWps.WayPoints[i].Line.To.X = wps.WayPoints[i].to.x() + config.OriginX; // Local to mercator coordinates
				msgWps.WayPoints[i].Line.To.Y = wps.WayPoints[i].to.y() + config.OriginY; // Local to mercator coordinates
				msgWps.WayPoints[i].Line.To.Z = wps.WayPoints[i].to.z();
				break;
			}
			case WP_CIRCLE:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_CIRCLE;
				msgWps.WayPoints[i].Circle.Center.X = wps.WayPoints[i].to.x() + config.OriginX; // Local to mercator coordinates
				msgWps.WayPoints[i].Circle.Center.Y = wps.WayPoints[i].to.y() + config.OriginY; // Local to mercator coordinates
				msgWps.WayPoints[i].Circle.Center.Z = wps.WayPoints[i].to.z();
				msgWps.WayPoints[i].Circle.Radius = wps.WayPoints[i].Radius;
				break;
			}
			case WP_ARC:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_ARC;
				msgWps.WayPoints[i].Arc.Center.X = wps.WayPoints[i].to.x() + config.OriginX; // Local to mercator coordinates
				msgWps.WayPoints[i].Arc.Center.Y = wps.WayPoints[i].to.y() + config.OriginY; // Local to mercator coordinates
				msgWps.WayPoints[i].Arc.Center.Z = wps.WayPoints[i].to.z();
				msgWps.WayPoints[i].Arc.Radius = wps.WayPoints[i].Radius;
				msgWps.WayPoints[i].Arc.AngleStart = -(wps.WayPoints[i].AngleStart - 0.5*M_PI);
				if (msgWps.WayPoints[i].Arc.AngleStart > M_PI)
					msgWps.WayPoints[i].Arc.AngleStart -= 2*M_PI;
				if (msgWps.WayPoints[i].Arc.AngleStart < -M_PI)
					msgWps.WayPoints[i].Arc.AngleStart += 2*M_PI;

				msgWps.WayPoints[i].Arc.AngleArc = -(wps.WayPoints[i].AngleArc -0.5*M_PI);
				break;
			}
		}
	}
	PlanOutFile << std::endl;

	std::cout << msgWps << std::endl;

	SendData((char*)&msgWps, sizeof(AutoPilotMsgWayPoints));

	// Set waypoints at mapself
	VecMsgType vecMsg;
	ToCont(wps, vecMsg);
	vecMsg.push_back(PROT_MAPSELF_DATAIN_WAYPOINTS);
	writeToMapSelf(vecMsg);

	// Update current waypoints
	CurWayPoints = wps;
}
