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
}

void CAutoPilot::Init(std::string module_id)
{
	autoPilot::Init(module_id);
	UavId = atoi(module_id.c_str());
	config.load("config.json");

	try
	{
		Serial = new BufferedAsyncSerial(config.PortName, AP_PROT_SERIAL_SPEED);
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
	LastSetLanding.LandPoint.X = config.LandPointX + config.OriginX;
	LastSetLanding.LandPoint.Y = config.LandPointY + config.OriginY;
	LastSetLanding.LandPoint.Z = 0;
	LastSetLanding.LandHeading = config.LandHeading;
	LastSetLanding.LandLeftTurn = config.LandLeftTurn;

	// Set the field at init
//	SendHeader(AP_PROT_SET_FIELD, sizeof(AutoPilotMsgField));
//	AutoPilotMsgField msgField;
//	msgField.Origin.GpsLat = 51.998902;
//	msgField.Origin.GpsLong = 4.373504;
//	msgField.Origin.GpsZ = 0;
//	msgField.XBound.GpsLat = 51.999183;
//	msgField.XBound.GpsLong = 4.378738;
//	msgField.XBound.GpsZ = 0;
//	msgField.YBound.GpsLat = 51.999837;
//	msgField.YBound.GpsLong = 4.374415;
//	msgField.YBound.GpsZ = 0;
//	msgField.Home.GpsLat = 51.998902;
//	msgField.Home.GpsLong = 4.373504;
//	msgField.Home.GpsZ = 0;
//	Serial->write((char*)&msgField, sizeof(AutoPilotMsgField));

//	SendData((char*)&(config.FieldGPS), sizeof(AutoPilotMsgField));
}

void CAutoPilot::Tick()
{
	IntMsg = readCommand(false);
	if (IntMsg != NULL)
	{

	}

	// Check if there is something to read
	ReadUart();

//	// From time to time: request AP for sensor data
//	if (get_cur_1ms() - LastReqTimeSensorData > config.SensorReqIntervalTime)
//	{
//		//std::cout << "Sending sensor request" << std::endl;
//		SendHeader(AP_PROT_REQ_SENSORDATA, 0);
//		LastReqTimeSensorData = get_cur_1ms();
//	}

//	// From time to time: request AP for wp data
//	if (get_cur_1ms() - LastReqTimeWpStatus > config.WPStatustReqIntervalTime)
//	{
//		//std::cout << "Sending wp status request" << std::endl;
//		SendHeader(AP_PROT_REQ_WP_STATUS, 0);
//		LastReqTimeWpStatus = get_cur_1ms();
//	}


	// TODO: set it every time? set on change maybe better, but msg might be missed
	// From time to time: set correct mode
	if (get_cur_1ms() - LastSetTimeMode > 125) // TODO: magic number
	{
		SendHeader(AP_PROT_SET_MODE, sizeof(AutoPilotMsgMode));
		SendData((char*)&LastSetMode, sizeof(AutoPilotMsgMode));
		LastSetTimeMode = get_cur_1ms();
	}

	if (get_cur_1ms() - LastSetTimeLanding > 250) // TODO: magic number
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
					break;
				}
			}
		}
		VecMsg->clear();
	}

	// Read from map self
	VecMsg = readMapSelf(false);
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
					LastSetLanding.LandPoint.X = land.Pos.x() + config.OriginX;
					LastSetLanding.LandPoint.Y = land.Pos.y() + config.OriginY;
					LastSetLanding.LandPoint.Z = land.Pos.z();
					LastSetLanding.LandHeading = land.Heading.angle();
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
	if (Serial->available() < 2*sizeof(AutoPilotMsgHeader))
		return false;

	//std::cout << "Synchronizing..";
	char chr;
	AutoPilotMsgHeaderType header;
	Serial->read((char*)&header, sizeof(AutoPilotMsgHeaderType));
	while (Serial->available() >= sizeof(AutoPilotMsgHeader))
	{
		//std::cout << " header=" << header;
		if (header == AP_PROT_HEADER)
		{
			msgHdr.Header = header;
			Serial->read((char*)&msgHdr+sizeof(AutoPilotMsgHeaderType), sizeof(AutoPilotMsgHeader)-sizeof(AutoPilotMsgHeaderType));
			Synchronize = false;
			LastReadHeaderUsed = false;
			//std::cout << std::endl;
			return true;
		}
		Serial->read(&chr, 1);
		header = (header << 8) | chr;
	}
	//std::cout << std::endl;
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
		std::cout << "Received sensor data:" << data << std::endl;


		// Write to state to mapSelf
		UavGeomStruct geom;
		geom.Pos.x() = data.Position.X - config.OriginX;
		geom.Pos.y() = data.Position.Y - config.OriginY;
		geom.Pos.z() = data.Position.Z;
		geom.GroundSpeed = data.GroundSpeed;
		geom.VerticalSpeed = data.VerticalSpeed;
		geom.Heading.angle() = data.Heading;
		geom.Yaw.angle() = data.Yaw;
		geom.Pitch.angle() = data.Pitch;
		geom.Roll.angle() = data.Roll;
		std::vector<float> vecMsg;
		vecMsg.clear();
		ToCont(geom, vecMsg);
		vecMsg.push_back(PROT_MAPSELF_DATAIN_GEOM);
		writeMapSelf(vecMsg);

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
		writeMapSelf(vecMsg);

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
			writeMapSelf(vecMsg);
		}

		vecMsg.clear();
		vecMsg.push_back(data.BatteryLeft/1000); // BatteryLeft is in ms?
		vecMsg.push_back(PROT_MAPSELF_DATAIN_BATTERY);
		writeMapSelf(vecMsg);

		break;
	}
	case AP_PROT_WP_STATUS:
	{
		AutoPilotMsgWpStatus data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgWpStatus)))
			return;
		std::cout << "Received wp status:" << data << std::endl;

//		// Set waypoints at mapself
//		VecMsgType vecMsg;
//		ToCont(wps, vecMsg);
//		vecMsg.push_back(PROT_MAPSELF_DATAIN_WAYPOINTS);
//		writeMapSelf(vecMsg);

		break;
	}
	case AP_PROT_WP_BOUNDS:
	{
		AutoPilotMsgWpBounds data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgWpBounds)))
			return;
		std::cout << "Received wp bounds:" << data << std::endl;
		break;
	}
	case AP_PROT_XBEE_MSG:
	{
		AutoPilotMsgXBeeMsg data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgXBeeMsg)))
			return;
		std::cout << "Received xbee msg:" << data << std::endl;
		break;
	}
	default:
	{
		Synchronize = true;
		std::cout << "Received unknown msg type from autopilot: " << LastReadHeader << std::endl;
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

	printf("Written:");
	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		printf(" %X", ((uint8_t*)&msgHdr)[i]);
	if (dataSize == 0)
		printf(" %X", (uint8_t)CheckSumOut);
	printf("\n");

	return true;
}

bool CAutoPilot::SendData(char* data, size_t size)
{
	for (int i=0; i<size; ++i)
		CheckSumOut+=data[i];
	Serial->write(data, size);
	Serial->write(&CheckSumOut, sizeof(CheckSumOut));

	printf("Written:");
	for (int i=0; i<size; ++i)
		printf(" %X", ((uint8_t*)data)[i]);
	printf(" %X", (uint8_t)CheckSumOut);
	printf("\n");
	return true;
}


void CAutoPilot::SetWayPoints(WayPointsStruct& wps)
{
	SendHeader(AP_PROT_SET_WAYPOINTS, sizeof(AutoPilotMsgWayPoints));

	AutoPilotMsgWayPoints msgWps;
	msgWps.NumWayPoints = wps.WayPointsNum;
	for (int i=0; i<wps.WayPointsNum; ++i)
	{
		// TODO: set this to something
		msgWps.WayPoints[i].Id = 0;
		msgWps.WayPoints[i].GroundSpeed = 0;
		msgWps.WayPoints[i].VerticalSpeed = wps.WayPoints[i].VerticalSpeed;

		switch(wps.WayPoints[i].wpMode)
		{
			case WP_LINE:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_LINE;
				msgWps.WayPoints[i].Line.From.X = wps.WayPoints[i].from.x() + config.OriginX;
				msgWps.WayPoints[i].Line.From.Y = wps.WayPoints[i].from.y() + config.OriginY;
				msgWps.WayPoints[i].Line.From.Z = wps.WayPoints[i].from.z();
				msgWps.WayPoints[i].Line.To.X = wps.WayPoints[i].to.x() + config.OriginX;
				msgWps.WayPoints[i].Line.To.Y = wps.WayPoints[i].to.y() + config.OriginY;
				msgWps.WayPoints[i].Line.To.Z = wps.WayPoints[i].to.z();
				break;
			}
			case WP_CIRCLE:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_CIRCLE;
				msgWps.WayPoints[i].Circle.Center.X = wps.WayPoints[i].to.x() + config.OriginX;
				msgWps.WayPoints[i].Circle.Center.Y = wps.WayPoints[i].to.y() + config.OriginY;
				msgWps.WayPoints[i].Circle.Center.Z = wps.WayPoints[i].to.z();
				msgWps.WayPoints[i].Circle.Radius = wps.WayPoints[i].Radius;
				break;
			}
			case WP_ARC:
			{
				msgWps.WayPoints[i].WpType = AP_PROT_WP_ARC;
				msgWps.WayPoints[i].Arc.Center.X = wps.WayPoints[i].to.x() + config.OriginX;
				msgWps.WayPoints[i].Arc.Center.Y = wps.WayPoints[i].to.y() + config.OriginY;
				msgWps.WayPoints[i].Arc.Center.Z = wps.WayPoints[i].to.z();
				msgWps.WayPoints[i].Arc.Radius = wps.WayPoints[i].Radius;
				msgWps.WayPoints[i].Arc.AngleStart = wps.WayPoints[i].AngleStart;
				msgWps.WayPoints[i].Arc.AngleArc = wps.WayPoints[i].AngleArc;
				break;
			}
		}
	}
	SendData((char*)&msgWps, sizeof(AutoPilotMsgWayPoints));

	// Set waypoints at mapself
	VecMsgType vecMsg;
	ToCont(wps, vecMsg);
	vecMsg.push_back(PROT_MAPSELF_DATAIN_WAYPOINTS);
	writeMapSelf(vecMsg);
}
