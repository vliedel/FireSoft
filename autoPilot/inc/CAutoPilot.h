/**
 * @brief 
 * @file CAutoPilot.h
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

#ifndef CAUTOPILOT_H_
#define CAUTOPILOT_H_

#include "autoPilot.h"
#include "AutoPilotProt.h"
#include "StructToFromCont.h"
#include "BufferedAsyncSerial.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct AutoPilotConfig
{
	long TickTime; // us
	int Debug;
	std::string PortName;
	int SerialSpeed;
//	AutoPilotMsgField FieldGPS;
	long SensorReqIntervalTime; // ms
	long WPStatustReqIntervalTime; // ms
	long WPBoundReqIntervalTime; // ms
	long SetModeIntervalTime; // ms
	long SetLandingIntervalTime; // ms
	std::string PathFileName;
	std::string PlanFileName;

	float OriginX; // Mercator coordinates
	float OriginY; // Mercator coordinates
	float LandPointX;
	float LandPointY;
	float LandHeading;
	bool LandLeftTurn;

	float CruiseSpeed;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("autoPilot.TickTime");
		Debug = pt.get<int>("autoPilot.Debug");
		PortName = pt.get<std::string>("autoPilot.PortName");
		SerialSpeed = pt.get<int>("autoPilot.SerialSpeed");
//		FieldGPS.Origin.GpsLat	= pt.get<float>("field.Origin.GpsLat");
//		FieldGPS.Origin.GpsLong	= pt.get<float>("field.Origin.GpsLong");
//		FieldGPS.Origin.GpsZ 	= pt.get<float>("field.Origin.GpsZ");
//		FieldGPS.XBound.GpsLat	= pt.get<float>("field.XBound.GpsLat");
//		FieldGPS.XBound.GpsLong	= pt.get<float>("field.XBound.GpsLong");
//		FieldGPS.XBound.GpsZ 	= pt.get<float>("field.XBound.GpsZ");
//		FieldGPS.YBound.GpsLat	= pt.get<float>("field.YBound.GpsLat");
//		FieldGPS.YBound.GpsLong	= pt.get<float>("field.YBound.GpsLong");
//		FieldGPS.YBound.GpsZ 	= pt.get<float>("field.YBound.GpsZ");
//		FieldGPS.Home.GpsLat	= pt.get<float>("field.Home.GpsLat");
//		FieldGPS.Home.GpsLong	= pt.get<float>("field.Home.GpsLong");
//		FieldGPS.Home.GpsZ 		= pt.get<float>("field.Home.GpsZ");
		SensorReqIntervalTime		= pt.get<long>("autoPilot.SensorReqIntervalTime");
		WPStatustReqIntervalTime	= pt.get<long>("autoPilot.WPStatustReqIntervalTime");
		WPBoundReqIntervalTime		= pt.get<long>("autoPilot.WPBoundReqIntervalTime");
		SetModeIntervalTime			= pt.get<long>("autoPilot.SetModeIntervalTime");
		SetLandingIntervalTime		= pt.get<long>("autoPilot.SetLandingIntervalTime");
		PathFileName 				= pt.get<std::string>("autoPilot.PathFileName");
		PlanFileName 				= pt.get<std::string>("autoPilot.PlanFileName");

		OriginX = pt.get<float>("field.OriginX");
		OriginY = pt.get<float>("field.OriginY");
		LandPointX = pt.get<float>("field.LandPointX");
		LandPointY = pt.get<float>("field.LandPointY");
		LandHeading = pt.get<float>("field.LandHeading");
		LandLeftTurn = pt.get<bool>("field.LandLeftTurn");

		CruiseSpeed = pt.get<float>("UAV.CruiseSpeed");
	}
};

class CAutoPilot : public autoPilot
{
	private:
		// Data
		int* IntMsg;
		VecMsgType* VecMsg;
		VecMsgType VecMsgReply;

		int UavId;
		AutoPilotConfig config;

		BufferedAsyncSerial *Serial;
		bool Synchronize;
		char CheckSumOut;	// Sum of bytes of whole msg written
		AutoPilotMsgHeader LastReadHeader;
		bool LastReadHeaderUsed; // True when new header should be read, false if we should use LastReadHeader
		long LastReqTimeSensorData;
		long LastReqTimeWpStatus;
		long LastReqTimeWpBounds;


		long LastSetTimeMode;
		AutoPilotMsgMode LastSetMode;

		long LastSetTimeLanding;
		AutoPilotMsgLanding LastSetLanding;

		//std::vector<WayPoint> wps;
		WayPointsStruct CurWayPoints;
		uint32_t LastWpId;
		//uint32_t CurWayPointIds[MAPSELF_MAX_WAYPOINTS];

		bool logPath;
		bool logPlan;
		std::ofstream PathOutFile;
		std::ofstream PlanOutFile;

		// Functions
		bool SynchronizeUart(AutoPilotMsgHeader& msgHdr);
		void ReadUart();
		bool ReadData(char* data, size_t size);
		bool SendHeader(EAutoPilotMsgType type, uint8_t dataSize);
		bool SendData(char* data, size_t size);

		void SetWayPoints(WayPointsStruct& wps);

	public:
		~CAutoPilot();
		void Init(std::string module_id);
		void Tick();
};



}
#endif // CAUTOPILOT_H_
