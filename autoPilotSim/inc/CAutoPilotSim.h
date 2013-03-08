/**
 * @brief 
 * @file CAutoPilotSim.h
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

#ifndef CAUTOPILOTSIM_H_
#define CAUTOPILOTSIM_H_

#include "autoPilotSim.h"
#include "StructToFromCont.h"
#include "Defs.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct AutoPilotSimConfig
{
	long TickTime; // us
	int Debug;
	float CruiseSpeed;
	float MaxRollAngle;
	float MaxVertSpeed;
	float BatteryTime;
	float RollAngleGain;
	float VerticalSpeedGain;
	float CarrotReachTime;
	float LineAheadTime;
	float PreBankAngleInner;
	float PreBankAngleOuter;
	float LandPointX;
	float LandPointY;
	float LandHeading;
	bool LandLeftTurn;
	float LandLength;
	float LandRadius;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("autoPilotSim.TickTime");
		Debug = pt.get<int>("autoPilotSim.Debug");
		CruiseSpeed 	= pt.get<float>("UAV.CruiseSpeed");
		MaxRollAngle 	= pt.get<float>("UAV.MaxRollAngle");
		MaxVertSpeed 	= pt.get<float>("UAV.MaxVertSpeed");
		BatteryTime 	= pt.get<float>("UAV.BatteryTime");
		RollAngleGain 		= pt.get<float>("autoPilotSim.RollAngleGain");
		VerticalSpeedGain 	= pt.get<float>("autoPilotSim.VerticalSpeedGain");
		CarrotReachTime 	= pt.get<float>("autoPilotSim.CarrotReachTime");
		LineAheadTime 		= pt.get<float>("autoPilotSim.LineAheadTime");
		PreBankAngleInner 	= pt.get<float>("autoPilotSim.PreBankAngleInner");
		PreBankAngleOuter 	= pt.get<float>("autoPilotSim.PreBankAngleOuter");
		LandPointX = pt.get<float>("field.LandPointX");
		LandPointY = pt.get<float>("field.LandPointY");
		LandHeading = pt.get<float>("field.LandHeading");
		LandLeftTurn = pt.get<bool>("field.LandLeftTurn");
		LandLength = pt.get<float>("field.LandLength");
		LandRadius = pt.get<float>("field.LandRadius");
	}
};

class CAutoPilotSim : public autoPilotSim
{

	//private:
	public:
		int* IntMsg;
		VecMsgType* VecMsg;
		VecMsgType VecMsgReply;

		int UavId;
		AutoPilotSimConfig config;

		UavGeomStruct Geom;
		//Speed Acceleration;
		//Rotation Rot;

		Rotation2DType RollAngle; // Roll angle in rad
		float GroundSpeed; // Velocity in horizontal plane in m/s
		float Heading; // Heading in horizontal plane in rad
		float HeadingRate; // Heading change rate in rad/s

		float WindHeading; // Heading of the wind in rad
		float WindSpeed; // Speed of the wind in horizontal plane in m/s

		WayPointsStruct WayPoints;
		UAVState State;
		float BatteryTimeLeft; // In seconds

		bool LandStraight;

		LandingStruct Landing;

	public:
		CAutoPilotSim();
		~CAutoPilotSim();
		void Init(std::string module_id);
		void ReInit(int moduleId = -1);
		void Tick();
		void TimeStep(float dt);
		bool GetCarrotLine(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt);
		bool GetCarrotCircle(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt);
		bool GetCarrotFree(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt);

		void AddWayPoint(WayPoint& wp);



		friend std::ostream& operator<<(std::ostream& os, CAutoPilotSim& sim);
};

}
#endif // CAUTOPILOTSIM_H_
