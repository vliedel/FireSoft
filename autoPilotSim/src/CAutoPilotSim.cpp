/**
 * @brief 
 * @file CAutoPilotSim.cpp
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

#include "CAutoPilotSim.h"
#include "Protocol.h"
#include <cmath>
#include "CTime.h"
#include "Print.hpp"
#include "AutoPilotProt.h"

using namespace rur;

CAutoPilotSim::CAutoPilotSim():
	RollAngle(0)
{

}

CAutoPilotSim::~CAutoPilotSim()
{

}

void CAutoPilotSim::Init(std::string module_id)
{
	autoPilotSim::Init(module_id);
	config.load("config.json");
	ReInit(atoi(module_id.c_str()));
}

void CAutoPilotSim::ReInit(int moduleId)
{
	if (moduleId >= 0)
		UavId = moduleId;
	Geom.Init();
	RollAngle.angle() = 0;
	GroundSpeed = 0;
	Heading = 0;
	HeadingRate = 0;
	WindHeading = 0;
	WindSpeed = 0;
	WayPoints.clear();
	BatteryTimeLeft = config.BatteryTime;
	//State = UAVSTATE_FLYING;
	State = UAVSTATE_LANDED;

	Landing.Pos.x() = config.LandPointX;
	Landing.Pos.y() = config.LandPointY;
	Landing.Heading.angle() = config.LandHeading;
	Landing.LeftTurn = config.LandLeftTurn;
	Landing.Length = config.LandLength;
	Landing.Radius = config.LandRadius;
	LandStraight = false;

}

void CAutoPilotSim::Tick()
{
//	IntMsg = readCommand(false);
//	if (IntMsg != NULL)
//	{
//
//	}

	VecMsg = readSimCommand(false);
	if (!VecMsg->empty())
	{
		std::cout << "AP " << UavId << " from Sim: ";
		dobots::print(VecMsg->begin(), VecMsg->end());

		// Reply with VecMsgReply, the different commands can add data to VecMsgReply
		VecMsgReply.clear();
		std::vector<float>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			//std::cout << "type=" << type << std::endl;

			switch(type)
			{
				case PROT_SIMCMD_INIT:
				{
					ReInit();
					//std::vector<float> vecMsg;
					//vecMsg.clear();
					//vecMsg.push_back(PROT_SIMSTAT_ACK);
					//writeSimState(vecMsg);
					break;
				}
				case PROT_SIMCMD_SET_GEOM:
				{
					//std::cout << "setting geom" << std::endl;
					it = FromCont(Geom, it, VecMsg->end());
					break;
				}
				case PROT_SIMCMD_SET_GEOM_EXTRA:
				{
					Heading = *it++;
					RollAngle.angle() = *it++;
					break;
				}
				case PROT_SIMCMD_SET_WAYPOINTS:
					it = FromCont(WayPoints, it, VecMsg->end());
					break;
				case PROT_SIMCMD_SET_BATTERY:
					//std::cout << "setting battery" << std::endl;
					BatteryTimeLeft = *it++;
					break;
				case PROT_SIMCMD_SET_STATE:
				{
					State = (UAVState)*it++;
					VecMsgType vecMsg;
					vecMsg.push_back(State);
					vecMsg.push_back(PROT_MAPSELF_DATAIN_STATE);
					writeToMapSelf(vecMsg);
					break;
				}
				case PROT_SIMCMD_SET_WIND:
					WindSpeed = *it++;
					WindHeading = *it++;
					break;
				case PROT_SIMCMD_TIMESTEP:
				{
					TimeStep(*it++);
					break;
				}
			}
		}
		VecMsg->clear();
		VecMsgReply.push_back(PROT_SIMSTAT_ACK);
		writeSimState(VecMsgReply);
	}


	// From WayPoint Planner
	VecMsg = readFromWayPointPlanner(false);
	if (!VecMsg->empty())
	{
//		std::cout << get_cur_1ms() << " AP " << UavId << " from WpPlanner: ";
//		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgReply.clear();
		std::vector<float>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_AP_SET_MODE:
				{
					uint8_t mode = *it++;
					switch (mode)
					{
					case AP_PROT_MODE_LAND:
						if (State != UAVSTATE_LANDED)
							State = UAVSTATE_LANDING;
						break;
					case AP_PROT_MODE_HOME:
						if (State != UAVSTATE_LANDED && State != UAVSTATE_LANDING)
							State = UAVSTATE_GOING_HOME;
						break;
					case AP_PROT_MODE_STAY:
						if (State != UAVSTATE_LANDED && State != UAVSTATE_LANDING)
							State = UAVSTATE_STAYING;
						break;
					}

					VecMsgType vecMsg;
					vecMsg.push_back(State);
					vecMsg.push_back(PROT_MAPSELF_DATAIN_STATE);
					writeToMapSelf(vecMsg);

					break;
				}
				case PROT_AP_SET_WAYPOINTS:
				{
					it = FromCont(WayPoints, it, VecMsg->end());
					std::cout << "Waypoints:" << WayPoints << std::endl;
					break;
				}
			}
		}
		VecMsg->clear();
	}

	// From MapSelf
	VecMsg = readFromMapSelf(false);
	if (!VecMsg->empty())
	{
//		std::cout << get_cur_1ms() << " AP " << UavId << " from MapSelf: ";
//		dobots::print(VecMsg->begin(), VecMsg->end());

		VecMsgReply.clear();
		std::vector<float>::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			switch (type)
			{
				case PROT_AP_SET_MODE:
				{
					uint8_t mode = *it++;
					switch (mode)
					{
						case AP_PROT_MODE_LAND:
							if (State != UAVSTATE_LANDED)
								State = UAVSTATE_LANDING;
							break;
						case AP_PROT_MODE_HOME:
							if (State != UAVSTATE_LANDED && State != UAVSTATE_LANDING)
								State = UAVSTATE_GOING_HOME;
							break;
						case AP_PROT_MODE_STAY:
							if (State != UAVSTATE_LANDED && State != UAVSTATE_LANDING)
								State = UAVSTATE_STAYING;
							break;
					}

					VecMsgType vecMsg;
					vecMsg.push_back(State);
					vecMsg.push_back(PROT_MAPSELF_DATAIN_STATE);
					writeToMapSelf(vecMsg);

					break;
				}
				case PROT_AP_SET_LAND:
				{
					it = FromCont(Landing, it, VecMsg->end());
					Landing.Length = config.LandLength;
					Landing.Radius = config.LandRadius;
					break;
				}
			}
		}
		VecMsg->clear();
	}


	usleep(config.TickTime);
}

void CAutoPilotSim::TimeStep(float dt)
{
	VecMsgType vecMsg;
	if (State == UAVSTATE_LANDED)
		return;

	Geom.Pos.x() += Geom.GroundSpeed * dt * cos(Geom.Heading.angle());
	Geom.Pos.y() += Geom.GroundSpeed * dt * sin(Geom.Heading.angle());

	Position carrot;
	bool result;
	while (!WayPoints.empty())
	{
		switch (WayPoints[0].wpMode)
		{
			case WP_LINE:
				result = GetCarrotLine(carrot, WayPoints[0], Geom.Pos, dt);
				break;
			case WP_CIRCLE:
			case WP_ARC:
				result = GetCarrotCircle(carrot, WayPoints[0], Geom.Pos, dt);
				break;
			case WP_FREE:
			{
				std::cout << "Error: not allowed to use free waypoints" << std::endl;
				break;
			}
		}
		if (!result)
		{
			WayPoints.pop_front();
			//continue;
		}
		else
			break;
	}

	if (State == UAVSTATE_LANDING)
	{
		// If current heading is almost the required heading, then go straight to landing spot
		float landHeading = Landing.Heading.angle();
		if (landHeading > M_PI)
			landHeading -= 2*M_PI;
		if (LandStraight ||
				((landHeading-2*M_PI/50 < Geom.Heading.angle()) && (Geom.Heading.angle() < landHeading+2*M_PI/50)
				&& (Geom.Pos.z() < 55)))
		{
			// Let's land
			LandStraight = true;
			std::cout << "Landing " << Landing.Heading.angle() << std::endl;
//			Heading = atan2(Geom.Pos.y()-Landing.Pos.y(), Geom.Pos.x()-Landing.Pos.x()) + M_PI;
//			if (Heading > M_PI)
//				Heading -= 2*M_PI;
			carrot << Landing.Pos.x(), Landing.Pos.y(), 0;
			WayPoint wp;
			wp.to = carrot;
			wp.from = Geom.Pos;
			wp.wpMode = WP_LINE;
			WayPoints.clear();
			WayPoints.push_back(wp);

			float dx = Geom.Pos.x() - Landing.Pos.x();
			float dy = Geom.Pos.y() - Landing.Pos.y();
			if (sqrt(dx*dx+dy*dy) < 10)
			{
				std::cout << "Landed" << std::endl;
				Geom.Pos.z() = 0;
				State = UAVSTATE_LANDED;
				vecMsg.clear();
				vecMsg.push_back(State);
				vecMsg.push_back(PROT_MAPSELF_DATAIN_STATE);
				writeToMapSelf(vecMsg);
			}
		}

		else
		{
			std::cout << "spiral downwards" << std::endl;
			WayPoint wp;
			wp.to = Landing.Pos;
			wp.to.z() = 50;
			wp.to.x() += Landing.Length * cos(Landing.Heading.angle()+M_PI);
			wp.to.y() += Landing.Length * sin(Landing.Heading.angle()+M_PI);
			if (Landing.LeftTurn)
			{
				wp.to.x() += Landing.Radius * cos(Landing.Heading.angle()+0.5*M_PI);
				wp.to.y() += Landing.Radius * sin(Landing.Heading.angle()+0.5*M_PI);
				wp.AngleArc = 1.0;
			}
			else
			{
				wp.to.x() += Landing.Radius * cos(Landing.Heading.angle()-0.5*M_PI);
				wp.to.y() += Landing.Radius * sin(Landing.Heading.angle()-0.5*M_PI);
				wp.AngleArc = -1.0;
			}
			wp.wpMode = WP_CIRCLE;
			wp.Radius = Landing.Radius;
			WayPoints.clear();
			WayPoints.push_back(wp);
		}
	}
	else
		LandStraight = false;

	// Set waypoints at mapself
	vecMsg.clear();
	ToCont(WayPoints, vecMsg);
	vecMsg.push_back(PROT_MAPSELF_DATAIN_WAYPOINTS);
	writeToMapSelf(vecMsg);

	std::cout << get_cur_1ms();
	if (!WayPoints.empty())
	{
		std::cout << " pos=[" << Geom.Pos.transpose() << "]";
		std::cout << " heading=" << Heading;
		std::cout << " wp=[" << WayPoints[0] << "]";
		std::cout << " carrot=[" << carrot.transpose() << "]" << std::endl;

		// Calculate roll angle from given carrot
		// Roll angle
		// roll_angle = path_angle_error * gain;
		// with gain about 1 to 1.2
		// path_angle_error = angle between ground course and the carrot remapped from -180 to 180 so it takes the shortest turn.
		// roll_angle is limited to +/- 30 degrees.
		// roll_angle += perturbation;                      // see below point 3)
		Position diff = carrot - Geom.Pos;
		//std::cout << "diff = " << std::endl << WayPoints[0].to << std::endl << " - " << std::endl << Geom.Pos << std::endl << " = " << std::endl << diff << std::endl;
		float error = atan2(diff.y(), diff.x()) - Heading;
		//std::cout << "error = " << atan2(diff.y(), diff.x()) << " - " << Heading << " = " << error << std::endl;
		if (error > M_PI)
			error -= 2*M_PI;
		if (error < -M_PI)
			error += 2*M_PI;
		RollAngle.angle() = error * 1.1;
	}
	else
	{
		std::cout << " Error: no waypoints to follow!" << std::endl;
		RollAngle.angle() = 0;
	}

	if (RollAngle.angle() > config.MaxRollAngle)
		RollAngle.angle() = config.MaxRollAngle;
	if (RollAngle.angle() < -config.MaxRollAngle)
		RollAngle.angle() = -config.MaxRollAngle;
	//RollAngle.angle() += pertubation;

	// constant airspeed = 22m/s;
	// groundspeed =  constant airspeed  - cos( wind_direction - heading) * wind_speed;
	// groundspeed is function of heading and wind: e.g. north wind of 10m/s -> groundspeed = 12m/s when flying north, 22 when flying east and west and 32 when flying south.
	GroundSpeed = config.CruiseSpeed - cos(WindHeading - Heading) * WindSpeed;

	// heading_rate = 9.81 / ground_speed * tan (roll_angle);     // the faster it flies, the slower it turns (you need more centrifugal force to bend the path)
	// heading += heading_rate * DT;
	HeadingRate = GRAVITY / GroundSpeed * tan(RollAngle.angle());
	Heading += HeadingRate * dt;
	if (Heading > M_PI)
		Heading -= 2*M_PI;
	if (Heading < -M_PI)
		Heading += 2*M_PI;
	//Geom.Speed.x() = cos(Heading) * GroundSpeed;
	//Geom.Speed.y() = sin(Heading) * GroundSpeed;
	Geom.GroundSpeed = GroundSpeed;

	// Climb speed
	// climb_speed = altitude_error * gain;
	// gain = 0.1 so 10 meter to low = climb at 1 m/s
	// climb_speed is limited to +/- 3 m/s;
	if (!WayPoints.empty())
	{
		Geom.VerticalSpeed = (WayPoints[0].to.z() - Geom.Pos.z()) * 0.1;
		if (Geom.VerticalSpeed > config.MaxVertSpeed)
			Geom.VerticalSpeed = config.MaxVertSpeed;
		if (Geom.VerticalSpeed < -config.MaxVertSpeed)
			Geom.VerticalSpeed = -config.MaxVertSpeed;
	}
	else
		Geom.VerticalSpeed = 0;
	Geom.Pos.z() += Geom.VerticalSpeed * dt;
	Geom.Heading.angle() = Heading;
	Geom.Roll = RollAngle;
	Geom.Pitch = atan2(Geom.VerticalSpeed, GroundSpeed);


	BatteryTimeLeft -= dt; // TODO: way too simple, should depend on speed etc


	// Write to state to mapSelf
	//std::vector<float> vecMsg;
	vecMsg.clear();
	ToCont(Geom, vecMsg);
	vecMsg.push_back(PROT_MAPSELF_DATAIN_GEOM);
	writeToMapSelf(vecMsg);

	vecMsg.clear();
	vecMsg.push_back(BatteryTimeLeft);
	vecMsg.push_back(PROT_MAPSELF_DATAIN_BATTERY);
	writeToMapSelf(vecMsg);


	// Write state to simulator
	// This all has to go in 1 msg, so that we're not dependent on msg order
	// Therefore, we put the type in front on the contents
	VecMsgReply.push_back(PROT_SIMSTAT_GEOM);
	ToCont(Geom, VecMsgReply);

	VecMsgReply.push_back(PROT_SIMSTAT_BATTERY);
	VecMsgReply.push_back(BatteryTimeLeft);

	VecMsgReply.push_back(PROT_SIMSTAT_WP);
	WayPoint wp = WayPoints[0];
	wp.to = carrot;

	//usleep(50*1000);

	ToCont(wp, VecMsgReply);
}

bool CAutoPilotSim::GetCarrotLine(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt)
{
	// Check if we already reached the waypoint, if so, remove it
	Position diff = wp.to - pos;
	if (diff.norm() < config.CruiseSpeed * config.CarrotReachTime) // TODO: use real speed
		return false;

	// Translate coordinate system to one with wp.from as center
	Position line = wp.to - wp.from;
	Position unitLine = line.normalized();

	// Projection of uav pos on line
	carrot = (pos - wp.from).dot(unitLine) * unitLine;

	// Check if projection if further than wp.to
	// (this statement is also be true when projection is norm(line) in front of wp.from, which shouldn't happen)
	if (carrot.norm() > line.norm())
		return false;

	// Translate coordinate system back to (0,0) and add the 5 seconds ahead
	carrot += wp.from + unitLine * config.LineAheadTime * config.CruiseSpeed; // TODO: use real speed
	return true;
}

bool CAutoPilotSim::GetCarrotCircle(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt)
{
	float r = wp.Radius;
	// px, py is the position of the UAV relative to circle center
	float px = pos.x() - wp.to.x();
	float py = pos.y() - wp.to.y();

	//std::cout << "px=" << px << " py=" << py;
	float theta = atan2(py,px); // -PI to +PI

	if (theta < 0)
		theta += 2*M_PI;
	// theta: 0 to 2PI

	float angleStart = wp.AngleStart; // 0 to 2PI
	float angleArc = wp.AngleArc; // -2PI to 2PI
	if (wp.wpMode == WP_ARC)
	{
		float angleEnd = angleStart+angleArc; // -2PI to 4PI

		if (angleEnd > 2*M_PI)
			angleEnd -= 2*M_PI;
		else if (angleEnd < 0)
			angleEnd += 2*M_PI;
		// angleEnd: 0 to 2PI

		// Check if the arc is already finished
		if (angleArc < 0) // Right, aka clock wise
		{
			float angleHalf = angleEnd - (2*M_PI + angleArc)/2;
			// angleHalf: -PI to 2PI
			if ((angleEnd > theta && theta > angleHalf) || (angleEnd > theta-2*M_PI && theta-2*M_PI > angleHalf))
				return false;
		}
		else
		{
			float angleHalf = angleEnd + (2*M_PI - angleArc)/2;
			// angleHalf: 0 to 3PI
			if ((angleEnd < theta && theta < angleHalf) || (angleEnd < theta+2*M_PI && theta+2*M_PI < angleHalf))
				return false;
		}
	}

	// Carrot position relative to circle center
	Position u(0,0,0);
	//u.z() = 0;

	// If the UAV is inside the circle, place carrot on the circle so that the line to the UAV is perpendicular to radius
	if (px*px+py*py < r*r)
	{
		// TODO: ask Delft how to actually do this
		float preBankAngle = config.PreBankAngleInner;
		float alpha = acos((px*px+py*py) / (r*r)); // 0 to +PI
		if (angleArc < 0) // Right, aka clock wise
		{
			alpha*=-1;
			preBankAngle*=-1;
		}
		u.x() = r*cos(theta+alpha+preBankAngle);
		u.y() = r*sin(theta+alpha+preBankAngle);

		std::cout << " rp=" << px*px+py*py << " theta=" << theta << " alpha=" << alpha << std::endl;

	}
	// If the UAV is outside the circle, place carrot on circle so that the line to the UAV is tangent
	else
	{
		if (py < -0.1 || 0.1 < py)
		{
			float d = sqrt(py*py*r*r*(px*px+py*py-r*r));
			if (angleArc < 0) // Right, aka clock wise
				d*=-1;
			if (py <= 0)
				d*=-1;

			u.x() =  (px*r*r - d) / (px*px + py*py);
			u.y() =  (py*py*r*r + px*d) / (px*px*py + py*py*py);
		}
		else
		{
			u.x() =  r*r/px;
			u.y() =  r*sqrt(px*px-r*r)/px;
			if (angleArc < 0) // Right, aka clock wise
				u.y() *= -1;
		}

		// Pre bank angle
		// TODO: ask Delft how to actually do this
		float preBankAngle = config.PreBankAngleOuter;
		if (angleArc < 0)
			preBankAngle*=-1;
		float alpha = atan2(u.y(), u.x()); // -PI to +PI
		u.x() = r*cos(alpha+preBankAngle);
		u.y() = r*sin(alpha+preBankAngle);
	}

	carrot = u + wp.to;
	return true;
}

bool CAutoPilotSim::GetCarrotFree(Position& carrot, const WayPoint& wp, const Position& pos, const float& dt)
{
	// Check if we already reached the waypoint, if so, remove it
	Position diff = wp.to - pos;
	if (diff.norm() < config.CruiseSpeed * config.CarrotReachTime) // use real speed
		return false;
	carrot = wp.to;
	return true;
}


void CAutoPilotSim::AddWayPoint(WayPoint& wp)
{
	WayPoints.push_back(wp);
}

namespace rur
{
std::ostream& operator<<(std::ostream& os, CAutoPilotSim& sim)
{
//	os << "Currently " << sim.WayPoints.size() << " waypoints:" << std::endl;
//	std::vector<WayPoint>::iterator it;
//	for (it = sim.WayPoints.begin(); it != sim.WayPoints.end(); ++it)
//	{
//		os << *it << std::endl;
//	}
	os << "Pos=" << std::endl << sim.Geom.Pos << std::endl;
	os << "Speed=" << std::endl << sim.Geom.GroundSpeed << std::endl;
	os << "Heading=" << sim.Heading << " RollAngle=" << sim.RollAngle.angle() << std::endl;
	return os;
}
}
