/**
 * @brief 
 * @file UAVStructs.h
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
 * @date          Aug 14, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef UAVSTRUCTS_H_
#define UAVSTRUCTS_H_

#define UAVS_NUM 10
#define MAPSELF_MAX_WAYPOINTS 16
#define MAPSELF_GS_CMDS_HIST 16
#define MAPUAV_RADIOMSG_HIST 16
#define UAVSTRUCT_NEXTWP_NUM 3


#include "Geometry.h"
#include "Waypoint.h"
#include "UAVState.h"
#include "RadioStructs.h"
//#include "Defs.h"
#include <vector>
//#include <cstddef>
//#include <cstdio>
#include <iostream>

enum UAVState {
	UAVSTATE_LANDED=0,
	UAVSTATE_TAKING_OFF,
	UAVSTATE_FLYING,
	UAVSTATE_GOING_HOME,
	UAVSTATE_WAITING_TO_LAND,
	UAVSTATE_LANDING,
	UAVSTATE_COLLISION_AVOIDING,
	UAVSTATE_CHECK_FIRE,
	UAVSTATE_FOLLOW_FIRE,
	UAVSTATE_STAYING

};

class UavGeomStruct
{
	public:
		Position Pos;
		SpeedType GroundSpeed;
		SpeedType VerticalSpeed;
		RotationMatrix RotMat;
//		Rotation Rot;
		Rotation2DType Heading;
		Rotation2DType Yaw;
		Rotation2DType Roll;
		Rotation2DType Pitch;
		bool RotationUpToDate;

		UavGeomStruct(): Heading(0), Yaw(0), Roll(0), Pitch(0), RotationUpToDate(false) {}

		void Init()
		{
			Pos.setZero();
//			Speed.setZero();
			GroundSpeed = 0;
			VerticalSpeed = 0;
//			Rot.coeffs().setZero();
			RotMat.setZero();
			Heading.angle() = 0;
			Yaw.angle() = 0;
			Roll.angle() = 0;
			Pitch.angle() = 0;
			RotationUpToDate = false;
		}

		friend std::ostream& operator<<(std::ostream& os, const UavGeomStruct& struc)
		{
			//os << "Pos=" << struc.Pos.transpose() << " Speed=" << struc.Speed.transpose();
			os << "Pos=" << struc.Pos.transpose() << " Speed=" << struc.GroundSpeed << " VerticalSpeed=" << struc.VerticalSpeed;
			//os << " Rotation=" << struc.Rot;
			os << " Heading=" << struc.Heading.angle() << " Yaw=" << struc.Yaw.angle() << " Roll=" << struc.Roll.angle() << " Pitch=" << struc.Pitch.angle();
			os << " RotationUpToDate=" << struc.RotationUpToDate;
			os << " RotMat=" << struc.RotMat;
			return os;
		}
};

class APStatusStruct
{
	public:
		uint8_t		FlyState;		// See EAutoPilotFlyState
		uint8_t		GPSState;		// 0 is none, 255 is best
		uint8_t		ServoState;		// Bitmask, see defines AP_PROT_STATE_AP_*
		uint8_t		AutoPilotState;	// Bitmask, see defines AP_PROT_STATE_SERVO_*
		uint8_t		SensorState;	// Bitmask, see defines AP_PROT_STATE_SENSOR_*

		APStatusStruct(): FlyState(255), GPSState(255), ServoState(255), AutoPilotState(255), SensorState(255) {}

//		void Init()
//		{
//			FlyState = 255;
//			GPSState = 255;
//			ServoState = 255;
//			AutoPilotState = 255;
//			SensorState = 255;
//		}

		friend std::ostream& operator<<(std::ostream& os, const APStatusStruct& struc)
		{
			os << "FlyState=" << struc.FlyState << " GPSState=" << struc.GPSState << " ServoState=" << struc.ServoState
					<< "AutoPilotState=" << struc.AutoPilotState << " SensorState=" << struc.SensorState;
			return os;
		}
};

class UavStruct
{
	public:
		int				UavId;
		UavGeomStruct	Geom;
		UAVState		State;
		WayPoint		WpNext[UAVSTRUCT_NEXTWP_NUM];
		float			BatteryTimeLeft;
		APStatusStruct	APStatus;

		friend std::ostream& operator<<(std::ostream& os, const UavStruct& struc)
		{
			os << "ID=" << struc.UavId << " " << struc.Geom << " State=" << struc.State;
			return os;
		}

		void FromRadioMsg(RadioMsgRelayPos& msg)
		{
			UavId = msg.UavId - 1;
			// X and Y are 0 to 2047, convert to 0 to 5000
			Geom.Pos.x() = msg.X * 5000/2047;
			Geom.Pos.y() = msg.Y * 5000/2047;
			// Z is 0 to 127, convert to 0 to 300
			Geom.Pos.z() = msg.Z * 300/127;
			// Heading is 0 to 63, convert to 0 to 2*pi
			Geom.Heading.angle() = msg.Heading * 2*M_PI/63;
			// GroundSpeed is 0 to 15, convert to 0 to 33
			Geom.GroundSpeed = msg.GroundSpeed * 33/15;
			Geom.VerticalSpeed = 0; // Assumption
			State = (UAVState)msg.State;
			// Roll is -15 to 15, convert to -0.5*pi to 0.5pi
			Geom.Roll.angle() = msg.Roll * M_PI/2/15;
			Geom.Pitch.angle() = 0; // Assumption
			Geom.RotationUpToDate = false;

			Position from(Geom.Pos);
			for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			{
				WpNext[i].from = from;
				// X and Y are -63 to 63, convert to -100 to 100
				WpNext[i].to.x() = from.x() + msg.DX[i] * 100/63;
				WpNext[i].to.y() = from.y() + msg.DY[i] * 100/63;
				//WpNext.to.z() = from.z() + msg.DZ[i];
				WpNext[i].to.z() = from.z(); // Assumption
				WpNext[i].wpMode = WP_LINE;
				WpNext[i].ETA = 0; // Unknown
				WpNext[i].VerticalSpeed = 0; // Assumption
				//WpNext[i].Radius = 0; // Only lines
				//WpNext[i].AngleStart = 0; // Only lines
				//WpNext[i].AngleArc = 0; // Only lines
				from = WpNext[i].to;
			}
			// BatteryLeft is 0 to 255, convert to 0 to 45*60
			BatteryTimeLeft = msg.BatteryLeft *45*60/255;
			// TODO: retrieve APStatus from radio msg
			//APStatus.AutoPilotState = msg.Status;
		}

		void ToRadioMsg(RadioMsgRelayPos& msg)
		{
			// Clamp values to be sure!
			msg.UavId = UavId + 1;
			// X and Y are 0 to 5000, convert to 0 to 2047
			msg.X = std::min(std::max((int)(Geom.Pos.x() * 2047/5000),0),2047);
			msg.Y = std::min(std::max((int)(Geom.Pos.y() * 2047/5000),0),2047);
			// Z is 0 to 300, convert to 0 to 127
			msg.Z = std::min(std::max((int)(Geom.Pos.z() * 127/300),0),127);
			// Heading is 0 to 2*pi, convert to 0 to 63
			msg.Heading = std::min(std::max((int)(Geom.Heading.angle() * 63/2/M_PI),0),63);
			// GroundSpeed is 0 to 33, convert to 0 to 15
			msg.GroundSpeed = std::min(std::max((int)(Geom.GroundSpeed * 15/33),0),15);
			msg.State = State;
			// Roll is -0.5*pi to 0.5pi, convert to -15 to 15
			msg.Roll = std::min(std::max((int)(Geom.Roll.angle() * 15*2/M_PI),-15),15);

			// Assuming WpNext are lines...
			Position from(Geom.Pos);
			Position to;
			for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			{
				switch (WpNext[i].wpMode)
				{
					case WP_CIRCLE:
					{
						// Fill up all next DX and DY as we stay on the circle
						// TODO: we might not be on the circle yet!
						std::vector<Position> posList;
						float distLeft;
						// Assumes we are on the circle
						WpNext[i].GetPath(posList, distLeft, 22*3, &to); // TODO: magic number
						for (int j=0; i<UAVSTRUCT_NEXTWP_NUM; ++i, ++j)
						{
							// X and Y are -100 to 100, convert to -63 to 63
							msg.DX[i] = std::min(std::max((int)(posList[j].x() * 63/100),-63),63);
							msg.DY[i] = std::min(std::max((int)(posList[j].y() * 63/100),-63),63);
						}
						break;
					}
					case WP_LINE:
					case WP_ARC:
						WpNext[i].GetEndPos(to);
						// X and Y are -100 to 100, convert to -63 to 63
						msg.DX[i] = std::min(std::max((int)((to.x() - from.x()) * 63/100),-63),63);
						msg.DY[i] = std::min(std::max((int)((to.y() - from.y()) * 63/100),-63),63);
						break;
					default:
						msg.DX[i] = 0;
						msg.DY[i] = 0;
						break;
				}
				from = to;
			}
			// BatteryLeft is 0 to 45*60, convert to 0 to 255
			msg.BatteryLeft = std::min(std::max((int)(BatteryTimeLeft * 255/45/60),0),255);
			msg.Status = 0; // TODO: fill this from APStatus
		}
};

class MapUavStruct
{
	public:
		MapUavStruct(): LastRadioMsgsIndex(0) {};
		UavStruct data;
		long LastRadioReceiveTime;
		long LastRadioSentTime;
		bool Connected;

		RadioMsgRelay LastRadioMsgs[MAPUAV_RADIOMSG_HIST]; // Little history so we can ignore radio msgs we already received
		int LastRadioMsgsIndex; // Index of latest message in array LastRadioMsgs

		friend std::ostream& operator<<(std::ostream& os, const MapUavStruct& struc)
		{
			os << struc.data << " LastRadioReceiveTime=" << struc.LastRadioReceiveTime << " LastRadioSentTime="
					<< struc.LastRadioSentTime << " Connected=" << struc.Connected << " LastRadioMsgs=[";
			for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
				os << " [" << struc.LastRadioMsgs[i] << "]";
			os << "] LastRadioMsgsIndex=" << struc.LastRadioMsgsIndex;
			return os;
		}
};



class MapSelfNeighboursStruct
{
	public:

		// Faster clear(), slower isConnected()
		int ConnectedNeighboursNum;
		int ConnectedNeighbours[UAVS_NUM];

		MapSelfNeighboursStruct(): ConnectedNeighboursNum(0) {}
		void clear() { ConnectedNeighboursNum = 0; }
		void push_back(int uavId)
		{
			ConnectedNeighbours[ConnectedNeighboursNum++] = uavId;
		}
		bool isConnected(int uavId)
		{
			for (int i=0; i<ConnectedNeighboursNum; ++i)
				if (ConnectedNeighbours[i] == uavId)
					return true;
			return false;
		}

		friend std::ostream& operator<<(std::ostream& os, const MapSelfNeighboursStruct& struc)
		{
			for (int i=0; i<struc.ConnectedNeighboursNum-1; ++i)
				os << struc.ConnectedNeighbours[i] << " ";
			os << struc.ConnectedNeighbours[struc.ConnectedNeighboursNum-1];
			return os;
		}

	/*
		// Slower clear(), faster isConnected()
		bool ConnectedNeighbours[UAVS_NUM];

		void clear()
		{
			for (int i=0; i<UAVS_NUM; ++i)
				ConnectedNeighbours[i] = false;
		}
		inline void push_back(int uavId)
		{
			ConnectedNeighbours[uavId] = true;
		}
		inline bool isConnected(int uavId) { return ConnectedNeighbours[uavId]; }
		friend std::ostream& operator<<(std::ostream& os, const MapSelfNeighboursStruct& struc)
		{
			for (int i=0; i<UAVS_NUM; ++i)
				if (struc.ConnectedNeighbours[i])
					os << i << " ";
			return os;
		}
	*/
};

class LandingStruct
{
	public:
		Position				Pos;	// In local coordinates
		Rotation2DType			Heading;	// In rad
		bool					LeftTurn;

		LandingStruct(): Heading(0) {}

	friend std::ostream& operator<<(std::ostream& os, const LandingStruct& struc)
	{
		os << "Position=[" << struc.Pos.transpose() << "] Heading=" << struc.Heading.angle() << " LeftTurn=" << struc.LeftTurn;
		return os;
	}
};

class GsCmdStruct
{
public:
	int						UavId; // ID of the UAV who the command is for (10 for all UAVs)
	int						MsgId; // ID of this message, so that UAVs know if they relayed this msg yet.

	float					HeightMin;
	float					HeightMax;

	Position				AreaZero;	// In local coordinates
	Position				AreaSize;	// In local coordinates
	Rotation2DType			AreaRotation;	// In rad

	LandingStruct			Landing;

	// See EAutoPilotMode
	// Home: go home and land (waiting for others)
	// Land: land _now_ (don't accept this command is for all UAVs)
	uint8_t					Mode;
	bool					EnablePlanner;

	GsCmdStruct(): AreaRotation(0) {}

	void fromMsg(RadioMsgRelayCmd& msg)
	{
		// Id 0 is invalid id
		UavId = msg.UavId - 1;
		MsgId = msg.MsgId;
		// Height is 0 to 255, translate to 50 to 305
		HeightMin = msg.HeightMin + 50;
		HeightMax = msg.HeightMax + 50;
		// Area is 0 to 1023, translate to 0 to 5000
		AreaZero.x() = msg.AreaMinX * 5000/1023;
		AreaZero.y() = msg.AreaMinY * 5000/1023;
		AreaSize.x() = msg.AreaDX * 5000/1023;
		AreaSize.y() = msg.AreaDY * 5000/1023;
		// Rotation is 0 to 1023, translate to 0 to 0.5*pi
		AreaRotation.angle() = msg.AreaRotation * M_PI/2/1023;
		// Landing pos is 0 to 4095, translate to 0 to 5000
		Landing.Pos.x() = msg.LandX * 5000/4095;
		Landing.Pos.y() = msg.LandY * 5000/4095;
		// Land heading is 0 to 255, translate to 0 to 2*pi
		Landing.Heading.angle() = msg.LandHeading * 2.0*M_PI/255;
		Landing.LeftTurn = msg.LandLeftTurn;
		Mode = msg.Mode;
		EnablePlanner = msg.EnablePlanner;
	}

	void toMsg(RadioMsgRelayCmd& msg)
	{
		// Clamp the values to be sure!
		msg.UavId = UavId + 1;
		msg.MsgId = MsgId;
		msg.HeightMin = std::min(std::max((int)(HeightMin - 50), 0), 255);
		msg.HeightMax = std::min(std::max((int)(HeightMax - 50), 0), 255);
		msg.AreaMinX = std::min(std::max((int)(AreaZero.x() * 1023/5000), 0), 1023);
		msg.AreaMinY = std::min(std::max((int)(AreaZero.y() * 1023/5000), 0), 1023);
		msg.AreaDX = std::min(std::max((int)(AreaSize.x() * 1023/5000), 0), 1023);
		msg.AreaDY = std::min(std::max((int)(AreaSize.y() * 1023/5000), 0), 1023);
		msg.AreaRotation = std::min(std::max((int)(AreaRotation.angle() * 1023*2/M_PI), 0), 1023);
		msg.LandX = std::min(std::max((int)(Landing.Pos.x() * 4095/5000), 0), 4095);
		msg.LandY = std::min(std::max((int)(Landing.Pos.y() * 4095/5000), 0), 4095);
		msg.LandHeading = std::min(std::max((int)(Landing.Heading.angle() * 255/2/M_PI), 0), 255);
		msg.LandLeftTurn = Landing.LeftTurn;
		msg.Mode = Mode;
		msg.EnablePlanner = EnablePlanner;
	}


	friend std::ostream& operator<<(std::ostream& os, const GsCmdStruct& struc)
	{
		os << "UavId=" << +struc.UavId << " MsgId=" << +struc.MsgId << " HeightMin=" << +struc.HeightMin << " HeightMax=" << +struc.HeightMax
				<< " AreaZero=[" << struc.AreaZero.transpose() << "] AreaDX=[" << struc.AreaSize.transpose() << "] AreaRotation=" << struc.AreaRotation.angle()
				<< " Landing=[" << struc.Landing << "] Mode=" << +struc.Mode << " EnablePlanner=" << struc.EnablePlanner;
		return os;
	}
};

class GsCmdHistStruct
{
	public:
		RadioMsgRelayCmd Msg;
		int TimesSent;
};

class MapSelfStruct
{
	public:
		UavStruct 				UavData;
		UAVState				PreviousState;
//		Position				Home;
		//WayPointVectorType WayPoints; // Can't really do this, since you need to init the vector after the shared memory is made
		WayPointsStruct 		WayPoints;
		MapSelfNeighboursStruct NeighBours;
		GsCmdHistStruct			LastGsCmds[MAPSELF_GS_CMDS_HIST]; // Little history, so we know which we already received
		int						LastGsCmdsIndex; // Index of latest message in array LastGsCmds
		GsCmdStruct				GsCmd;
		uint8_t					RequestedAPModeByWP; // Requested by WP, see EAutoPilotMode

		friend std::ostream& operator<<(std::ostream& os, const MapSelfStruct& struc)
		{
			os << struc.UavData << " Previous state=" << struc.PreviousState
					<< " WayPoints=[" << struc.WayPoints << "]" << " NeighBours=[" << struc.NeighBours << "]"
					<< " LastGsCmds=[";
					for (int i=0; i<MAPSELF_GS_CMDS_HIST; ++i)
						os << "[" << struc.LastGsCmds[i].Msg << "] TimesSent=" << struc.LastGsCmds[i].TimesSent << " ";
					os << "]" << " GsCmd=" << struc.GsCmd << " RequestedAPModeByWP=" << struc.RequestedAPModeByWP;

			return os;
		}
};

class SimUavStruct
{
	public:
		MapSelfStruct UavData;
		bool WaitForReplyAutoPilot;
		bool WaitForReplyRadio;
		float TakeOffTime;
		bool RadioWorks;
		bool RadioSim;
		bool AutoPilotSim;

		//SimUavStruct(): WaitForReplyAutoPilot(false) {}

		friend std::ostream& operator<<(std::ostream& os, const SimUavStruct& struc)
		{
			os << struc.UavData << " WaitForReplyAP=" << struc.WaitForReplyAutoPilot << " WaitForReplyRadio=" << struc.WaitForReplyRadio
					<< " TakeOffTime=" << struc.TakeOffTime << " RadioWorks=" << struc.RadioWorks;
			return os;
		}
};

#endif /* UAVSTRUCTS_H_ */
