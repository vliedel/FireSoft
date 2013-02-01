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
			os << "FlyState=" << struc.FlyState << " GPSState=" << struc.GPSState << " ServoState=" << struc.ServoState;
			os << "AutoPilotState=" << struc.AutoPilotState << " SensorState=" << struc.SensorState;
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
		//WayPoint WpFar;

		friend std::ostream& operator<<(std::ostream& os, const UavStruct& struc)
		{
			os << "ID=" << struc.UavId << " " << struc.Geom << " State=" << struc.State;
			return os;
		}

		// TODO: conversions
		void FromRadioMsg(RadioMsgRelay& msg)
		{
			if (msg.MessageType != RADIO_MSG_RELAY_POS)
				return;
			UavId = msg.Pos.UavId - 1;
			State = (UAVState)msg.Pos.State;
			Geom.Pos.x() = msg.Pos.X;
			Geom.Pos.y() = msg.Pos.Y;
			Geom.Pos.z() = msg.Pos.Z;
			Geom.GroundSpeed = msg.Pos.GroundSpeed;
			//Geom.VerticalSpeed = msg.Pos.DZ[0]; // wrong
			Geom.VerticalSpeed = 0; // Assumption

			Geom.Heading.angle() = msg.Pos.Heading;
			Geom.Roll.angle() = msg.Pos.Roll;
			//Geom.Pitch.angle() = msg.Pos.DZ[0]; // Wrong
			Geom.Pitch.angle() = 0; // Assumption
			Geom.RotationUpToDate = false;

			Position from(Geom.Pos);
			for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			{
				WpNext[i].from = from;
				WpNext[i].to.x() = from.x() + msg.Pos.DX[i];
				WpNext[i].to.y() = from.y() + msg.Pos.DY[i];
				//WpNext.to.z() = from.z() + msg.Pos.DZ[i];
				WpNext[i].to.z() = from.z(); // Assumption
				WpNext[i].wpMode = WP_LINE;
				WpNext[i].ETA = 0; // Unknown
				WpNext[i].VerticalSpeed = 0; // Assumption
				//WpNext[i].Radius = 0; // Only lines
				//WpNext[i].AngleStart = 0; // Only lines
				//WpNext[i].AngleArc = 0; // Only lines
				from = WpNext[i].to;
			}
			BatteryTimeLeft = msg.Pos.BatteryLeft;
			// TODO: retrieve APStatus from radio msg
			//APStatus.AutoPilotState = msg.Pos.Status;
		}

		// TODO: conversions
		void ToRadioMsg(RadioMsgRelay& msg)
		{
			msg.MessageType = RADIO_MSG_RELAY_POS;
			msg.Pos.UavId = UavId + 1;
			msg.Pos.X = Geom.Pos.x();
			msg.Pos.Y = Geom.Pos.y();
			msg.Pos.Z = Geom.Pos.z();
			msg.Pos.Heading = Geom.Heading.angle();
			msg.Pos.GroundSpeed = Geom.GroundSpeed;
			msg.Pos.State = State;
			msg.Pos.Roll = Geom.Roll.angle();

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
							msg.Pos.DX[i] = posList[j].x();
							msg.Pos.DY[i] = posList[j].y();
						}
						break;
					}
					case WP_LINE:
					case WP_ARC:
						WpNext[i].GetEndPos(to);
						msg.Pos.DX[i] = to.x() - from.x();
						msg.Pos.DY[i] = to.y() - from.y();
						break;
					default:
						msg.Pos.DX[i] = 0;
						msg.Pos.DY[i] = 0;
						break;
				}
				from = to;
			}
			msg.Pos.BatteryLeft = BatteryTimeLeft;
			msg.Pos.Status = 0; // TODO: fill this from APStatus
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
		int LastRadioMsgsIndex;

//		WayPoint WayPointNext;
//		WayPoint WayPointFar;
//		int WaPointFarETA;

		friend std::ostream& operator<<(std::ostream& os, const MapUavStruct& struc)
		{
			os << struc.data << " LastRadioReceiveTime=" << struc.LastRadioReceiveTime << " LastRadioSentTime=" << struc.LastRadioSentTime << " Connected=" << struc.Connected;
			for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
				os << " [" << struc.LastRadioMsgs[i] << "]";
			os << " LastRadioMsgsIndex=" << struc.LastRadioMsgsIndex;
//			os << " WayPointNext=[" << struc.WayPointNext << "]";
//			os << " WayPointFar=[" << struc.WayPointFar << "] WaPointFarETA=" << struc.WaPointFarETA;
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

class MapSelfStruct
{
	public:
		UavStruct 				UavData;
		UAVState				PreviousState;
//		float 					BatteryTimeLeft;

		Position				Home; // TODO: don't use anymore

//		MapSelfAPStatusStruct	APStatus;
		WayPointsStruct 		WayPoints;
		MapSelfNeighboursStruct NeighBours;
		RadioMsgRelayCmd		LastGsCmds[MAPSELF_GS_CMDS_HIST]; // Last cmd at index MAPSELF_GS_CMDS_HIST-1

		float					HeightMin;
		float					HeightMax;
//		Position				Origin;		// Origin of local coordinate system in mercator coordinates
		Position				AreaZero;	// In local coordinates
		Position				AreaSize;	// In local coordinates
		Rotation2DType			AreaRotation;	// In rad
		LandingStruct			Landing;
		uint8_t					RequestedAPModeByGS; // Requested by GS, see EAutoPilotMode
		uint8_t					RequestedAPModeByWP; // Requested by WP, see EAutoPilotMode
		bool					EnablePlanner;

		MapSelfStruct(): AreaRotation(0) {}


		//WayPointVectorType WayPoints; // Can't really do this, since you need to init the vector after the shared memory is made

		friend std::ostream& operator<<(std::ostream& os, const MapSelfStruct& struc)
		{
			os << struc.UavData << " Previous state=" << struc.PreviousState \
					<< " Home=" << struc.Home.transpose() << " Connected neighbours=[" << struc.NeighBours << "]";
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
			os << struc.UavData << " WaitForReplyAP=" << struc.WaitForReplyAutoPilot << " WaitForReplyRadio=" << struc.WaitForReplyRadio;
			os << " TakeOffTime=" << struc.TakeOffTime << " RadioWorks=" << struc.RadioWorks;
			return os;
		}
};

#endif /* UAVSTRUCTS_H_ */
