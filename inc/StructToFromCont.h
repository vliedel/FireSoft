/**
 * @brief 
 * @file StructToFromCont.h
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
 * @date          Aug 22, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef STRUCTTOFROMCONT_H_
#define STRUCTTOFROMCONT_H_

#define _GLIBCXX_CONCEPT_CHECKS

#include <iterator>
#include "Geometry.h"
#include "UAVStructs.h"
#include "UAVState.h"
#include "Waypoint.h"
#include "RadioStructs.h"
#include "Fire.h"

//#include <iostream>

// TODO: Add checks when loading from container

template<typename InputForwardIter, typename OutputForwardIter, class T>
struct SfC;
template<typename SequenceContainer, class T>
struct StC;


// Template functions, for easy calling
// These will use a class instead, see: http://www.gotw.ca/publications/mill17.htm
// "Why Not Specialize Function Templates"

template<class T, typename InputForwardIter>
InputForwardIter FromCont(T& struc, InputForwardIter first, InputForwardIter last)
{
	return SfC<InputForwardIter, InputForwardIter, T>::FromCont(struc, first, last);
}

template<class T, typename SequenceContainer>
void ToCont(T& struc, SequenceContainer& cont)
{
	__glibcxx_class_requires(cont, _SequenceConcept)
	StC<SequenceContainer, T>::ToCont(struc, cont);
}



// Template class and specializations

template<typename InputForwardIter, typename OutputForwardIter, class T>
struct SfC
{
	static OutputForwardIter FromCont(T& struc, InputForwardIter first, InputForwardIter last)
	{
		throw; //?? assert(false);
	}
};
template<typename SequenceContainer, class T>
struct Stv
{
	static void ToCont(T& struc, SequenceContainer& cont)
	{
		throw; //?? assert(false);
	}
};

/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, Position>
{
	static OutputForwardIter FromCont(Position& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.x = *first++;
		struc.y = *first++;
		struc.z = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, Position>
{
	static void ToCont(Position& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.x);
		cont.push_back(struc.y);
		cont.push_back(struc.z);
	}
};
*/
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, Position>
{
	static OutputForwardIter FromCont(Position& struc, InputForwardIter first, InputForwardIter last)
	{
		struc(0) = *first++;
		struc(1) = *first++;
		struc(2) = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, Position>
{
	static void ToCont(Position& struc, SequenceContainer& cont)
	{
		cont.push_back(struc(0));
		cont.push_back(struc(1));
		cont.push_back(struc(2));
	}
};



template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, SpeedType>
{
	static OutputForwardIter FromCont(SpeedType& struc, InputForwardIter first, InputForwardIter last)
	{
		struc = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, SpeedType>
{
	static void ToCont(SpeedType& struc, SequenceContainer& cont)
	{
		cont.push_back(struc);
	}
};



/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, Rotation>
{
	static OutputForwardIter FromCont(Rotation& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.r[0] = *first++;
		struc.r[1] = *first++;
		struc.r[2] = *first++;
		struc.r[3] = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, Rotation>
{
	static void ToCont(Rotation& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.r[0]);
		cont.push_back(struc.r[1]);
		cont.push_back(struc.r[2]);
		cont.push_back(struc.r[3]);
	}
};
*/
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, Rotation>
{
	static OutputForwardIter FromCont(Rotation& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.w() = *first++;
		struc.x() = *first++;
		struc.y() = *first++;
		struc.z() = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, Rotation>
{
	static void ToCont(Rotation& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.w());
		cont.push_back(struc.x());
		cont.push_back(struc.y());
		cont.push_back(struc.z());
	}
};


/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RotationMatrix>
{
	static OutputForwardIter FromCont(RotationMatrix& struc, InputForwardIter first, InputForwardIter last)
	{
		for (int i=0; i<9; ++i)
			struc.r[i] = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RotationMatrix>
{
	static void ToCont(RotationMatrix& struc, SequenceContainer& cont)
	{
		for (int i=0; i<9; ++i)
			cont.push_back(struc.r[i]);
	}
};
*/
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RotationMatrix>
{
	static OutputForwardIter FromCont(RotationMatrix& struc, InputForwardIter first, InputForwardIter last)
	{
		for (int i=0; i<9; ++i)
			struc(i) = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RotationMatrix>
{
	static void ToCont(RotationMatrix& struc, SequenceContainer& cont)
	{
		for (int i=0; i<9; ++i)
			cont.push_back(struc(i));
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, Rotation2DType>
{
	static OutputForwardIter FromCont(Rotation2DType& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.angle() = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, Rotation2DType>
{
	static void ToCont(Rotation2DType& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.angle());
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, UavGeomStruct>
{
	static OutputForwardIter FromCont(UavGeomStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.Pos, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, SpeedType>::FromCont(struc.GroundSpeed, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, SpeedType>::FromCont(struc.VerticalSpeed, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, RotationMatrix>::FromCont(struc.RotMat, first, last);
		//first = SfC<InputForwardIter, OutputForwardIter, Rotation>::FromCont(struc.Rot, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Rotation2DType>::FromCont(struc.Heading, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Rotation2DType>::FromCont(struc.Yaw, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Rotation2DType>::FromCont(struc.Roll, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Rotation2DType>::FromCont(struc.Pitch, first, last);
		struc.RotationUpToDate = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, UavGeomStruct>
{
	static void ToCont(UavGeomStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, Position>::ToCont(struc.Pos, cont);
		StC<SequenceContainer, SpeedType>::ToCont(struc.GroundSpeed, cont);
		StC<SequenceContainer, SpeedType>::ToCont(struc.VerticalSpeed, cont);
		StC<SequenceContainer, RotationMatrix>::ToCont(struc.RotMat, cont);
		//StC<SequenceContainer, Rotation>::ToCont(struc.Rot, cont);
		StC<SequenceContainer, Rotation2DType>::ToCont(struc.Heading, cont);
		StC<SequenceContainer, Rotation2DType>::ToCont(struc.Yaw, cont);
		StC<SequenceContainer, Rotation2DType>::ToCont(struc.Roll, cont);
		StC<SequenceContainer, Rotation2DType>::ToCont(struc.Pitch, cont);
		cont.push_back(struc.RotationUpToDate);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, WayPoint>
{
	static OutputForwardIter FromCont(WayPoint& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.wpMode = (WayPointMode) *first++;
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.from, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.to, first, last);
		struc.Radius = *first++;
		struc.AngleStart = *first++;
		struc.AngleArc = *first++;
		struc.VerticalSpeed = *first++;
		struc.ETA = *first++;
		struc.ID = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, WayPoint>
{
	static void ToCont(WayPoint& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.wpMode);
		StC<SequenceContainer, Position>::ToCont(struc.from, cont);
		StC<SequenceContainer, Position>::ToCont(struc.to, cont);
		cont.push_back(struc.Radius);
		cont.push_back(struc.AngleStart);
		cont.push_back(struc.AngleArc);
		cont.push_back(struc.VerticalSpeed);
		cont.push_back(struc.ETA);
		cont.push_back(struc.ID);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, APStatusStruct>
{
	static OutputForwardIter FromCont(APStatusStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.FlyState = *first++;
		struc.GPSState = *first++;
		struc.ServoState = *first++;
		struc.AutoPilotState = *first++;
		struc.SensorState = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, APStatusStruct>
{
	static void ToCont(APStatusStruct& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.FlyState);
		cont.push_back(struc.GPSState);
		cont.push_back(struc.ServoState);
		cont.push_back(struc.AutoPilotState);
		cont.push_back(struc.SensorState);
	}
};

/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, UavStruct>
{
	static OutputForwardIter FromCont(UavStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.UavId = *first++;
		first = SfC<InputForwardIter, OutputForwardIter, UavGeomStruct>::FromCont(struc.Geom, first, last);
		struc.State = (UAVState) *first++;
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			first = SfC<InputForwardIter, OutputForwardIter, WayPoint>::FromCont(struc.WpNext[i], first, last);
		struc.BatteryTimeLeft = *first++;
		first = SfC<InputForwardIter, OutputForwardIter, APStatusStruct>::FromCont(struc.APStatus, first, last);
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, UavStruct>
{
	static void ToCont(UavStruct& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.UavId);
		StC<SequenceContainer, UavGeomStruct>::ToCont(struc.Geom, cont);
		cont.push_back(struc.State);
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			StC<SequenceContainer, WayPoint>::ToCont(struc.WpNext[i], cont);
		cont.push_back(struc.BatteryTimeLeft);
		StC<SequenceContainer, APStatusStruct>::ToCont(struc.APStatus, cont);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, MapUavStruct>
{
	static OutputForwardIter FromCont(MapUavStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, UavStruct>::FromCont(struc.data, first, last);
		struc.LastRadioReceiveTime = *first++;
		struc.LastRadioSentTime = *first++;
		struc.Connected = *first++;

		for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
			first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelay>::FromCont(struc.LastRadioMsgs[i], first, last);
		struc.LastRadioMsgsIndex = *first++;

//		first = SfC<InputForwardIter, OutputForwardIter, WayPoint>::FromCont(struc.WayPointNext, first, last);
//		first = SfC<InputForwardIter, OutputForwardIter, WayPoint>::FromCont(struc.WayPointFar, first, last);
//		struc.WaPointFarETA = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, MapUavStruct>
{
	static void ToCont(MapUavStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, UavStruct>::ToCont(struc.data, cont);
		cont.push_back(struc.LastRadioReceiveTime);
		cont.push_back(struc.LastRadioSentTime);
		cont.push_back(struc.Connected);
		for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
			StC<SequenceContainer, RadioMsgRelay>::ToCont(struc.LastRadioMsgs[i], cont);
		cont.push_back(struc.LastRadioMsgsIndex);
//		StC<SequenceContainer, WayPoint>::ToCont(struc.WayPointNext, cont);
//		StC<SequenceContainer, WayPoint>::ToCont(struc.WayPointFar, cont);
//		cont.push_back(struc.WaPointFarETA);
	}
};
*/

template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, WayPointsStruct>
{
	static OutputForwardIter FromCont(WayPointsStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.WayPointsNum = *first++;
		//std::cout << "WPnum=" << struc.WayPointsNum << std::endl;
		for (int i=0; i<struc.WayPointsNum; ++i)
			first = SfC<InputForwardIter, OutputForwardIter, WayPoint>::FromCont(struc.WayPoints[i], first, last);

		//struc.WayPoints.clear();
		//WayPoint wp;
		//int size = *first++;
		//for (int i=0; i<size; ++i)
		//{
		//	first = SfC<InputForwardIter, OutputForwardIter, WayPoint>::FromCont(wp, first, last);
		//	struc.WayPoints.push_back(wp);
		//}
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, WayPointsStruct>
{
	static void ToCont(WayPointsStruct& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.WayPointsNum);
		for (int i=0; i<struc.WayPointsNum; ++i)
			StC<SequenceContainer, WayPoint>::ToCont(struc.WayPoints[i], cont);
//		cont.push_back(struc.WayPoints.size());
//		WayPointVectorType::iterator iter;
//		for (iter=struc.WayPoints.begin(); iter!=struc.WayPoints.end(); ++iter)
//			StC<SequenceContainer, WayPoint>::ToCont(*iter, cont);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, MapSelfNeighboursStruct>
{
	static OutputForwardIter FromCont(MapSelfNeighboursStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.ConnectedNeighboursNum = *first++;
		//std::cout << "NeighBoursNum=" << struc.ConnectedNeighboursNum << std::endl;
		for (int i=0; i<struc.ConnectedNeighboursNum; ++i)
			struc.ConnectedNeighbours[i] = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, MapSelfNeighboursStruct>
{
	static void ToCont(MapSelfNeighboursStruct& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.ConnectedNeighboursNum);
		for (int i=0; i<struc.ConnectedNeighboursNum; ++i)
			cont.push_back(struc.ConnectedNeighbours[i]);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, LandingStruct>
{
	static OutputForwardIter FromCont(LandingStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.Pos, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, Rotation2DType>::FromCont(struc.Heading, first, last);
		struc.LeftTurn = *first++;
		struc.Length = *first++;
		struc.Radius = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, LandingStruct>
{
	static void ToCont(LandingStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, Position>::ToCont(struc.Pos, cont);
		StC<SequenceContainer, Rotation2DType>::ToCont(struc.Heading, cont);
		cont.push_back(struc.LeftTurn);
		cont.push_back(struc.Length);
		cont.push_back(struc.Radius);
	}
};



template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, FireStruct>
{
	static OutputForwardIter FromCont(FireStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.Center, first, last);
		struc.Amplitude = *first++;
		struc.A = *first++;
		struc.B = *first++;
		struc.C = *first++;
		struc.UavId = *first++;
		for (int i=0; i<FIRE_SRC_NUM; ++i)
			struc.Probability[i] = *first++;
		struc.Height = *first++;
		struc.SigmaX = *first++;
		struc.SigmaY = *first++;
		struc.Rotation = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, FireStruct>
{
	static void ToCont(FireStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, Position>::ToCont(struc.Center, cont);
		cont.push_back(struc.Amplitude);
		cont.push_back(struc.A);
		cont.push_back(struc.B);
		cont.push_back(struc.C);
		cont.push_back(struc.UavId);
		for (int i=0; i<FIRE_SRC_NUM; ++i)
			cont.push_back(struc.Probability[i]);
		cont.push_back(struc.Height);
		cont.push_back(struc.SigmaX);
		cont.push_back(struc.SigmaY);
		cont.push_back(struc.Rotation);
	}
};


/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, MapSelfStruct>
{
	static OutputForwardIter FromCont(MapSelfStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, UavStruct>::FromCont(struc.UavData, first, last);
		struc.PreviousState = (UAVState) *first++;
		struc.BatteryTimeLeft = *first++;
		first = SfC<InputForwardIter, OutputForwardIter, Position>::FromCont(struc.Home, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, MapSelfAPStatusStruct>::FromCont(struc.APStatus, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, MapSelfNeighboursStruct>::FromCont(struc.NeighBours, first, last);
		first = SfC<InputForwardIter, OutputForwardIter, WayPointsStruct>::FromCont(struc.WayPoints, first, last);
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, MapSelfStruct>
{
	static void ToCont(MapSelfStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, UavStruct>::ToCont(struc.UavData, cont);
		cont.push_back(struc.PreviousState);
		cont.push_back(struc.BatteryTimeLeft);
		StC<SequenceContainer, Position>::ToCont(struc.Home, cont);
		StC<SequenceContainer, MapSelfAPStatusStruct>::ToCont(struc.APStatus, cont);
		StC<SequenceContainer, MapSelfNeighboursStruct>::ToCont(struc.NeighBours, cont);
		StC<SequenceContainer, WayPointsStruct>::ToCont(struc.WayPoints, cont);
	}
};
*/

/*
template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, SimUavStruct>
{
	static OutputForwardIter FromCont(SimUavStruct& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, MapSelfStruct>::FromCont(struc.UavData, first, last);
		struc.WaitForReplyAutoPilot = *first++;
		struc.WaitForReplyRadio = *first++;
		struc.TakeOffTime = *first++;
		struc.RadioWorks = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, SimUavStruct>
{
	static void ToCont(SimUavStruct& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, MapSelfStruct>::ToCont(struc.UavData, cont);
		cont.push_back(struc.WaitForReplyAutoPilot);
		cont.push_back(struc.WaitForReplyRadio);
		cont.push_back(struc.TakeOffTime);
		cont.push_back(struc.RadioWorks);
	}
};
*/

//template<typename InputForwardIter, typename OutputForwardIter>
//struct SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelayPos>
//{
//	static OutputForwardIter FromCont(SimRadioMsgRelayPos& struc, InputForwardIter first, InputForwardIter last)
//	{
//		struc.x = *first++;
//		struc.y = *first++;
//		struc.z = *first++;
//		struc.heading = *first++;
//		struc.speed = *first++;
//		struc.newZ = *first++;
//		struc.state = *first++;
//		struc.roll = *first++;
//		return first;
//	}
//};
//template<typename SequenceContainer>
//struct StC<SequenceContainer, SimRadioMsgRelayPos>
//{
//	static void ToCont(SimRadioMsgRelayPos& struc, SequenceContainer& cont)
//	{
//		cont.push_back(struc.x);
//		cont.push_back(struc.y);
//		cont.push_back(struc.z);
//		cont.push_back(struc.heading);
//		cont.push_back(struc.speed);
//		cont.push_back(struc.newZ);
//		cont.push_back(struc.state);
//		cont.push_back(struc.roll);
//	}
//};
//
//
//template<typename InputForwardIter, typename OutputForwardIter>
//struct SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelay>
//{
//	static OutputForwardIter FromCont(SimRadioMsgRelay& struc, InputForwardIter first, InputForwardIter last)
//	{
//		struc.MessageType = *first++;
//		struc.UavId = *first++;
//		if (struc.MessageType == RADIO_MSG_RELAY_POS)
//			first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelayPos>::FromCont(struc.pos, first, last);
//		return first;
//	}
//};
//template<typename SequenceContainer>
//struct StC<SequenceContainer, SimRadioMsgRelay>
//{
//	static void ToCont(SimRadioMsgRelay& struc, SequenceContainer& cont)
//	{
//		cont.push_back(struc.MessageType);
//		cont.push_back(struc.UavId);
//		if (struc.MessageType == RADIO_MSG_RELAY_POS)
//			StC<SequenceContainer, SimRadioMsgRelayPos>::ToCont(struc.pos, cont);
//	}
//};
//
//
//template<typename InputForwardIter, typename OutputForwardIter>
//struct SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelays>
//{
//	static OutputForwardIter FromCont(SimRadioMsgRelays& struc, InputForwardIter first, InputForwardIter last)
//	{
//		first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelay>::FromCont(struc.data[0], first, last);
//		first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelay>::FromCont(struc.data[1], first, last);
//		first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelay>::FromCont(struc.data[2], first, last);
//		first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelay>::FromCont(struc.data[3], first, last);
//		return first;
//	}
//};
//template<typename SequenceContainer>
//struct StC<SequenceContainer, SimRadioMsgRelays>
//{
//	static void ToCont(SimRadioMsgRelays& struc, SequenceContainer& cont)
//	{
//		StC<SequenceContainer, SimRadioMsgRelay>::ToCont(struc.data[0], cont);
//		StC<SequenceContainer, SimRadioMsgRelay>::ToCont(struc.data[1], cont);
//		StC<SequenceContainer, SimRadioMsgRelay>::ToCont(struc.data[2], cont);
//		StC<SequenceContainer, SimRadioMsgRelay>::ToCont(struc.data[3], cont);
//	}
//};
//
//
//template<typename InputForwardIter, typename OutputForwardIter>
//struct SfC<InputForwardIter, OutputForwardIter, SimRadioMsg>
//{
//	static OutputForwardIter FromCont(SimRadioMsg& struc, InputForwardIter first, InputForwardIter last)
//	{
//		struc.MessageType = *first++;
//		first = SfC<InputForwardIter, OutputForwardIter, SimRadioMsgRelays>::FromCont(struc.data, first, last);
//		return first;
//	}
//};
//template<typename SequenceContainer>
//struct StC<SequenceContainer, SimRadioMsg>
//{
//	static void ToCont(SimRadioMsg& struc, SequenceContainer& cont)
//	{
//		cont.push_back(struc.MessageType);
//		StC<SequenceContainer, SimRadioMsgRelays>::ToCont(struc.data, cont);
//	}
//};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayPos>
{
	static OutputForwardIter FromCont(RadioMsgRelayPos& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.UavId = *first++;

		struc.X = *first++;
		struc.Y = *first++;
		struc.Z = *first++;
		struc.Heading = *first++;
		struc.GroundSpeed = *first++;
		struc.State = *first++;
		struc.Roll = *first++;
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			struc.DX[i] = *first++;
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			struc.DY[i] = *first++;
		struc.BatteryLeft = *first++;
		struc.Status = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelayPos>
{
	static void ToCont(RadioMsgRelayPos& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.UavId);

		cont.push_back(struc.X);
		cont.push_back(struc.Y);
		cont.push_back(struc.Z);
		cont.push_back(struc.Heading);
		cont.push_back(struc.GroundSpeed);
		cont.push_back(struc.State);
		cont.push_back(struc.Roll);
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			cont.push_back(struc.DX[i]);
		for (int i=0; i<UAVSTRUCT_NEXTWP_NUM; ++i)
			cont.push_back(struc.DY[i]);
		cont.push_back(struc.BatteryLeft);
		cont.push_back(struc.Status);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayCmd>
{
	static OutputForwardIter FromCont(RadioMsgRelayCmd& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.UavId = *first++;
		struc.MsgId = *first++;

		struc.HeightMin = *first++;
		struc.HeightMax = *first++;

		struc.AreaMinX = *first++;
		struc.AreaMinY = *first++;
		struc.AreaDX = *first++;
		struc.AreaDY = *first++;
		struc.AreaRotation = *first++;

		struc.LandX = *first++;
		struc.LandY = *first++;
		struc.LandHeading = *first++;
		struc.LandLeftTurn = *first++;

		struc.Mode = *first++;
		struc.EnablePlanner = *first++;

		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelayCmd>
{
	static void ToCont(RadioMsgRelayCmd& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.UavId);
		cont.push_back(struc.MsgId);

		cont.push_back(struc.HeightMin);
		cont.push_back(struc.HeightMax);

		cont.push_back(struc.AreaMinX);
		cont.push_back(struc.AreaMinY);
		cont.push_back(struc.AreaDX);
		cont.push_back(struc.AreaDY);
		cont.push_back(struc.AreaRotation);

		cont.push_back(struc.LandX);
		cont.push_back(struc.LandY);
		cont.push_back(struc.LandHeading);
		cont.push_back(struc.LandLeftTurn);

		cont.push_back(struc.Mode);
		cont.push_back(struc.EnablePlanner);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayFire>
{
	static OutputForwardIter FromCont(RadioMsgRelayFire& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.UavId = *first++;

		struc.X = *first++;
		struc.Y = *first++;
		struc.VarX = *first++;
		struc.VarY = *first++;
		struc.Rot = *first++;
		struc.PCam = *first++;
		struc.PTPA = *first++;
		struc.PGas = *first++;
		struc.Z = *first++;
		struc.Forwards = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelayFire>
{
	static void ToCont(RadioMsgRelayFire& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.UavId);
		cont.push_back(struc.X);
		cont.push_back(struc.Y);
		cont.push_back(struc.VarX);
		cont.push_back(struc.VarY);
		cont.push_back(struc.Rot);
		cont.push_back(struc.PCam);
		cont.push_back(struc.PTPA);
		cont.push_back(struc.PGas);
		cont.push_back(struc.Z);
		cont.push_back(struc.Forwards);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayFires>
{
	static OutputForwardIter FromCont(RadioMsgRelayFires& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayFire>::FromCont(struc.Fire[0], first, last);
		first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayFire>::FromCont(struc.Fire[1], first, last);
		//struc.Unused = *first++;
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelayFires>
{
	static void ToCont(RadioMsgRelayFires& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, RadioMsgRelayFire>::ToCont(struc.Fire[0], cont);
		StC<SequenceContainer, RadioMsgRelayFire>::ToCont(struc.Fire[1], cont);
		//cont.push_back(struc.Unused);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelay>
{
	static OutputForwardIter FromCont(RadioMsgRelay& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.MessageType = *first++;
		switch (struc.MessageType)
		{
			case RADIO_MSG_RELAY_POS:
				first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayPos>::FromCont(struc.Pos, first, last);
				break;
			case RADIO_MSG_RELAY_FIRE:
				first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayFires>::FromCont(struc.Fires, first, last);
				break;
			case RADIO_MSG_RELAY_CMD:
				first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelayCmd>::FromCont(struc.Cmd, first, last);
				break;
		}
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelay>
{
	static void ToCont(RadioMsgRelay& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.MessageType);
		switch (struc.MessageType)
		{
			case RADIO_MSG_RELAY_POS:
				StC<SequenceContainer, RadioMsgRelayPos>::ToCont(struc.Pos, cont);
				break;
			case RADIO_MSG_RELAY_FIRE:
				StC<SequenceContainer, RadioMsgRelayFires>::ToCont(struc.Fires, cont);
				break;
			case RADIO_MSG_RELAY_CMD:
				StC<SequenceContainer, RadioMsgRelayCmd>::ToCont(struc.Cmd, cont);
				break;
		}
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsgRelays>
{
	static OutputForwardIter FromCont(RadioMsgRelays& struc, InputForwardIter first, InputForwardIter last)
	{
		first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelay>::FromCont(struc.Data[0], first, last);
		first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelay>::FromCont(struc.Data[1], first, last);
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsgRelays>
{
	static void ToCont(RadioMsgRelays& struc, SequenceContainer& cont)
	{
		StC<SequenceContainer, RadioMsgRelay>::ToCont(struc.Data[0], cont);
		StC<SequenceContainer, RadioMsgRelay>::ToCont(struc.Data[1], cont);
	}
};


template<typename InputForwardIter, typename OutputForwardIter>
struct SfC<InputForwardIter, OutputForwardIter, RadioMsg>
{
	static OutputForwardIter FromCont(RadioMsg& struc, InputForwardIter first, InputForwardIter last)
	{
		struc.MessageType = *first++;
		first = SfC<InputForwardIter, OutputForwardIter, RadioMsgRelays>::FromCont(struc.Data, first, last);
		return first;
	}
};
template<typename SequenceContainer>
struct StC<SequenceContainer, RadioMsg>
{
	static void ToCont(RadioMsg& struc, SequenceContainer& cont)
	{
		cont.push_back(struc.MessageType);
		StC<SequenceContainer, RadioMsgRelays>::ToCont(struc.Data, cont);
	}
};

#endif /* STRUCTTOFROMCONT_H_ */
