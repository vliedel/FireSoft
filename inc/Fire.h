/**
 * @brief 
 * @file Fire.h
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
 * @date          Feb 12, 2013
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef FIRE_H_
#define FIRE_H_

#include "MapFunctions.h"

enum EFireSource
{
	FIRE_SRC_CAM=0,
	FIRE_SRC_TPA,
	FIRE_SRC_CO,
	FIRE_SRC_TPA_FRONT,
	FIRE_SRC_CAM_FRONT,
	FIRE_SRC_NUM
};

class FireStruct: public Gaussian2D
{
public:
	int UavId;
	float Probability[FIRE_SRC_NUM];
	float Height;
	float SigmaX, SigmaY;
	float Rotation;
	FireStruct() {}

	FireStruct(Position& center, float amplitude, float sigmaX, float sigmaY, float rotation):
		Gaussian2D::Gaussian2D(center, amplitude, sigmaX, sigmaY, rotation)
	{
		SigmaX = sigmaX;
		SigmaY = sigmaY;
		Rotation = rotation;
	}

	void Init(float sigmaX, float sigmaY, float rotation)
	{
		Gaussian2D::Init(sigmaX, sigmaY, rotation);
		SigmaX = sigmaX;
		SigmaY = sigmaY;
		Rotation = rotation;
	}

	void FromRadioMsg(RadioMsgRelayFire& msg)
	{
		// Id 0 is invalid id
		UavId = msg.UavId - 1;
		// X and Y are 0 to 2047, convert to 0 to 5000
		Center.x() = msg.X * 5000.0/2047;
		Center.y() = msg.Y * 5000.0/2047;
		Center.z() = 0;
		// sigma X and Y are 0 to 15, convert to 2 to 17
		float sigmaX = msg.VarX * 1.0+2;
		float sigmaY = msg.VarY * 1.0+2;
		// rotation is 0 to 15, convert to 0 to 0.5pi
		float rotation = msg.Rot * M_PI/2/15;
		Init(sigmaX, sigmaY, rotation);
		// Probabilities are 0 to 15, convert to 0 to 1
		Probability[FIRE_SRC_CAM] = msg.PCam / 15.0;
		Probability[FIRE_SRC_TPA] = msg.PTPA / 15.0;
		Probability[FIRE_SRC_CO] = msg.PGas / 15.0;
		// Height is 0 to 31, convert to 0 to 310
		Height = msg.Z * 10.0;

		Amplitude = 1-(1-Probability[FIRE_SRC_CAM])*(1-Probability[FIRE_SRC_TPA])*(1-Probability[FIRE_SRC_CO]);
	}

	void ToRadioMsg(RadioMsgRelayFire& msg)
	{
		// Id 0 is invalid id
		msg.UavId = UavId + 1;
		// X and Y are 0 to 5000, convert to 0 to 2047
		msg.X = std::min(std::max((int)(Center.x() * 2047/5000),0),2047);
		msg.Y = std::min(std::max((int)(Center.y() * 2047/5000),0),2047);
		// sigma X and Y are 2 to 17, convert to 0 to 15
		msg.VarX = std::min(std::max((int)(SigmaX - 2),0),15);
		msg.VarY = std::min(std::max((int)(SigmaY - 2),0),15);
		// rotation is 0 to 15, convert to 0 to 0.5pi
		msg.Rot = std::min(std::max((int)(Rotation * 15*2/M_PI),0),15);
		// Probabilities are 0 to 1, convert to 0 to 15
		msg.PCam = std::min(std::max((int)(Probability[FIRE_SRC_CAM] * 15),0),15);
		msg.PTPA = std::min(std::max((int)(Probability[FIRE_SRC_TPA] * 15),0),15);
		msg.PGas = std::min(std::max((int)(Probability[FIRE_SRC_CO] * 15),0),15);
		// Height is 0 to 310, convert to 0 to 31
		msg.Z = std::min(std::max((int)(Height / 10),0),31);
	}

	friend std::ostream& operator<<(std::ostream& os, const FireStruct& struc)
	{
		os << static_cast<const Gaussian2D&>(struc);
		for (int i=0; i<FIRE_SRC_NUM; ++i)
			os << " p" << i << "=" << struc.Probability[i];
		os << " Height=" << struc.Height;
		return os;
	}
};

class MapFireStruct
{
public:
	FireStruct Fire;
	bool Seen;
	bool Sent;
//	long RadioReceiveTime;
//	long RadioSentTime;

	MapFireStruct() {}

	MapFireStruct(Position& center, float amplitude, float sigmaX, float sigmaY, float rotation):
		Fire(center, amplitude, sigmaX, sigmaY, rotation),
		Seen(false),
		Sent(false)
	{}

	friend std::ostream& operator<<(std::ostream& os, const MapFireStruct& struc)
	{
		os << struc.Fire << " Seen=" << struc.Seen << " Sent=" << struc.Sent;
				//<< " RadioReceiveTime=" << struc.RadioReceiveTime << " RadioSentTime=" << struc.RadioSentTime;
		return os;
	}
};


#endif /* FIRE_H_ */
