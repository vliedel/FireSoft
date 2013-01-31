/**
 * @brief Radio message structures
 * @file RadioStructs.h
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
 * @date          Apr 24, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef RADIOSTRUCTS_H_
#define RADIOSTRUCTS_H_

#include <inttypes.h>

#define RADIO_NUM_RELAY_PER_MSG 2 // This number is still hardcoded at some places (this file and structsToFromcont.h)

enum ERadioRoundState {
	RADIO_STATE_ROUND_START=0,
	RADIO_STATE_ROUND_SENDRECEIVE,
	RADIO_STATE_ROUND_END,
	RADIO_STATE_ROUND_IDLE
};

// 4 bits --> max 16 types
enum ERadioMsgType {
	RADIO_MSG_2POS=0,
	RADIO_MSG_POS_2FIRE,
	RADIO_MSG_4FIRE,
	RADIO_MSG_POS_CMD
};

enum ERadioMsgRelayType {
	RADIO_MSG_RELAY_POS=0,
	RADIO_MSG_RELAY_FIRE,
	RADIO_MSG_RELAY_CMD
};

#pragma pack(1)


struct RadioMsgRelayPos
{
	uint8_t UavId; // ID of the UAV where the msg originates from

	int X;
	int Y;
	uint8_t Z;
	uint8_t Heading;
	uint8_t GroundSpeed;

	uint8_t State; // Can be 3 so far
	uint8_t Roll; // why? --> scanning area

	int WpNextDX;
	int WpNextDY;
	uint8_t WpNextZ;
	uint8_t WpNextMode; // Line/circle/arc/?
	uint8_t WpNextRadius;

	int WpFarX;
	int WpFarY;
	uint8_t WpFarZ;
	uint8_t WpFarETA;

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelayPos& struc)
	{
		os << "UavId=" << +struc.UavId << " x=" << struc.X << " y=" << struc.Y << " z=" << +struc.Z << " heading=" << +struc.Heading;
		os << " speed=" << +struc.GroundSpeed << " state=" << +struc.State << " roll=" << +struc.Roll;
		os << " WpNextDX=" << struc.WpNextDX << " WpNextDY=" << struc.WpNextDY << " WpNextZ=" << +struc.WpNextZ;
		os << " WpNextMode=" << +struc.WpNextMode << " WpNextRadius=" << +struc.WpNextRadius;
		os << " WpFarX=" << struc.WpFarX << " WpFarY=" << struc.WpFarY << " WpFarZ=" << +struc.WpFarZ << " WpFarETA=" << +struc.WpFarETA;
		return os;
	}
};
// 110 bits
#define RADIO_STRUCT_POS(n) \
	unsigned _##n##_UavId : 4; \
	unsigned _##n##_X : 10; \
	unsigned _##n##_Y : 10; \
	unsigned _##n##_Z : 5; \
	unsigned _##n##_Heading : 6; \
	unsigned _##n##_GroundSpeed : 4; \
	unsigned _##n##_State : 4; \
	unsigned _##n##_Roll : 4; \
	signed _##n##_WpNextDX : 8; \
	signed _##n##_WpNextDY : 8; \
	unsigned _##n##_WpNextZ : 5; \
	unsigned _##n##_WpNextMode : 2; \
	unsigned _##n##_WpNextRadius : 6; \
	signed _##n##_WpFarX : 10; \
	signed _##n##_WpFarY : 10; \
	unsigned _##n##_WpFarZ : 5; \
	unsigned _##n##_WpFarETA : 8; \
	unsigned _##n##_Unused : 1;

#define RADIO_STRUCT_POS_GET(struc, n, var) struc._##n##_##var

#define RADIO_STRUCT_POS_UNPACK(unpacked, packed, n) \
	unpacked.UavId = 		packed._##n##_UavId; \
	unpacked.X = 			packed._##n##_X; \
	unpacked.Y = 			packed._##n##_Y; \
	unpacked.Z = 			packed._##n##_Z; \
	unpacked.Heading = 		packed._##n##_Heading; \
	unpacked.GroundSpeed = 	packed._##n##_GroundSpeed; \
	unpacked.State = 		packed._##n##_State; \
	unpacked.Roll = 		packed._##n##_Roll; \
	unpacked.WpNextDX = 	packed._##n##_WpNextDX; \
	unpacked.WpNextDY = 	packed._##n##_WpNextDY; \
	unpacked.WpNextZ = 		packed._##n##_WpNextZ; \
	unpacked.WpNextMode = 	packed._##n##_WpNextMode; \
	unpacked.WpNextRadius = packed._##n##_WpNextRadius; \
	unpacked.WpFarX = 		packed._##n##_WpFarX; \
	unpacked.WpFarY = 		packed._##n##_WpFarY; \
	unpacked.WpFarZ = 		packed._##n##_WpFarZ; \
	unpacked.WpFarETA = 	packed._##n##_WpFarETA;

#define RADIO_STRUCT_POS_PACK(unpacked, packed, n) \
	packed._##n##_UavId = 			unpacked.UavId; \
	packed._##n##_X = 				unpacked.X; \
	packed._##n##_Y = 				unpacked.Y; \
	packed._##n##_Z = 				unpacked.Z; \
	packed._##n##_Heading = 		unpacked.Heading; \
	packed._##n##_GroundSpeed = 	unpacked.GroundSpeed; \
	packed._##n##_State = 			unpacked.State; \
	packed._##n##_Roll = 			unpacked.Roll; \
	packed._##n##_WpNextDX = 		unpacked.WpNextDX; \
	packed._##n##_WpNextDY = 		unpacked.WpNextDY; \
	packed._##n##_WpNextZ = 		unpacked.WpNextZ; \
	packed._##n##_WpNextMode = 		unpacked.WpNextMode; \
	packed._##n##_WpNextRadius = 	unpacked.WpNextRadius; \
	packed._##n##_WpFarX = 			unpacked.WpFarX; \
	packed._##n##_WpFarY = 			unpacked.WpFarY; \
	packed._##n##_WpFarZ = 			unpacked.WpFarZ; \
	packed._##n##_WpFarETA = 		unpacked.WpFarETA;



struct RadioMsgRelayFire
{
	uint8_t UavId; // ID of the UAV where the msg originates from

	int X;
	int Y;
	uint8_t VarX; // width of guassian
	uint8_t VarY; // width of guassian
	uint8_t Rot; // rotation of guassian
	uint8_t PCam;
	uint8_t PTPA;
	uint8_t PGas;
	uint8_t Z; // Height of the UAV when it saw this fire
	bool Forwards;

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelayFire& struc)
	{
		os << "UavId=" << +struc.UavId << " x=" << struc.X << " y=" << struc.Y << " VarX=" << +struc.VarX << " VarY=" << +struc.VarY;
		os << " Rot=" << +struc.Rot << " PCam=" << +struc.PCam << " PTPA=" << +struc.PTPA << " PGas=" << +struc.PGas;
		os << " Z=" << +struc.Z << " Forwards=" << +struc.Forwards;
		return os;
	}
};
// 55 bits
#define RADIO_STRUCT_FIRE(n) \
	unsigned _##n##_UavId : 4; \
	signed _##n##_X : 10; \
	signed _##n##_Y : 10; \
	unsigned _##n##_VarX : 4; \
	unsigned _##n##_VarY : 4; \
	unsigned _##n##_Rot : 4; \
	unsigned _##n##_PCam : 4; \
	unsigned _##n##_PTPA : 4; \
	unsigned _##n##_PGas : 4; \
	unsigned _##n##_Z : 5; \
	unsigned _##n##_Forwards : 1; \
	unsigned _##n##_Unused : 1;

#define RADIO_STRUCT_FIRE_GET(struc, n, var) struc._##n##_##var

#define RADIO_STRUCT_FIRE_UNPACK(unpacked, packed, n) \
	unpacked.UavId = 	packed._##n##_UavId; \
	unpacked.X = 		packed._##n##_X; \
	unpacked.Y = 		packed._##n##_Y; \
	unpacked.VarX = 	packed._##n##_VarX; \
	unpacked.VarY = 	packed._##n##_VarY; \
	unpacked.Rot = 		packed._##n##_Rot; \
	unpacked.PCam = 	packed._##n##_PCam; \
	unpacked.PTPA = 	packed._##n##_PTPA; \
	unpacked.PGas = 	packed._##n##_PGas; \
	unpacked.Z = 		packed._##n##_Z; \
	unpacked.Forwards = packed._##n##_Forwards;

#define RADIO_STRUCT_FIRE_PACK(unpacked, packed, n) \
	packed._##n##_UavId = 		unpacked.UavId; \
	packed._##n##_X = 			unpacked.X; \
	packed._##n##_Y = 			unpacked.Y; \
	packed._##n##_VarX = 		unpacked.VarX; \
	packed._##n##_VarY =	 	unpacked.VarY; \
	packed._##n##_Rot = 		unpacked.Rot; \
	packed._##n##_PCam = 		unpacked.PCam; \
	packed._##n##_PTPA = 		unpacked.PTPA; \
	packed._##n##_PGas = 		unpacked.PGas; \
	packed._##n##_Z = 			unpacked.Z; \
	packed._##n##_Forwards = 	unpacked.Forwards;

struct RadioMsgRelayFires
{
	RadioMsgRelayFire Fire[2];

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelayFires& struc)
	{
		os << "Fire0=[" << struc.Fire[0] << "] Fire1=[" << struc.Fire[1] << "]";
		return os;
	}
};



struct RadioMsgRelayCmd
{
	uint8_t UavId; // ID of the UAV who the command is for (15 for all UAVs)
	uint8_t MsgId; // ID of this message, so that UAVs know if they relayed this msg yet.

	uint16_t HeightMin;
	uint16_t HeightMax;

	int AreaMinX;
	int AreaMinY;
	uint16_t AreaDX;
	uint16_t AreaDY;
	uint16_t AreaRotation;

	int LandX;
	int LandY;
	uint16_t LandHeading;
	bool LandLeftTurn;

	// See EAutoPilotMode
	// Home: go home and land (waiting for others)
	// Land: land _now_ (don't accept this command is for all UAVs)
	uint8_t Mode;
	bool EnablePlanner;

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelayCmd& struc)
	{
		os << "UavId=" << +struc.UavId << " MsgId=" << +struc.MsgId << " HeightMin=" << struc.HeightMin << " HeightMax=" << struc.HeightMax \
				<< " AreaMinX=" << struc.AreaMinX << " AreaMinY=" << struc.AreaMinY \
				<< " AreaDX=" << struc.AreaDX << " AreaDY=" << struc.AreaDY << " AreaRotation=" << struc.AreaRotation \
				<< " LandX=" << struc.LandX << " LandY=" << struc.LandY << " LandHeading=" << struc.LandHeading \
				<< " LandLeftTurn=" << struc.LandLeftTurn << " Mode=" << struc.Mode << " EnablePlanner=" << struc.EnablePlanner;
		return os;
	}
};
// 110 bits
#define RADIO_STRUCT_CMD(n) \
	unsigned _##n##_UavId : 4; \
	unsigned _##n##_MsgId : 4; \
	unsigned _##n##_HeightMin : 8; \
	unsigned _##n##_HeightMax : 8; \
	signed _##n##_AreaMinX : 10; \
	signed _##n##_AreaMinY : 10; \
	unsigned _##n##_AreaDX : 10; \
	unsigned _##n##_AreaDY : 10; \
	unsigned _##n##_AreaRotation : 8; \
	signed _##n##_LandX : 13; \
	signed _##n##_LandY : 13; \
	unsigned _##n##_LandHeading : 8; \
	unsigned _##n##_LandLeftTurn : 1; \
	unsigned _##n##_Mode : 2; \
	unsigned _##n##_EnablePlanner : 1;
	//unsigned _##n##_Unused : 3;

#define RADIO_STRUCT_CMD_GET(struc, n, var) struc._##n##_##var

#define RADIO_STRUCT_CMD_UNPACK(unpacked, packed, n) \
	unpacked.UavId = 			packed._##n##_UavId; \
	unpacked.MsgId = 			packed._##n##_MsgId; \
	unpacked.HeightMin = 		packed._##n##_HeightMin; \
	unpacked.HeightMax = 		packed._##n##_HeightMax; \
	unpacked.AreaMinX = 		packed._##n##_AreaMinX; \
	unpacked.AreaMinY = 		packed._##n##_AreaMinY; \
	unpacked.AreaDX = 			packed._##n##_AreaDX; \
	unpacked.AreaDY = 			packed._##n##_AreaDY; \
	unpacked.AreaRotation = 	packed._##n##_AreaRotation; \
	unpacked.LandX = 			packed._##n##_LandX; \
	unpacked.LandY = 			packed._##n##_LandY; \
	unpacked.LandHeading = 		packed._##n##_LandHeading; \
	unpacked.LandLeftTurn =		packed._##n##_LandLeftTurn; \
	unpacked.Mode =				packed._##n##_Mode; \
	unpacked.EnablePlanner =	packed._##n##_EnablePlanner;

#define RADIO_STRUCT_CMD_PACK(unpacked, packed, n) \
	packed._##n##_UavId = 			unpacked.UavId; \
	packed._##n##_MsgId = 			unpacked.MsgId; \
	packed._##n##_HeightMin = 		unpacked.HeightMin; \
	packed._##n##_HeightMax = 		unpacked.HeightMax; \
	packed._##n##_AreaMinX = 		unpacked.AreaMinX; \
	packed._##n##_AreaMinY = 		unpacked.AreaMinY; \
	packed._##n##_AreaDX = 			unpacked.AreaDX; \
	packed._##n##_AreaDY = 			unpacked.AreaDY; \
	packed._##n##_AreaRotation =	unpacked.AreaRotation; \
	packed._##n##_LandX = 			unpacked.LandX; \
	packed._##n##_LandY = 			unpacked.LandY; \
	packed._##n##_LandHeading = 	unpacked.LandHeading; \
	packed._##n##_LandLeftTurn = 	unpacked.LandLeftTurn; \
	packed._##n##_Mode =		 	unpacked.Mode; \
	packed._##n##_EnablePlanner =	unpacked.EnablePlanner;



struct RadioMsgRelay
{
	uint8_t MessageType; // Extra (not used in packed radiomsg: // ERadioMsgRelayType
	// Bit tricky
	RadioMsgRelay() { Fires.Fire[0].UavId=0; Fires.Fire[1].UavId=0; }
	union {
		RadioMsgRelayPos Pos;
		RadioMsgRelayFires Fires;
		RadioMsgRelayCmd Cmd;
	};

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelay& struc)
	{
		os << " RelayType=" << +struc.MessageType;
		switch (struc.MessageType)
		{
		case RADIO_MSG_RELAY_POS:
			os << " Pos=[" << struc.Pos << "]";
			break;
		case RADIO_MSG_RELAY_FIRE:
			os << " Fires=[" << struc.Fires << "]";
			break;
		case RADIO_MSG_RELAY_CMD:
			os << " Cmd=[" << struc.Cmd << "]";
			break;
		}
		return os;
	}
};



struct RadioMsgRelays
{
	// 220 bits = 2 * 110 bits
	RadioMsgRelay Data[2];

	friend std::ostream& operator<<(std::ostream& os, const RadioMsgRelays& struc)
	{
		os << "[" << struc.Data[0] << "] [" << struc.Data[1] << "]";
		return os;
	}
};

struct RadioMsgPacked2Pos
{
	unsigned MessageType : 4;
	RADIO_STRUCT_POS(0)
	RADIO_STRUCT_POS(1)
};

struct RadioMsgPackedPos2Fire
{
	unsigned MessageType : 4;
	RADIO_STRUCT_POS(0)
	RADIO_STRUCT_FIRE(2)
	RADIO_STRUCT_FIRE(3)
};

struct RadioMsgPacked4Fire
{
	unsigned MessageType : 4;
	RADIO_STRUCT_FIRE(0)
	RADIO_STRUCT_FIRE(1)
	RADIO_STRUCT_FIRE(2)
	RADIO_STRUCT_FIRE(3)
};

struct RadioMsgPackedPosCmd
{
	unsigned MessageType : 4;
	RADIO_STRUCT_POS(0)
	RADIO_STRUCT_CMD(1)
};

struct RadioMsgPacked
{
	union
	{
		RadioMsgPacked2Pos Pos;
		RadioMsgPackedPos2Fire PosFire;
		RadioMsgPacked4Fire Fire;
		RadioMsgPackedPosCmd PosCmd;
	};
};

struct RadioMsg
{
	// 28 bytes for myrianed = 224 bits
	uint8_t MessageType; // ERadioMsgType
	RadioMsgRelays Data;

	void Unpack(const RadioMsgPacked& msg)
	{
		MessageType = msg.Pos.MessageType;
		switch(MessageType)
		{
			case RADIO_MSG_2POS:
			{
//				Data.Data[0].Pos.UavId = RADIO_STRUCT_POS_GET(msg.Pos, 0, UavId);
				RADIO_STRUCT_POS_UNPACK(Data.Data[0].Pos, msg.Pos, 0)
				Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
				RADIO_STRUCT_POS_UNPACK(Data.Data[1].Pos, msg.Pos, 1)
				Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
				break;
			}
			case RADIO_MSG_POS_2FIRE:
			{
				RADIO_STRUCT_POS_UNPACK(Data.Data[0].Pos, msg.PosFire, 0)
				Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[1].Fires.Fire[0], msg.PosFire, 2)
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[1].Fires.Fire[1], msg.PosFire, 3)
				Data.Data[1].MessageType = RADIO_MSG_RELAY_FIRE;
				break;
			}
			case RADIO_MSG_4FIRE:
			{
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[0].Fires.Fire[0], msg.Fire, 0)
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[0].Fires.Fire[1], msg.Fire, 1)
				Data.Data[0].MessageType = RADIO_MSG_RELAY_FIRE;
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[1].Fires.Fire[0], msg.Fire, 2)
				RADIO_STRUCT_FIRE_UNPACK(Data.Data[1].Fires.Fire[1], msg.Fire, 3)
				Data.Data[1].MessageType = RADIO_MSG_RELAY_FIRE;
				break;
			}
			case RADIO_MSG_POS_CMD:
			{
				RADIO_STRUCT_POS_UNPACK(Data.Data[0].Pos, msg.PosCmd, 0)
				Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
				RADIO_STRUCT_CMD_UNPACK(Data.Data[1].Cmd, msg.PosCmd, 1)
				Data.Data[1].MessageType = RADIO_MSG_RELAY_CMD;
				break;
			}
		}
	}
	void Pack(RadioMsgPacked& msg)
	{
		msg.Pos.MessageType = MessageType;
		switch(MessageType)
		{
			case RADIO_MSG_2POS:
			{
				RADIO_STRUCT_POS_PACK(Data.Data[0].Pos, msg.Pos, 0)
				RADIO_STRUCT_POS_PACK(Data.Data[1].Pos, msg.Pos, 1)
				break;
			}
			case RADIO_MSG_POS_2FIRE:
			{
				RADIO_STRUCT_POS_PACK(Data.Data[0].Pos, msg.PosFire, 0)
				RADIO_STRUCT_FIRE_PACK(Data.Data[1].Fires.Fire[0], msg.PosFire, 2)
				RADIO_STRUCT_FIRE_PACK(Data.Data[1].Fires.Fire[1], msg.PosFire, 3)
				break;
			}
			case RADIO_MSG_4FIRE:
			{
				RADIO_STRUCT_FIRE_PACK(Data.Data[0].Fires.Fire[0], msg.Fire, 0)
				RADIO_STRUCT_FIRE_PACK(Data.Data[0].Fires.Fire[1], msg.Fire, 1)
				RADIO_STRUCT_FIRE_PACK(Data.Data[1].Fires.Fire[0], msg.Fire, 2)
				RADIO_STRUCT_FIRE_PACK(Data.Data[1].Fires.Fire[1], msg.Fire, 3)
				break;
			}
			case RADIO_MSG_POS_CMD:
			{
				RADIO_STRUCT_POS_PACK(Data.Data[0].Pos, msg.PosCmd, 0)
				RADIO_STRUCT_CMD_PACK(Data.Data[1].Cmd, msg.PosCmd, 1)
				break;
			}
		}
	}

	friend std::ostream& operator<<(std::ostream& os, const RadioMsg& struc)
	{
		os << "MessageType=" << +struc.MessageType;
		os << " " << struc.Data;
		return os;
	}
};

#pragma pack()


// Structs for the simulation

struct SimRadioMsgRelayPos
{
	//SimRadioMsgRelayPos(): x(0), y(0), z(0), heading(0), speed(0), newZ(0), state(0), roll(0) {}
	float x;
	float y;
	float z;
	float heading;
	float speed;
	float newZ;
	int state;
	float roll;

	friend std::ostream& operator<<(std::ostream& os, const SimRadioMsgRelayPos& struc)
	{
		os << "x=" << struc.x << " y=" << struc.y << " z=" << struc.z << " heading=" << struc.heading;
		os << " speed=" << struc.speed << " newZ=" << struc.newZ << " state=" << struc.state << " roll=" << struc.roll;
		return os;
	}
};

struct SimRadioMsgRelayFire
{

	float x;
	float y;
	float varX;
	float varY;
	float rot;
	float pCam;
	float pTPA;
	float pMicro;
	float z;
};



struct SimRadioMsgRelay
{
	SimRadioMsgRelay(): MessageType(0), UavId(0) {}
	bool MessageType;
	int UavId;
	union {
		SimRadioMsgRelayPos pos;
		SimRadioMsgRelayFire fire;
	};

	friend std::ostream& operator<<(std::ostream& os, const SimRadioMsgRelay& struc)
	{
		os << "MessageType=" << struc.MessageType << " UavId=" << struc.UavId << " ";
		if (struc.MessageType == RADIO_MSG_RELAY_POS)
			os << struc.pos;
		return os;
	}
};

struct SimRadioMsgRelays
{
	SimRadioMsgRelay data[RADIO_NUM_RELAY_PER_MSG];

	friend std::ostream& operator<<(std::ostream& os, const SimRadioMsgRelays& struc)
	{
		os << "[" << struc.data[0] << "] [" << struc.data[1] << "] [" << struc.data[2] << "] [" << struc.data[3] << "]";
		return os;
	}
};

struct SimRadioMsg
{
	int MessageType;
	SimRadioMsgRelays data;

	friend std::ostream& operator<<(std::ostream& os, const SimRadioMsg& struc)
	{
		os << "MessageType=" << struc.MessageType << " " << struc.data;
		return os;
	}
};

#endif // RADIOSTRUCTS_H_
