/**
 * @brief 
 * @file CGroundStation.cpp
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

#include "CGroundStation.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"

using namespace rur;

CGroundStation::CGroundStation(): ModuleId("Anonymous"), UavId(0), IntMsg(NULL), Serial(NULL), fd_cts(0),
		Synchronize(false), LastReadHeaderUsed(false),
		CheckSum(0), StopByte(0x16), RSSI(0) {

}

CGroundStation::~CGroundStation()
{

}

void CGroundStation::Init(std::string module_id)
{
	groundStation::Init(module_id);
	config.load("config.json");
	ModuleId = module_id;
	UavId = atoi(module_id.c_str());

//	RadioRoundState = RADIO_STATE_ROUND_IDLE;

	try
	{
		Serial = new BufferedAsyncSerial(config.PortName, MYRIANED_SERIAL_SPEED);
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}

	GsCmdStruct gsCmd;
	gsCmd.UavId = UAVS_NUM; // 10 for all uavs
	gsCmd.MsgId = 0;
	gsCmd.HeightMin = config.MinHeight;
	gsCmd.HeightMax = config.MaxHeight;
	gsCmd.AreaZero << config.AreaOriginX, config.AreaOriginY, 0;
	gsCmd.AreaSize << config.AreaSizeX, config.AreaSizeY, 0;
	gsCmd.AreaRotation.angle() = config.AreaRotation;
	gsCmd.Landing.Pos << config.LandPointX, config.LandPointY, 0;
	gsCmd.Landing.Heading.angle() = config.LandHeading;
	gsCmd.Landing.LeftTurn = config.LandLeftTurn;
	gsCmd.Mode = AP_PROT_MODE_WP;
	gsCmd.EnablePlanner = true;



	CmdMsg.MessageType = RADIO_MSG_POS_CMD;
	CmdMsg.Data.Data[0].MessageType = RADIO_MSG_RELAY_POS;
	CmdMsg.Data.Data[0].Pos.UavId = 0; // Invalid msg

	CmdMsg.Data.Data[1].MessageType = RADIO_MSG_RELAY_CMD;
	gsCmd.ToRadioMsg(CmdMsg.Data.Data[1].Cmd);

	WriteToOutBuffer(CmdMsg);
	LastWriteTime = get_cur_1ms();
}

void CGroundStation::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}

	// Read the stuff, puts it in a buffer
	ReadFromRadio();

	// Read the stuff from the buffer
	ReadReceiveBuffer();

	VecMsg = readFromGuiInterface(false);
	if (!VecMsg->empty())
	{
		std::cout << "RADIO " << ModuleId << " from GuiInterface: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		// Should have more protocol here?

		VecMsgType::iterator it = VecMsg->begin();
		while (it != VecMsg->end())
		{
			int type = *it++;
			//std::cout << "Type=" << type << std::endl;
			switch (type)
			{
				case RADIO_MSG_RELAY_CMD:
				{
					RadioMsgRelayCmd cmdMsg;
					it = FromCont(cmdMsg, it, VecMsg->end());
					CmdMsg.Data.Data[1].Cmd = cmdMsg;
					break;
				}
			}

		}
		VecMsg->clear();
	}

	// Send everything that is in the outgoing buffer (blocking)
	// TODO: make non blocking?
	if (get_duration(LastWriteTime, get_cur_1ms()) > 125) // TODO: magic number
	{
		WriteToRadio();
		WriteToOutBuffer(CmdMsg);
		LastWriteTime = get_cur_1ms();
	}

	usleep(config.TickTime);
}

/**
 * Checksum for next byte, requiring previously calculated checksum as argument.
 */
uint16_t culCalcCRC(uint8_t crcData, uint16_t crcReg) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
			crcReg = (crcReg << 1) ^ MYRIANED_CRC16_POLY;
		else
			crcReg = (crcReg << 1);
		crcData <<= 1;
	}
	return crcReg;
}

/**
 * There are some idiocracies in checksum calculations. On incoming messages the checksum is calculated
 * over the payload itself plus the preceding length field and the subsequent rssi field. For outgoing
 * messages the checksum is only calculated over the payload itself (not the length byte).
 */
uint16_t CGroundStation::CRC(const char *data, const int length, const uint8_t *precession, const int p_length,
		const uint8_t *succession, const int s_length) {
	uint16_t checksum = MYRIANED_CRC_INIT;
	uint8_t i;

	for (i = 0; i != p_length; i++)
		checksum = culCalcCRC(precession[i], checksum);

	for (i = 0; i != length; i++)
		checksum = culCalcCRC(data[i], checksum);

	for (i = 0; i != s_length; i++)
		checksum = culCalcCRC(succession[i], checksum);

	return checksum;
}

void CGroundStation::ReadFromRadio()
{
	ReadUart();
}

bool CGroundStation::SynchronizeUart(RadioMsgHeader& msgHdr)
{
	if (Serial->available() < 2*sizeof(RadioMsgHeader))
		return false;

	std::cout << "Synchronizing the radio stream on start of the message with magic header.." << std::endl;
	char chr;
	RadioMsgHeaderType header;
	Serial->read((char*)&header, sizeof(RadioMsgHeaderType));
	while (Serial->available() >= sizeof(RadioMsgHeader))
	{
		//std::cout << " header=" << header;
		if (header == MYRIANED_HEADER)
		{
			msgHdr.Header = header;
			Serial->read((char*)&msgHdr+sizeof(RadioMsgHeaderType), sizeof(RadioMsgHeader)-sizeof(RadioMsgHeaderType));
			Synchronize = false;
			LastReadHeaderUsed = false;
			//std::cout << std::endl;
			return true;
		}
		Serial->read(&chr, 1);
		//header = (header << 8) | chr;
		header = (uint8_t) chr;
	}
	//std::cout << std::endl;
	return false;
}


void CGroundStation::ReadUart()
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
		if (Serial->available() >= sizeof(RadioMsgHeader))
		{
			Serial->read((char*)&LastReadHeader, sizeof(RadioMsgHeader));
			if (LastReadHeader.Header != MYRIANED_HEADER)
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
	if (Serial->available() < sizeof(RadioMsgPacked) + sizeof(RSSI) + sizeof(CheckSum) + sizeof(StopByte))
		return;
	LastReadHeaderUsed = true;

	// There is only one type of message that can arrive here

	RadioMsgPacked data;
	if (!ReadData((char*)&data, sizeof(RadioMsgPacked)))
		return;
	//	std::cout << "Received radio message:" << data << std::endl;

	RadioMsg msg;
	msg.Unpack(data);
	std::cout << "Received: " << msg << std::endl;
	ReceiveBuffer.push_back(msg);
}

bool CGroundStation::ReadData(char* data, size_t size)
{
	if (size+1 != LastReadHeader.DataSize)
	{
		std::cerr << "size=" << +size << " LastReadHeader.DataSize=" << +LastReadHeader.DataSize << std::endl;
		Synchronize = true;
		return false;
	}

	if (size > 0)
		Serial->read(data, size);

	char chr;
	Serial->read(&chr, 1);
	RSSI = chr;

	// get the checksum over serial
	uint16_t checkSum1 = 0;
	Serial->read(&chr, 1);
	checkSum1 = (uint8_t)chr;
	Serial->read(&chr, 1);
	checkSum1 = (checkSum1 << 8) | (uint8_t) chr;



	Serial->read(&chr, 1);

	uint8_t length = 29;

	// calculate the checksum
	uint16_t checkSum2 = CRC(data, 28, &length, 1, &RSSI, 1);
	if (checkSum1 != checkSum2)
	{
		std::cerr << "Read checksum error: " << +checkSum1 << " vs " << +checkSum2 << std::endl;
		std::cerr << "Check if I by accident did swap MSB and LSB..." << std::endl;
		for (int i = 0; i < size; ++i) {
			std::cerr << +data[i];
		}
		std::cerr << " RSSI=" << +RSSI << std::endl;
		return false;
	}
	if (chr != StopByte) {
		std::cout << "StopByte error: " << +chr << " (should be 0x16=22) " << std::endl;
		return false;
	}
	return true;
}

void CGroundStation::WriteData(const char* data, ssize_t length) {

	// For now: we can ignore the cts and just send the data to the radio.
	// This means we are not sure the data is actually sent over the radio!
	uint8_t header = MYRIANED_HEADER;
	Serial->write((char*)&header, 1);
	uint8_t dataSize = 28;
	Serial->write((char*)&dataSize, 1);
	Serial->write(data, length);
	uint16_t crc = CRC(data, length, (uint8_t*) NULL, 0, (uint8_t*) NULL, 0);
	uint8_t crcMsb = ((crc & 0xFF00) >> 8);
	uint8_t crcLsb = crc & 0x00FF;
	Serial->write((char*)&crcMsb, 1);
	Serial->write((char*)&crcLsb, 1);
	Serial->write((char*)&StopByte, 1);
	//std::cout << "Message " << std::string(data, length) << " sent" << std::endl;
	std::cout << get_cur_1ms() << " Sent message:";
	for (int i=0; i<length; ++i)
		std::cout << " " << +(uint8_t)data[i];
	std::cout << std::endl;

	// The ground station doesn't have a gpio line for the cts.
	// Check for TIOCM_CTS with ioctl(fd, TIOCMGET).
//	int ret;
//	uint8_t cts[1];
//	while(true) {
//		// Read 'CTS' line
//		ret = read(fd_cts, cts, 1);
//		if( ret == -1 )
//			return;
////			continue;
//
//		// If 'CTS' is low, send reply
//		if( cts[0] )
//		{
//			Serial->write(data, length);
//			std::cout << "Message " << std::string(data) << " sent" << std::endl;
//			return;
//		}
//		else
//		{
//			std::cout << "CTS busy..." << std::endl;
////			continue;
//			return;
//		}
//	}
}

void CGroundStation::WriteToRadio()
{
	RadioMsg radioMsg;
	if (SendBuffer.empty())
		return;

	radioMsg = SendBuffer.front();
	SendBuffer.pop_front();

	RadioMsgPacked data;
	radioMsg.Pack(data);
	WriteData((char*)(&data), sizeof(RadioMsgPacked));


	//writeToMsgPlanner(PROT_RADIOSTATUS_IDLE);
}

bool CGroundStation::ReadReceiveBuffer()
{
	UavStruct uav;
	while (!ReceiveBuffer.empty())
	{
		for (int i=0; i<RADIO_NUM_RELAY_PER_MSG; ++i)
		{
			switch (ReceiveBuffer.front().Data.Data[i].MessageType)
			{
				case RADIO_MSG_RELAY_POS:
				{
					uav.UavId = ReceiveBuffer.front().Data.Data[i].Pos.UavId -1;

					// Ignore invalid uav IDs and own messages
					if ((uav.UavId > -1) && (uav.UavId != UavId))
					{
						uav.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Pos);
						std::cout << "Received: " << ReceiveBuffer.front().Data.Data[i] << " === " << uav << std::endl;
						VecMsgType vecMsg;
						vecMsg.push_back(PROT_RADIO_MSG_RELAY);
						ToCont(ReceiveBuffer.front().Data.Data[i], vecMsg);
						writeToMapUavs(vecMsg);

						vecMsg.clear();
						vecMsg.push_back(PROT_RADIO_MSG_RELAY_POS);
						ToCont(ReceiveBuffer.front().Data.Data[i].Pos, vecMsg);
						writeToGuiInterface(vecMsg);
					}
					break;
				}
				case RADIO_MSG_RELAY_FIRE:
				{
					FireStruct fire;

					VecMsgType vecMsg;
					fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[0]);
					if (fire.UavId > -1)
					{
						std::cout << "Received: " << ReceiveBuffer.front().Data.Data[i].Fires.Fire[0] << std::endl;

						vecMsg.push_back(PROT_FIRE_STRUCT);
						ToCont(fire, vecMsg);
						writeToMapFire(vecMsg);

						vecMsg.clear();
						vecMsg.push_back(PROT_RADIO_MSG_RELAY_FIRE);
						ToCont(ReceiveBuffer.front().Data.Data[i].Fires.Fire[0], vecMsg);
						writeToGuiInterface(vecMsg);
					}

					fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[1]);
					if (fire.UavId > -1)
					{
						std::cout << "Received: " << ReceiveBuffer.front().Data.Data[i].Fires.Fire[1] << std::endl;

						vecMsg.clear();
						vecMsg.push_back(PROT_FIRE_STRUCT);
						ToCont(fire, vecMsg);
						writeToMapFire(vecMsg);

						vecMsg.clear();
						vecMsg.push_back(PROT_RADIO_MSG_RELAY_FIRE);
						ToCont(ReceiveBuffer.front().Data.Data[i].Fires.Fire[1], vecMsg);
						writeToGuiInterface(vecMsg);
					}
					break;
				}
				case RADIO_MSG_RELAY_CMD:
				{
					// Ignore
					break;
				}
			}
		}
		ReceiveBuffer.pop_front();
	}
}

void CGroundStation::WriteToOutBuffer(RadioMsg& msg)
{
	std::cout << "Requested msg to be sent: " << msg << std::endl;

	SendBuffer.push_back(msg);
}
