/**
 * @brief Radio class: can receive and send messages via the radio.
 * @file CRadio.cpp
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

#include "CRadio.h"
#include <iostream>

using namespace rur;
using namespace std;

CRadio::CRadio(): ModuleId("Anonymous"), UavId(0), IntMsg(NULL), Serial(NULL), fd_cts(0),
		Synchronize(false), LastReadHeaderUsed(false),
		CheckSum(0), StopByte(0x16), RSSI(0) {

}

CRadio::~CRadio()
{
	close(fd_cts);
	Power(false);
	delete Serial;
}

void CRadio::Init(std::string &module_id) {
	ModuleId = module_id;
	UavId = atoi(module_id.c_str());

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

	// Get gpio as 'CTS'
	fd_cts = open("/sys/class/gpio/gpio56/value", O_RDONLY);
	if( fd_cts > 0 )
		printf("CTS line succesfully opened\n");

	Power(true);
}

/**
 * Power on the PCB that contains the low-power peer-to-peer Myrianed radio.
 */
void CRadio::Power(bool enable) {
	int fd_pwr = 0;

	/* Get ENA_VCOM as gpio */
	fd_pwr = open("/sys/class/gpio/gpio38/value", O_WRONLY);
	if( fd_pwr > 0 )
		printf("ENA_VCOM succesfully opened\n");

	if (enable) {
		// Power on the Vitelec PCB (active low pin!)
		if (write( fd_pwr, "0", 2 ) < 0) {
			std::cerr << "Could not write byte to turn on radio" << std::endl;
		}
	}
	else {
		if (write( fd_pwr, "1", 2 ) < 0) {
			std::cerr << "Could not write byte to turn off radio" << std::endl;
		}
	}

	/* Close filehandle again */
	close(fd_pwr);
}


/**
 * The message planner requires a RADIO_STATE_ROUND_IDLE message after things
 * have been written towards the radio.
 */
void CRadio::Tick()
{
	// Read the stuff, puts it in a buffer
	ReadFromRadio();

	// read the stuff from the buffer
	ReadReceiveBuffer();

	VecMsg = readFromMsgPlanner(false);
	if (!VecMsg->empty())
	{
		std::cout << "RADIO " << ModuleId << " from MsgPlanner: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		// Should have more protocol here?
		WriteToOutBuffer(VecMsg);
		VecMsg->clear();
	}

	// Send everything that is in the outgoing buffer (blocking)
	WriteToRadio();

	usleep(config.TickTime);
}

/**
 * Checksum for next byte, requiring previously calculated checksum as argument.
 */
uint16_t culCalcCRC(uint8_t crcData, uint16_t crcReg) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
			crcReg = (crcReg << 1) ^ CRC16_POLY;
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
uint16_t CRadio::CRC(const char *data, const int length, const uint8_t *precession, const int p_length,
		const uint8_t *succession, const int s_length) {
	uint16_t checksum = CRC_INIT;
	uint8_t i;

	for (i = 0; i != p_length-1; i++)
		checksum = culCalcCRC(precession[i], checksum);

	for (i = 0; i != length-1; i++)
		checksum = culCalcCRC(data[i], checksum);

	for (i = 0; i != s_length-1; i++)
		checksum = culCalcCRC(succession[i], checksum);

	return checksum;
}

void CRadio::ReadFromRadio()
{
	ReadUart();
}

bool CRadio::SynchronizeUart(RadioMsgHeader& msgHdr)
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
		header = (header << 8) | chr;
	}
	//std::cout << std::endl;
	return false;
}


void CRadio::ReadUart()
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
	if (Serial->available() < LastReadHeader.DataSize + sizeof(RSSI) + sizeof(CheckSum) + sizeof(StopByte))
		return;
	LastReadHeaderUsed = true;

	// There is only one type of message that can arrive here

	RadioMsgPacked data;
	if (!ReadData((char*)&data, sizeof(RadioMsgPacked)))
		return;
	//	std::cout << "Received radio message:" << data << std::endl;

	RadioMsg msg;
	msg.Unpack(data);
	ReceiveBuffer.push_back(msg);
}

bool CRadio::ReadData(char* data, size_t size)
{
	if (size != LastReadHeader.DataSize)
	{
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
	checkSum1 = chr;
	Serial->read(&chr, 1);
	checkSum1 = (checkSum1 << 8) | chr;

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
		std::cerr << std::endl;
		return false;
	}
	if (chr != StopByte) {
		std::cout << "StopByte error: " << +chr << " (should be 0x16=22) " << std::endl;
		return false;
	}
	return true;
}

/**
 * Blocking wait for sending data
 */
void CRadio::WriteData(const char* data, ssize_t length) {
	int ret;
	uint8_t cts[1];

	while(true) {
		// Read 'CTS' line
		ret = read(fd_cts, cts, 1);
		if( ret == -1 )
			continue;

		// If 'CTS' is low, send reply
		if( cts[0] )
		{
			Serial->write(data, length);
			std::cout << "Message " << string(data) << " sent" << std::endl;
		}
		else
			std::cout << "CTS busy..." << std::endl;
	}
}

void CRadio::WriteToRadio()
{
	RadioMsg radioMsg;
	if (!SendBuffer.empty())
	{
		radioMsg = SendBuffer.front();
		SendBuffer.pop_front();
	}
	RadioMsgPacked data;
	radioMsg.Pack(data);
	WriteData((char*)data, sizeof(RadioMsgPacked));

	writeToMsgPlanner(PROT_RADIOSTATUS_IDLE);
}

void CRadio::UpdateState()
{

}

bool CRadio::ReadReceiveBuffer()
{
	VecMsgType vecMsgUavs;
	//VecMsgType vecMsgSelf;
	VecMsgType vecMsgFire;
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
					std::cout << "Radio " << ModuleId << " sending: " << ReceiveBuffer.front().Data.Data[i] << " === " << uav << std::endl;

					vecMsgUavs.push_back(PROT_RADIO_MSG_RELAY);
					ToCont(ReceiveBuffer.front().Data.Data[i], vecMsgUavs);
				}
				break;
			}
			case RADIO_MSG_RELAY_FIRE:
			{
				FireStruct fire;

				fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[0]);
				if (fire.UavId > 0)
				{
					vecMsgFire.push_back(PROT_FIRE_STRUCT);
					ToCont(fire, vecMsgFire);
				}

				fire.FromRadioMsg(ReceiveBuffer.front().Data.Data[i].Fires.Fire[1]);
				if (fire.UavId > 0)
				{
					vecMsgFire.push_back(PROT_FIRE_STRUCT);
					ToCont(fire, vecMsgFire);
				}
				break;
			}
			case RADIO_MSG_RELAY_CMD:
			{
				int id = ReceiveBuffer.front().Data.Data[i].Cmd.UavId -1;
				if (id < 0)
					break;

				// If message is meant for this uav, execute command
				if (id == UavId || id == 14) // TODO: magic number
				{

				}
				// If message is meant for other uav, relay command
				if (id != UavId || id == 14) // TODO: magic number
				{

				}

				VecMsgType vecMsgSelf;
				ToCont(ReceiveBuffer.front().Data.Data[i].Cmd, vecMsgSelf);
				vecMsgSelf.push_back(PROT_MAPSELF_GS_CMD);
				writeToMapSelf(vecMsgSelf);

				break;
			}
			}
		}
		ReceiveBuffer.pop_front();
	}

	if (!vecMsgFire.empty())
	{
		writeToMapFire(vecMsgFire);
	}

	if (!vecMsgUavs.empty())
	{
		writeToMapUAVs(vecMsgUavs);
		return true;
	}

	return false;
}

/**
 * Writes it to a buffer
 */
void CRadio::WriteToOutBuffer(VecMsgType* vecMsg)
{
	RadioMsg msg;
	FromCont(msg, vecMsg->begin(), vecMsg->end());
	SendBuffer.push_back(msg);
}
