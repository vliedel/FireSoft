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
	radio::Init(module_id);
	config.load("config.json");
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
		std::cout << "CTS line successfully opened" << std::endl;

	Power(true);

	LastSentBufStatusTime = get_cur_1us();
}

/**
 * Power on the PCB that contains the low-power peer-to-peer Myrianed radio.
 */
void CRadio::Power(bool enable) {
	int fd_pwr = 0;

	/* Get ENA_VCOM as gpio */
	fd_pwr = open("/sys/class/gpio/gpio38/value", O_WRONLY);
	if(fd_pwr > 0)
		std::cout << "ENA_VCOM succesfully opened" << std::endl;

	if (enable) {
		// Power on the Vitelec PCB (active low pin!)
		if (write(fd_pwr, "0", 2) < 0) {
			std::cerr << "Could not write byte to turn on radio" << std::endl;
		}
	}
	else {
		if (write(fd_pwr, "1", 2) < 0) {
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
	if (ReadFromRadio())
	{
		// Send next message in the outgoing buffer (blocking when buffer is not empty)
		WriteToRadio();
	}

	// read the stuff from the buffer
	ReadReceiveBuffer();

	VecMsg = readFromMsgPlanner(false);
	if (!VecMsg->empty())
	{
		std::cout << get_cur_1ms() << " RADIO " << ModuleId << " from MsgPlanner: ";
		dobots::print(VecMsg->begin(), VecMsg->end());
		// Should have more protocol here?
		WriteToOutBuffer(VecMsg);
		VecMsg->clear();
	}

	// Let the msgPlanner know about the buffer size
	if (get_duration(LastSentBufStatusTime, get_cur_1ms()) > config.MsgPlannerTickTime)
	{
		std::vector<int> vecMsg;
		vecMsg.push_back(PROT_RADIO_STATUS_BUF_SIZE);
		vecMsg.push_back(SendBuffer.size());
		writeToMsgPlanner(vecMsg);
		LastSentBufStatusTime = get_cur_1us();
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
uint16_t CRadio::CRC(const char *data, const int length, const uint8_t *precession, const int p_length,
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

bool CRadio::ReadFromRadio()
{
	return ReadUart();
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
		//header = (header << 8) | chr;
		header = (uint8_t) chr;
	}
	//std::cout << std::endl;
	return false;
}


bool CRadio::ReadUart()
{
	//std::cout << get_cur_1ms() << " Read UART " << Serial->available() << std::endl;

	// Check if we need to synchronize (also reads a new header)
	if (Synchronize)
	{
		if (!SynchronizeUart(LastReadHeader))
			return false;
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
				return false;
			}
		}
		else
			return false;
		LastReadHeaderUsed = false;
		std::cout << "Read new header: " << LastReadHeader << std::endl;
	}

	// Header is read successfully, now read the data
	if (Serial->available() < sizeof(RadioMsgPacked) + sizeof(RSSI) + sizeof(CheckSum) + sizeof(StopByte))
		return false;
	LastReadHeaderUsed = true;

	// There is only one type of message that can arrive here

	RadioMsgPacked data;
	if (!ReadData((char*)&data, sizeof(RadioMsgPacked)))
		return false;
	//	std::cout << "Received radio message:" << data << std::endl;

	RadioMsg msg;
	msg.Unpack(data);
	std::cout << get_cur_1ms() << " Received: " << msg << std::endl;
	ReceiveBuffer.push_back(msg);
	return true;
}

bool CRadio::ReadData(char* data, size_t size)
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

	// For now: do not check CTS, just write after successful read.
	uint8_t header = MYRIANED_HEADER;
	Serial->write((char*)&header, 1);
	std::cout << get_cur_1ms() << " written: " << +header << std::endl;

	uint8_t dataSize = 28;
	Serial->write((char*)&dataSize, 1);
	std::cout << get_cur_1ms() << " written: " << +dataSize << std::endl;

	Serial->write(data, length);
	std::cout << get_cur_1ms() << " Sent message:";
	for (int i=0; i<length; ++i)
		std::cout << " " << +(uint8_t)data[i];
	std::cout << std::endl;

	uint16_t crc = CRC(data, length, (uint8_t*) NULL, 0, (uint8_t*) NULL, 0);
	uint8_t crcMsb = ((crc & 0xFF00) >> 8);
	uint8_t crcLsb = crc & 0x00FF;
	Serial->write((char*)&crcMsb, 1);
	std::cout << get_cur_1ms() << " written: " << +crcMsb << std::endl;
	Serial->write((char*)&crcLsb, 1);
	std::cout << get_cur_1ms() << " written: " << +crcLsb << std::endl;
	Serial->write((char*)&StopByte, 1);
	std::cout << get_cur_1ms() << " written: " << +StopByte << std::endl;

	/*
	// Wait for CTS to go active, then write data to radio, then wait for CTS to go inactive.
	// This way we can be reasonably that the data will be transmitted over the radio.
	int ret;
	uint8_t cts[1];

	while(true) {
		// Read 'CTS' (clear to send) line
		ret = read(fd_cts, cts, 1);
		if( ret == -1 )
		{
			// Wait a little, we don't want to use too much cpu.
			// CTS will last for about 50ms, sending 33B at 115200b/s takes less than 3ms.
			//usleep(config.TickTime);
			continue;
		}

		// If 'CTS' is low, send reply
		if (cts[0])
		{

			uint8_t header = MYRIANED_HEADER;
			Serial->write((char*)&header, 1);
			std::cout << "written: " << +header << std::endl;

			uint8_t dataSize = 28;
			Serial->write((char*)&dataSize, 1);
			std::cout << "written: " << +dataSize << std::endl;

			Serial->write(data, length);
			std::cout << get_cur_1ms() << " Sent message:";
			for (int i=0; i<length; ++i)
				std::cout << " " << +(uint8_t)data[i];
			std::cout << std::endl;

			uint16_t crc = CRC(data, length, (uint8_t*) NULL, 0, (uint8_t*) NULL, 0);
			uint8_t crcMsb = ((crc & 0xFF00) >> 8);
			uint8_t crcLsb = crc & 0x00FF;
			Serial->write((char*)&crcMsb, 1);
			std::cout << "written: " << +crcMsb << std::endl;
			Serial->write((char*)&crcLsb, 1);
			std::cout << "written: " << +crcLsb << std::endl;
			Serial->write((char*)&StopByte, 1);
			std::cout << "written: " << +StopByte << std::endl;

			while (true)
			{
				// Read 'CTS' (clear to send) line
				ret = read(fd_cts, cts, 1);
				if (ret == -1)
				{
					// Wait a little, we don't want to use too much cpu.
					// CTS will last for about 50ms, sending 33B at 115200b/s takes less than 3ms.
					//usleep(config.TickTime);
					continue;
				}
				if (!cts[0])
				{
					break;
				}
				else
				{
					usleep(config.TickTime);
					std::cout << get_cur_1ms() << " Waiting for CTS to go inactive..." << std::endl;
				}
			}

			return;
		}
		else
		{
			// Wait a little, we don't want to use too much cpu.
			// CTS will last for about 50ms, sending 33B at 115200b/s takes less than 3ms.
			usleep(config.TickTime);
			std::cout << get_cur_1ms() << " CTS busy..." << std::endl;
			continue;
		}
	}
	*/
}

void CRadio::WriteToRadio()
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
