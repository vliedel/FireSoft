/**
 * @brief 
 * @file CComSalland.cpp
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

#include "CComSalland.h"
#include "Protocol.h"
#include "CTime.h"

using namespace rur;

CComSalland::CComSalland(): ShMemSelf(NULL), MutexSelf(NULL), MapSelf(NULL), Serial(NULL)
{
}

CComSalland::~CComSalland()
{
	delete ShMemSelf;
	delete Serial;
	Power(false);
}

void CComSalland::Init(std::string module_id)
{
	comSalland::Init(module_id);
	config.load("config.json");
	UavId = atoi(module_id.c_str());

	ShMemNameSelf = "mapSelf_" + module_id;
	try
	{
		// Open the shared memory
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());
		MapSelf = 			ShMemSelf->find<MapSelfStruct>("Map").first;
		MutexSelf = 		ShMemSelf->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		MapSelf = NULL;
		MutexSelf = NULL;
		delete ShMemSelf;
		std::cout << "Error in " << ShMemNameSelf << std::endl;
		throw;
	}

	try
	{
		boost::asio::serial_port_base::parity opt_parity(boost::asio::serial_port_base::parity::none);
		boost::asio::serial_port_base::character_size opt_csize(8);
		boost::asio::serial_port_base::flow_control opt_flow(boost::asio::serial_port_base::flow_control::hardware);
		Serial = new BufferedAsyncSerial(config.PortName, SALLAND_BAUDRATE, opt_parity, opt_csize, opt_flow);
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete ShMemSelf;
		delete Serial;
		throw;
	}

	Synchronize = true;
	LastReadHeaderUsed = false;
	LastTimeWriteUis = get_cur_1ms();
	State = ESALLAND_STATE_FAIL; // Best init?

	Power(true);

	// Check sizes
	std::cout << "sizeof(SallandMIM)=" << sizeof(SallandMIM) << " sizeof(SallandIDM)=" << sizeof(SallandIDM)
			<< " sizeof(SallandUIS)=" << sizeof(SallandUIS) << " sizeof(SallandGIS)=" << sizeof(SallandGIS)
			<< " sizeof(SallandUIDM)=" << sizeof(SallandUIDM) << std::endl;
}

void CComSalland::Tick()
{
	if (get_duration(LastTimeWriteUis, get_cur_1ms()) > config.IntervalTimeUis)
	{
		// Write UIS to salland
		SallandHeader header;
		header.Header = SALLAND_MAGIC_HEADER;
		header.Type = SALLAND_TYPE_UIDM;

		UavGeomStruct geom;
		{
			// Copy current state
			boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
			geom = MapSelf->UavData.Geom;
		}

		SallandUIDM uidm;
//		uidm.Id = UavId+1; // no need to set
//		uidm.Freshness = 1; // no need to set
		uidm.UIS.DataType = SALLAND_TYPE_UIS;
//		uidm.UIS.NodesHeard = 0; // no need to set
		uidm.UIS.FireProbability = 0; // TODO: fill this with something relevant
		uidm.UIS.ImageAvailable = 0; // TODO: fill this with something relevant
		uidm.UIS.DX = geom.GroundSpeed * cos(geom.Heading.angle()); // -127 to 127
		uidm.UIS.DY = geom.GroundSpeed * sin(geom.Heading.angle()); // -127 to 127
		uidm.UIS.DZ = geom.VerticalSpeed; // -127 to 127
		uidm.UIS.X = geom.Pos.x(); // -32767 to 32767
		uidm.UIS.Y = geom.Pos.y(); // -32767 to 32767
		uidm.UIS.Z = geom.Pos.z(); // 0 to 4096
		uidm.UIS.DT = 1;
		uidm.UIS.GpsTime = geom.GpsTimeStamp & 0x000000000000FFFF;
		uidm.UIS.FireSource = 0; // TODO: fill this with something relevant
//		uidm.NodeId = 0; // no need to set
		if (config.Debug > 1)
			std::cout << "Writing: " << uidm << std::endl;
		WriteData(header, (char*)&uidm, sizeof(SallandUIDM));
		LastTimeWriteUis = get_cur_1ms();
	}

	ReadUart();

	usleep(config.TickTime);
}

/**
 * Power on the Salland PCB .
 */
void CComSalland::Power(bool enable) {
	int fd_pwr = 0;

	/* Get ENA_VCOM as gpio */
	fd_pwr = open("/sys/class/gpio/gpio41/value", O_WRONLY);
	if(fd_pwr > 0)
		std::cout << "ENA_VCOM successfully opened" << std::endl;

	if (enable) {
		// Power on the Vitelec PCB (active low pin!)
		if (write(fd_pwr, "0", 2) < 0)
			std::cerr << "Could not write byte to turn on salland pcb" << std::endl;
		else
			std::cout << "Powered on the salland pcb" << std::endl;
	}
	else {
		if (write(fd_pwr, "1", 2) < 0)
			std::cerr << "Could not write byte to turn off salland pcb" << std::endl;
		else
			std::cout << "Powered down the salland pcb" << std::endl;
	}

	/* Close filehandle again */
	close(fd_pwr);
}

bool CComSalland::SynchronizeUart(SallandHeader& header)
{
	if (Serial->available() < 2*sizeof(SallandHeader))
		return false;

	if (config.Debug > 0)
		std::cout << "Synchronizing the stream on start of the message with magic header.." << std::endl;
	char chr;
	SallandMagicHeader hdr;
	Serial->read((char*)&hdr, sizeof(SallandMagicHeader));
	//std::cout << +hdr << std::endl;
	while (Serial->available() >= sizeof(SallandHeader))
	{
		//std::cout << " header=" << +hdr;
		if (hdr == SALLAND_MAGIC_HEADER)
		{
			header.Header = hdr;
			Serial->read((char*)&header+sizeof(SallandMagicHeader), sizeof(SallandHeader)-sizeof(SallandMagicHeader)); // Reading 0 bytes works?
			Synchronize = false;
			LastReadHeaderUsed = false;
			return true;
		}
		Serial->read(&chr, 1);
		hdr = (hdr << 8) | (SallandMagicHeader)chr;
		//std::cout << +hdr << std::endl;
	}
	return false;
}

bool CComSalland::ReadUart()
{
	//std::cout << get_cur_1ms() << " Read UART " << Serial->available() << std::endl;

	// Check if we need to synchronize (also reads a new header)
	if (Synchronize)
	{
		if (!SynchronizeUart(LastReadHeader))
			return false;
		if (config.Debug > 0)
			std::cout << "Synched, header=" << LastReadHeader << std::endl;
	}

	// Check if we need to read a new header
	if (LastReadHeaderUsed)
	{
		if (Serial->available() >= sizeof(SallandHeader))
		{
			Serial->read((char*)&LastReadHeader, sizeof(SallandHeader));
			if (LastReadHeader.Header != SALLAND_MAGIC_HEADER)
			{
				if (config.Debug > 0)
					std::cout << "Error header doesn't match, header=" << LastReadHeader << std::endl;
				Synchronize = true;
				return false;
			}
		}
		else
			return false;
		LastReadHeaderUsed = false;
		if (config.Debug > 0)
			std::cout << "Read new header: " << LastReadHeader << std::endl;
	}

	// Header is read successfully, now read the data
	switch (LastReadHeader.Type)
	{
		case SALLAND_TYPE_MIM:
		{
			if (Serial->available() < sizeof(SallandMIM))
				return false;
			LastReadHeaderUsed = true;
			SallandMIM mim;
			if (!ReadData((char*)&mim, sizeof(SallandMIM)))
				return false;
			std::cout << "Received: " << mim << std::endl;
//			return true;
			break;
		}
		case SALLAND_TYPE_UIDM:
		{
			if (Serial->available() < sizeof(SallandUIDM))
				return false;
			LastReadHeaderUsed = true;
			SallandUIDM uidm;
			if (!ReadData((char*)&uidm, sizeof(SallandUIDM)))
				return false;
			std::cout << "Received: " << uidm << std::endl;
//			return true;
			break;
		}
	}


//	if (Serial->available() < sizeof(RadioMsgPacked) + sizeof(RSSI) + sizeof(CheckSum) + sizeof(StopByte))
//		return false;
//	LastReadHeaderUsed = true;
//
//	// There is only one type of message that can arrive here
//
//	RadioMsgPacked data;
//	if (!ReadData((char*)&data, sizeof(RadioMsgPacked)))
//		return false;
//	//	std::cout << "Received radio message:" << data << std::endl;
//
//	RadioMsg msg;
//	msg.Unpack(data);
//	if (config.Debug > 0)
//		std::cout << get_cur_1ms() << " Received: " << msg << std::endl;
//	ReceiveBuffer.push_back(msg);
	return true;
}

bool CComSalland::ReadData(char* data, size_t size)
{
//	if (size != LastReadHeader.DataSize)
//	{
//		Synchronize = true;
//		return false;
//	}

	if (size > 0)
		Serial->read(data, size);

	char checkSum2(0);
	// Magic header is not included in checksum
	for (int i=sizeof(SallandMagicHeader); i<sizeof(SallandHeader); ++i)
		checkSum2 += ((char*)&LastReadHeader)[i];
	for (int i=0; i<size; ++i)
		checkSum2 += data[i];

	if (config.Debug > 1)
	{
		std::cout << "Read:";
		for (int i=sizeof(SallandMagicHeader); i<sizeof(SallandHeader); ++i)
			std::cout << " " << +((uint8_t*)&LastReadHeader)[i];
		for (int i=0; i<size; ++i)
			std::cout << " " << +data[i];
		std::cout << std::endl;
	}

	char checkSum1;
	Serial->read(&checkSum1, 1);

	if (checkSum1 != checkSum2)
	{
		if (config.Debug > 0)
			std::cout << "Error: read checksum: " << +checkSum1 << " vs " << +checkSum2 << std::endl;
		return false;
	}
	return true;
}

void CComSalland::WriteData(SallandHeader& header, char* data, size_t size)
{
	char checkSum(0);
	// Magic header is not included in checksum
	for (int i=sizeof(SallandMagicHeader); i<sizeof(SallandHeader); ++i)
		checkSum += ((char*)&header)[i];
	for (int i=0; i<size; ++i)
		checkSum += data[i];
	Serial->write((char*)&header,sizeof(SallandHeader));
	Serial->write(data, size);
	Serial->write(&checkSum,1);
	if (config.Debug > 1)
	{
		std::cout << "Written:";
		for (int i=0; i<sizeof(SallandHeader); ++i)
			std::cout << " " << +((uint8_t*)&header)[i];
		for (int i=0; i<size; ++i)
			std::cout << " " << +(uint8_t)(data[i]);
		std::cout << " " << +(uint8_t)checkSum << std::endl;
	}
}
