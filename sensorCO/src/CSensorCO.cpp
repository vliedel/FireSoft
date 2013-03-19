/**
 * @brief 
 * @file CSensorCO.cpp
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

#include "CSensorCO.h"
#include "Protocol.h"

using namespace rur;

CSensorCO::~CSensorCO()
{
	delete Serial;
	Power(false);
}

void CSensorCO::Init(std::string module_id)
{
	sensorCO::Init(module_id);
	config.load("config.json");


	try
	{
		Serial = new BufferedAsyncSerial(config.PortName, SENSOR_CO_BAUDRATE);
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}
	State = SENSOR_CO_STATE_OFF;

	Power(true);

	Write('P');
//	Write('V');
	Write('H');
}


/**
 * Power on the PCB that contains the sensor.
 */
void CSensorCO::Power(bool enable) {
	int fd_pwr = 0;

	/* Get ENA_VCOM as gpio */
	fd_pwr = open("/sys/class/gpio/gpio46/value", O_WRONLY);
	if(fd_pwr > 0)
		std::cout << "ENA_CO successfully opened" << std::endl;

	if (enable)
	{
		// Power on the van Mierlo PCB (active low pin!)
		if (write(fd_pwr, "0", 2) < 0)
			std::cerr << "Could not write byte to turn on CO sensor" << std::endl;
		else
		{
			std::cout << "Powered on the CO sensor" << std::endl;
			State = SENSOR_CO_STATE_COLD;
		}
	}
	else
	{
		if (write(fd_pwr, "1", 2) < 0)
			std::cerr << "Could not write byte to turn off CO sensor" << std::endl;
		else
		{
			std::cout << "Powered down the CO sensor" << std::endl;
			State = SENSOR_CO_STATE_OFF;
		}
	}

	/* Close filehandle again */
	close(fd_pwr);
}


void CSensorCO::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}

//	size_t read = Serial->read(ReadBuf, SENSOR_CO_READBUF_SIZE);
//	std::cout.write(ReadBuf, read);

	std::string delim("\n");
	std::string readStr = Serial->readStringUntil(delim);
	if (config.Debug > 1 && readStr.size() > 0)
	{
		std::cout << "State=" << State << " Read: " << readStr << std::endl;
	}

	if (readStr.size() > 0)
	{
		switch (State)
		{
			case SENSOR_CO_STATE_OFF:
			{
				Power(true);
				break;
			}
			case SENSOR_CO_STATE_COLD:
			{
				if (readStr.compare("Heating") == 0)
				{
					if (config.Debug > 0)
						std::cout << "Sensor is heating" << std::endl;
					State = SENSOR_CO_STATE_HEATING;
				}
				else
				{
//					Write('H');
				}
				break;
			}
			case SENSOR_CO_STATE_HEATING:
			{
				if (readStr.compare("Ready") == 0)
				{
					if (config.Debug > 0)
						std::cout << "Sensor is ready" << std::endl;
					State = SENSOR_CO_STATE_READY;

//					Write('W');
					Write('G');
				}
				break;
			}
			case SENSOR_CO_STATE_READY:
			{
				if (readStr.compare("Go") == 0)
				{
					if (config.Debug > 0)
						std::cout << "Sensor is printing values" << std::endl;
					State = SENSOR_CO_STATE_PRINT;
				}
				else
				{
//					Write('W');
//					Write('G');
				}
				break;
			}
			case SENSOR_CO_STATE_PRINT:
			{
				if (readStr.size() > 0)
				{
//					std::string val, tail;
//					val = readStr.substr(0, readStr.size()-6);
//					tail = readStr.substr(readStr.size()-5);
//					std::cout << "val=" << val << " tail=" << tail << std::endl;

					bool number = true;
					for (int i=0; i<readStr.size(); ++i)
						if (readStr[i] < 48 || readStr[i] > 57)
							number = false;
					if (number)
//						std::cout << "val=" << atoi(readStr.c_str()) << std::endl;
						std::cout << readStr << std::endl;
				}
				break;
			}
		}
	}


	usleep(config.TickTime);
}

void CSensorCO::Write(const char c)
{
	char chr = c;
	Serial->write(&chr, 1);
	if (config.Debug > 1)
		std::cout << "Written: " << c << std::endl;
}
