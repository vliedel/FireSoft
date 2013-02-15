/**
 * @brief 
 * @file CTests.cpp
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

#include "CTests.h"
//#include <Eigen/Core>
#include <Eigen/Geometry>
#include "CTime.h"
#include "Fitness.h"

//#include <boost/asio.hpp>
//#include <boost/thread.hpp>



using namespace rur;

CTests::~CTests()
{

}

void CTests::Init(std::string module_id)
{
	config.load("config.json");
//int id, FitnessSource source, Position& center, float amplitude, float sigmaX, float sigmaY, float rotation, float minVal=0, float maxVal=0)

	srand(get_cur_1ms());
	int repeats = 100*1000;
	int numGauss = 1000;
	int numValPerGauss = 20*20*M_PI; // Circle area with radius = 2*coverage_sigma

	Position c;
	c << 0,0,0;
	FitnessGaussian2D gauss(0, FITSRC_STATIC, c, 1, 1, 1, 0);
	std::vector<FitnessGaussian2D> gaussians;
	for (int i=0; i<numGauss; ++i)
	{
		c << rand()%10, rand()%10, 0;
		gauss.Center = c;
		gaussians.push_back(gauss);
	}

	float fit=0;
	c << rand()%10, rand()%10, 0;
	long timeStart = get_cur_1us();
	std::vector<FitnessGaussian2D>::iterator it;
	for (int j=0; j<repeats; ++j)
		for (it=gaussians.begin(); it != gaussians.end(); ++it)
			fit += it->GetValue(c);
	std::cout << "time to get fitness with gaussians: " << (get_cur_1us()-timeStart)/1000 << "ms" << std::endl;



	std::vector<float> values;
	for (int i=0; i<numGauss*numValPerGauss; ++i)
		values.push_back(rand()%10);

	fit=0;
	timeStart = get_cur_1us();
	std::vector<float>::iterator itv;
	for (int j=0; j<repeats; ++j)
		for (itv=values.begin(); itv != values.end(); ++itv)
			fit += *itv;
	std::cout << "time to get fitness with grid: " << (get_cur_1us()-timeStart)/1000 << "ms" << std::endl;

//	return;

//	Position p0, px, py, p;
//	float x=2;
//	float y=3;
//	p0 << 0+x,0+y,1;
//	px << 10+x,3+y,2;
//	py << 2+x,10+y,3;
//	Plane plane(p0, px, py);
//	for (int i=0; i<10; ++i)
//	{
//		p << 3+x,0+y+i,0;
//		std::cout << p.transpose() << "=" << plane.GetValue(p) << std::endl;
//	}
//	return;


//	TestConfig cfg, cfg2;
//	cfg.Int = 3;
//	cfg.flt = M_PI;
//	cfg.str = "hello world l  o  l";
//	cfg.save("cfgTest.json");
//
//	cfg2.load("cfgTest.json");
//	std::cout << "config=" << cfg2.Int << " " << cfg2.flt << " " << cfg2.str << std::endl;
//
//	Position pos1, pos2, diff;
//	pos1 << 10, 20, 30;
//	pos2 << 20, 40, 60;
//	diff = pos2 - pos1;
//	std::cout << (pos1+diff/2).transpose() << std::endl;


//	std::iostream os;
//	unsigned char c=13;
//	long l=123456;
//	os.write(&c, sizeof(unsigned char));
//	os.write(&l, sizeof(long));
//	AutoPilotMsgHeader apH;
//	os.write(&apH, sizeof(AutoPilotMsgHeader));

	uint32_t val = (11 << 24) | (12 << 16) | (13 << 8) | (14 << 0);
	std::cout << ((val & 0xFF000000) >> 24) << " " << ((val & 0x00FF0000) >> 16) << " " << ((val & 0x0000FF00) >> 8) << " " << (val & 0x000000FF) << std::endl;
	val = (val << 8) | 15;
	std::cout << ((val & 0xFF000000) >> 24) << " " << ((val & 0x00FF0000) >> 16) << " " << ((val & 0x0000FF00) >> 8) << " " << (val & 0x000000FF) << std::endl;

	AutoPilotMsgHeader msgHdr;

#pragma pack(1)
	struct fooo
	{
		uint8_t 					MsgType; 	// EAutoPilotMsgType
		AutoPilotMsgTimeType		TimeStamp;
		uint8_t 					DataSize;
	};
#pragma pack()

	fooo moo;
	moo.MsgType = 21;
	moo.TimeStamp = 22;
	moo.DataSize = 23;
	std::cout << "p=" << &msgHdr << " p2=" << &(msgHdr.MsgType) << std::endl;
	//memcpy((char*)&msgHdr.MsgType, &moo, sizeof(AutoPilotMsgHeader)-2);
	memcpy((char*)&msgHdr+2, &moo, sizeof(AutoPilotMsgHeader)-2);
	std::cout << "type=" << +msgHdr.MsgType << " time=" << +msgHdr.TimeStamp << " size=" << +msgHdr.DataSize << std::endl;

//	AutoPilotMsgHeaderU msgHdrU;
//	std::cout << "p=" << &msgHdr << " p2=" << &(msgHdr.MsgType) << std::endl;
//	memcpy(msgHdrU.Buf+2, &moo, sizeof(AutoPilotMsgHeader)-2);
//	std::cout << "type=" << +msgHdrU.Data.MsgType << " time=" << +msgHdrU.Data.TimeStamp << " size=" << +msgHdrU.Data.DataSize << std::endl;


	//return;

/*
	AutoPilotMsgPosition p, p2;
	//p.Data.X = 3;
	//p.Data.Y = 4;
	//p.Data.Z = 5;
	p.X = 3;
	p.Y = 4;
	p.Z = 5;
	for (int i=0; i<sizeof(AutoPilotMsgPosition); ++i)
	{
		//std::cout << p.Buf[i] << " ";
		std::cout << ((char*)&p)[i] << " ";
		//p2.Buf[i] = p.Buf[i];
		((char*)&p2)[i] = ((char*)&p)[i];
	}
	std::cout << std::endl;
	//std::cout << p2.Data.X << " " << p2.Data.Y << " " << p2.Data.Z << std::endl;
	std::cout << p2.X << " " << p2.Y << " " << p2.Z << std::endl;

//	p.Data.X = 13;
//	p.Data.Y = 14;
//	p.Data.Z = 15;
	p.X = 13;
	p.Y = 14;
	p.Z = 15;

	BufferedAsyncSerial serial("/dev/ttyUSB0", 115200);
	BufferedAsyncSerial serialB("/dev/ttyUSB1", 115200);
	//serial.write((char*)p.Buf, sizeof(AutoPilotMsgPosition));
	//serial.write((char*)&p, sizeof(AutoPilotMsgPosition));

	//this_thread::sleep(posix_time::seconds(2));
	sleep(1);
	//serialB.read((char*)p2.Buf, sizeof(AutoPilotMsgPosition));
	serialB.read((char*)&p2, sizeof(AutoPilotMsgPosition));
	//std::cout << p2.Data.X << " " << p2.Data.Y << " " << p2.Data.Z << std::endl;
	std::cout << p2.X << " " << p2.Y << " " << p2.Z << std::endl;
*/


	std::cout << "sizeof(AutoPilotMsgHeader)=" << sizeof(AutoPilotMsgHeader) << std::endl;
	std::cout << "sizeof(AutoPilotMsgSensorData)=" << sizeof(AutoPilotMsgSensorData) << std::endl;
//	std::cout << "sizeof(AutoPilotMsgField)=" << sizeof(AutoPilotMsgField) << std::endl;
	std::cout << "sizeof(AutoPilotMsgWayPoints)=" << sizeof(AutoPilotMsgWayPoints) << std::endl;
	std::cout << "sizeof(AutoPilotMsgWpStatus)=" << sizeof(AutoPilotMsgWpStatus) << std::endl;
	std::cout << "sizeof(AutoPilotMsgWpBounds)=" << sizeof(AutoPilotMsgWpBounds) << std::endl;


	Eigen::AngleAxisf rota(M_PI, Eigen::Vector3f::UnitX());
	std::cout << rota.matrix() << std::endl;


	Rotation2DType rot(4), rot2(3);
//	rot.angle() = 4;
//	rot2.angle() = 3;
	rot*=rot2;
	std::cout << rot.angle() << std::endl;

	//exit(0);


	BitTest bt;
	int a=3;
	int b=-3;
	bt._0_b = b;
	bt._0_a = a;

	std::cout << (int)bt._0_a << " " << (int)bt._0_b << std::endl;

	std::cout << "sizeof(RadioMsgPacked)=" << sizeof(RadioMsgPacked2Pos) << "=" << sizeof(RadioMsgPackedPos2Fire) \
			<< "=" << sizeof(RadioMsgPacked4Fire) << "=" << sizeof(RadioMsgPackedPosCmd) \
			<< "=" << sizeof(RadioMsgPacked) << std::endl;

	//tests::Init(module_id);
	std::vector<int> vec;
	std::vector<int> vec2;
	std::vector<int>::iterator iter;


	int posSize = 3;
	vec.clear();
	for (int i=0; i<posSize; ++i)
		vec.push_back(i);
	Position pos;
	Check<Position>(pos, vec, vec2, "Position");


	int speedSize = 1;
	vec.clear();
	for (int i=0; i<speedSize; ++i)
		vec.push_back(i);
	SpeedType speed;
	Check<SpeedType>(speed, vec, vec2, "Speed");


	int rotSize = 4;
	vec.clear();
	for (int i=0; i<rotSize; ++i)
		vec.push_back(i);
	Rotation rotation;
	Check<Rotation>(rotation, vec, vec2, "Rotation");


	int rotMatSize = 9;
	vec.clear();
	for (int i=0; i<rotMatSize; ++i)
		vec.push_back(i);
	RotationMatrix rotationMatrix;
	Check<RotationMatrix>(rotationMatrix, vec, vec2, "RotationMatrix");


	int rot2DSize = 1;
//	vec.clear();
//	for (int i=0; i<rotMatSize; ++i)
//		vec.push_back(i);
//	Rotation2DType rotation2DType(0);
//	Check<Rotation2DType>(rotation2DType, vec, vec2, "Rotation2DType");

//	int geomSize = posSize+speedSize+rotSize;
	int geomSize = posSize+speedSize+speedSize+rotMatSize+4*rot2DSize+1;
	vec.clear();
	for (int i=0; i<geomSize; ++i)
		vec.push_back(i);
	vec[geomSize-1] = 1; // bool RotationUpToDate
	UavGeomStruct uavGeomStruct;
	Check<UavGeomStruct>(uavGeomStruct, vec, vec2, "UavGeomStruct");


	int wayPointSize = 12;
	vec.clear();
	for (int i=0; i<wayPointSize; ++i)
		vec.push_back(i);
	WayPoint wayPoint;
	Check<WayPoint>(wayPoint, vec, vec2, "WayPoint");

/*
	int uavSize = 1+geomSize+1 + 2*wayPointSize;
	vec.clear();
	for (int i=0; i<uavSize; ++i)
		vec.push_back(i);
	vec[1+geomSize-1] = 1; // bool RotationUpToDate
	UavStruct uavStruct;
	Check<UavStruct>(uavStruct, vec, vec2, "UavStruct");


	//int mapUavSize = uavSize+3 + MAPUAV_RADIOMSG_HIST*(1+17)+1 + 2*wayPointSize+1;
	int mapUavSize = uavSize+3 + MAPUAV_RADIOMSG_HIST*(1+17)+1;
	vec.clear();
	for (int i=0; i<mapUavSize; ++i)
		vec.push_back(i);
	vec[1+geomSize-1] = 1; // bool RotationUpToDate
	vec[uavSize+2] = 1; // Bool connected
	for (int i=0; i<MAPUAV_RADIOMSG_HIST; ++i)
	{
		vec[uavSize+3 + i*(1+17) + 0] = 0; // relay msgtype = pos
		vec[uavSize+3 + i*(1+17) + 1] = 1; // uavid
		vec[uavSize+3 + i*(1+17) + 2] = 2; // x
		vec[uavSize+3 + i*(1+17) + 3] = 3; // y
		vec[uavSize+3 + i*(1+17) + 4] = 4; // z
		vec[uavSize+3 + i*(1+17) + 5] = 5; // heading
		vec[uavSize+3 + i*(1+17) + 6] = 6; // speed
		vec[uavSize+3 + i*(1+17) + 7] = 7; // state
		vec[uavSize+3 + i*(1+17) + 8] = 0; // roll
		vec[uavSize+3 + i*(1+17) + 9] = 9; // dx
		vec[uavSize+3 + i*(1+17) + 10] = 10; // dy
		vec[uavSize+3 + i*(1+17) + 11] = 11; // z
		vec[uavSize+3 + i*(1+17) + 12] = 0; // mode
		vec[uavSize+3 + i*(1+17) + 13] = 12; // radius
		vec[uavSize+3 + i*(1+17) + 14] = 13; // x
		vec[uavSize+3 + i*(1+17) + 15] = 14; // y
		vec[uavSize+3 + i*(1+17) + 16] = 15; // z
		vec[uavSize+3 + i*(1+17) + 17] = 16; // eta
	}
	MapUavStruct mapUavStruct;
	Check<MapUavStruct>(mapUavStruct, vec, vec2, "MapUavStruct");


	int mapSelfSize = uavSize+2+posSize+5 + 1+3*1 + 1+3*wayPointSize;
	vec.clear();
	for (int i=0; i<mapSelfSize; ++i)
		vec.push_back(i);
	vec[1+geomSize-1] = 1; // bool RotationUpToDate
	vec[uavSize+2+posSize+5] = 3; // Size of neighbours vector
	vec[uavSize+2+posSize+5+4] = 3; // Size of waypoints vector
	MapSelfStruct mapSelfStruct;
	Check<MapSelfStruct>(mapSelfStruct, vec, vec2, "MapSelfStruct");


	int simUavSize = mapSelfSize + 4;
	vec.clear();
	for (int i=0; i<simUavSize; ++i)
		vec.push_back(i);
	vec[1+geomSize-1] = 1; // bool RotationUpToDate
	vec[uavSize+2+posSize+5] = 3; // Size of neighbours vector
	vec[uavSize+2+posSize+5+4] = 3; // Size of waypoints vector
	vec[mapSelfSize+0] = 1; // bool
	vec[mapSelfSize+1] = 1; // bool
	vec[mapSelfSize+3] = 1; // bool
	SimUavStruct simUavStruct;
	Check<SimUavStruct>(simUavStruct, vec, vec2, "SimUavStruct");
*/

/*
	vec.clear();
	for (int i=0; i<1+4*(2+8); ++i)
		vec.push_back(i);
	vec[1] = 0;
	vec[1+10] = 0;
	vec[1+20] = 0;
	vec[1+30] = 0;
	SimRadioMsg simRadioMsg;
	Check<SimRadioMsg>(simRadioMsg, vec, vec2, "SimRadioMsg");
*/

	vec.clear();
	for (int i=0; i<1+(1+16)+(1+2*(11)); ++i)
		vec.push_back(i);
	vec[0] = RADIO_MSG_POS_2FIRE; // msgtype
	vec[1] = RADIO_MSG_RELAY_POS; // relay msgtype = pos
	vec[1+1+16] = RADIO_MSG_RELAY_FIRE; // relay msgtype = fires
	vec[1+1+16+1+ 0] = 1; // uavid
	vec[1+1+16+1+ 3] = 2; // varx
	vec[1+1+16+1+ 4] = 3; // vary
	vec[1+1+16+1+ 5] = 4; // rot
	vec[1+1+16+1+ 6] = 5; // pcam
	vec[1+1+16+1+ 7] = 6; // ptpa
	vec[1+1+16+1+ 8] = 7; // pgas
	vec[1+1+16+1+ 9] = 8; // Z
	vec[1+1+16+1+ 10] = 0; // Forwards

	vec[1+1+16+1+11+ 0] = 2; // uavid
	vec[1+1+16+1+11+ 3] = 2; // varx
	vec[1+1+16+1+11+ 4] = 3; // vary
	vec[1+1+16+1+11+ 5] = 4; // rot
	vec[1+1+16+1+11+ 6] = 5; // pcam
	vec[1+1+16+1+11+ 7] = 6; // ptpa
	vec[1+1+16+1+11+ 8] = 7; // pgas
	vec[1+1+16+1+11+ 9] = 8; // Z
	vec[1+1+16+1+11+ 10] = 0; // Forwards

	RadioMsg radioMsg;
	RadioMsgPacked msgp;
	Check<RadioMsg>(radioMsg, vec, vec2, "RadioMsg");

	// Extra check for packed radio msg
	radioMsg.Pack(msgp);
	radioMsg.Unpack(msgp);
	vec2.clear();
	ToCont(radioMsg, vec2);
	if (vec != vec2)
	{
		std::cout << ">>>> ERROR <<<< in " << "RadioMsg" << " to vector" << std::endl;
		dobots::print(vec2.begin(), vec2.end());
	}
	else
		std::cout << "RadioMsg" << " OK :D" << std::endl;


	vec.clear();
	for (int i=0; i<1+(1+16)+(1+15); ++i)
		vec.push_back(i);
	vec[0] = RADIO_MSG_POS_CMD; // msgtype
	vec[1] = RADIO_MSG_RELAY_POS; // relay msgtype = pos
	vec[1+1+16] = RADIO_MSG_RELAY_CMD; // relay msgtype = cmd
	vec[1+1+16+1+ 0] = 1; // uavid
	vec[1+1+16+1+ 1] = 2; // msgid
	vec[1+1+16+1+ 12] = 1; // land left turn
	vec[1+1+16+1+ 13] = 2; // mode
	vec[1+1+16+1+ 14] = 1; // enable planner

//	RadioMsg radioMsg;
//	RadioMsgPacked msgp;
	Check<RadioMsg>(radioMsg, vec, vec2, "RadioMsg");

	// Extra check for packed radio msg
	radioMsg.Pack(msgp);
	radioMsg.Unpack(msgp);
	vec2.clear();
	ToCont(radioMsg, vec2);
	if (vec != vec2)
	{
		std::cout << ">>>> ERROR <<<< in " << "RadioMsg" << " to vector" << std::endl;
		dobots::print(vec2.begin(), vec2.end());
	}
	else
		std::cout << "RadioMsg" << " OK :D" << std::endl;


	int fireSize = 17;
	vec.clear();
	for (int i=0; i<fireSize; ++i)
		vec.push_back(i);
	FireStruct fire;
	Check<FireStruct>(fire, vec, vec2, "FireStruct");

//	int mapFireSize = fireSize + 2;
//	vec.clear();
//	for (int i=0; i<mapFireSize; ++i)
//		vec.push_back(i);
//	vec[fireSize] = 1; // bool
//	vec[fireSize+1] = 1; // bool
//	MapFireStruct mapFire;
//	Check<MapFireStruct>(mapFire, vec, vec2, "MapFireStruct");




	//////////////////////////////////////
	// Testing out the Eigen matrix lib //
	//////////////////////////////////////
	Eigen::Matrix3d m = Eigen::Matrix3d::Random();
	m = (m + Eigen::Matrix3d::Constant(1.2)) * 50;
	std::cout << "m =" << std::endl << m << std::endl;
	Eigen::Vector3d v(1,2,3);
	std::cout << "m * v =" << std::endl << m * v << std::endl;

	Eigen::Array3i   v3(1, 2, 3);
	Eigen::Vector4f  v4(1, 2, 3, 4);
	Eigen::ArrayXf   v5(5);
	std::cout << "three ints:" << std::endl << v3 << std::endl;
	std::cout << "four floats:" << std::endl << v4/3 << std::endl;


	// Testing the serial connection, act as autopilot
	std::string devName = "/dev/ttyUSB" + module_id;
	std::cout << "Opening uart connection on: " << devName << std::endl;
	try
	{
		Serial = new BufferedAsyncSerial(devName, AP_PROT_SERIAL_SPEED);
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}
	Synchronize = true;
	LastReadHeaderUsed = true;
	LastWriteTime = get_cur_1ms();
}

void CTests::Tick()
{
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{

	}

	ReadUart();
//	if (get_cur_1ms() - LastWriteTime > 500)
//	{
//		LastWriteTime = get_cur_1ms();
//		std::string str = "abcdefghijklmno";
//		char chr = (rand()%26) +65;
//		str += chr;
//		Serial->write(str.c_str(), str.size());
//		std::cout << get_cur_1ms() << " written: " << str << std::endl;
//	}

	if (get_cur_1ms() - LastWriteTime > 500)
	{
		SendHeader(AP_PROT_SENSORDATA, sizeof(AutoPilotMsgSensorData));
		AutoPilotMsgSensorData data;
		data.FlyState = AP_PROT_FLY_STATE_FLYING;
		data.GPSState = 2;
		data.BatteryLeft = 3;
		data.ServoState = 4;
		data.AutoPilotState = 5;
		data.SensorState = 6;
		data.Position.X = 7;
		data.Position.Y = 8;
		data.Position.Z = 9;
		data.GroundSpeed = 10;
		data.VerticalSpeed = 11;
		data.Heading = 12;
		data.Yaw = 13;
		data.Pitch = 14;
		data.Roll = 15;
		data.WindHeading = 16;
		data.WindSpeed = 17;
		SendData((char*)&data, sizeof(AutoPilotMsgSensorData));
	}

	usleep(config.TickTime);
}


bool CTests::SynchronizeUart(AutoPilotMsgHeader& msgHdr)
{
	if (Serial->available() < 2*sizeof(AutoPilotMsgHeader))
		return false;

	char chr;
	AutoPilotMsgHeaderType header;
	Serial->read((char*)&header, sizeof(AutoPilotMsgHeaderType));
	while (Serial->available() >= sizeof(AutoPilotMsgHeader))
	{
		//printf("header=%04X\n", header);
		//std::cout << "header=" << +header << std::endl;
		if (header == AP_PROT_HEADER)
		{
			msgHdr.Header = header;
			Serial->read((char*)&msgHdr+sizeof(AutoPilotMsgHeaderType), sizeof(AutoPilotMsgHeader)-sizeof(AutoPilotMsgHeaderType));
			Synchronize = false;
			LastReadHeaderUsed = false;
			return true;
		}
		Serial->read(&chr, 1);
		header = (header << 8) | chr;
	}
	std::cout << "Failed to sync" << std::endl;
	return false;
}


void CTests::ReadUart()
{
//	if (Serial->available())
//	{
//		std::vector<char> msg = Serial->read();
//		//for (std::vector<char>::reverse_iterator it=msg.rbegin(); it != msg.rend(); ++it)
//		for (std::vector<char>::iterator it=msg.begin(); it != msg.end(); ++it)
//			printf("%X ", (uint8_t)*it);
//		printf("\n");
//	}
//	return;

	//std::cout << get_cur_1ms() << " Read UART " << Serial->available() << std::endl;

	// Check if we need to synchronize (also reads a header)
	if (Synchronize)
	{
		if (!SynchronizeUart(LastReadHeader))
			return;
		std::cout << "Synched, " << LastReadHeader << std::endl;
	}

	// Check if we need to read a new header
	if (LastReadHeaderUsed)
	{
		if (Serial->available() >= sizeof(AutoPilotMsgHeader))
		{
			Serial->read((char*)&LastReadHeader, sizeof(AutoPilotMsgHeader));
			if (LastReadHeader.Header != AP_PROT_HEADER)
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
	if (Serial->available() < LastReadHeader.DataSize + sizeof(CheckSumOut))
		return;
	LastReadHeaderUsed = true;

	switch (LastReadHeader.MsgType)
	{
	case AP_PROT_SET_MODE:
	{
		AutoPilotMsgMode data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgMode)))
			return;
		std::cout << "Setting mode:" << data << std::endl;
		break;
	}
//	case AP_PROT_SET_FIELD:
//	{
//		AutoPilotMsgField data;
//		if (!ReadData((char*)&data, sizeof(AutoPilotMsgField)))
//			return;
//		std::cout << "Setting field:" << data << std::endl;
//		break;
//	}
	case AP_PROT_SET_WAYPOINTS:
	{
		AutoPilotMsgWayPoints data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgWayPoints)))
			return;
		std::cout << "Setting waypoints:" << data << std::endl;
		break;
	}
	case AP_PROT_REQ_SENSORDATA:
	{
		if (!ReadData((char*)NULL, 0))
			return;
		std::cout << "Received sensor data request" << std::endl;

		SendHeader(AP_PROT_SENSORDATA, sizeof(AutoPilotMsgSensorData));
		AutoPilotMsgSensorData data;
		data.FlyState = AP_PROT_FLY_STATE_FLYING;
		data.GPSState = 2;
		data.BatteryLeft = 3;
		data.ServoState = 4;
		data.AutoPilotState = 5;
		data.SensorState = 6;
		data.Position.X = 7;
		data.Position.Y = 8;
		data.Position.Z = 9;
		data.GroundSpeed = 10;
		data.VerticalSpeed = 11;
		data.Heading = 12;
		data.Yaw = 13;
		data.Pitch = 14;
		data.Roll = 15;
		data.WindHeading = 16;
		data.WindSpeed = 17;

		SendData((char*)&data, sizeof(AutoPilotMsgSensorData));
//		Serial->write((char*)&data, sizeof(AutoPilotMsgSensorData));
//		for (int i=0; i<sizeof(AutoPilotMsgSensorData); ++i)
//			CheckSumOut+=((char*)&data)[i];
//		Serial->write(&CheckSumOut, sizeof(CheckSumOut));
		break;
	}
	case AP_PROT_REQ_WP_STATUS:
	{
		if (!ReadData((char*)NULL, 0))
			return;
		std::cout << "Received wp status request" << std::endl;

		SendHeader(AP_PROT_WP_STATUS, sizeof(AutoPilotMsgWpStatus));
		AutoPilotMsgWpStatus data;
		data.NumWaypoints = AP_PROT_WAYPOINTS_MAX-1;
		for (int i=0; i<data.NumWaypoints; ++i)
		{
			data.ID[i] = 2*i;
			data.ETA[i] = 2*i+1;
		}
		SendData((char*)&data, sizeof(AutoPilotMsgWpStatus));
//		Serial->write((char*)&data, sizeof(AutoPilotMsgWpStatus));
//		for (int i=0; i<sizeof(AutoPilotMsgWpStatus); ++i)
//			CheckSumOut+=((char*)&data)[i];
//		Serial->write(&CheckSumOut, sizeof(CheckSumOut));
		break;
	}
	case AP_PROT_REQ_WP_BOUNDS:
	{
		if (!ReadData((char*)NULL, 0))
			return;
		std::cout << "Received sensordata request" << std::endl;

		SendHeader(AP_PROT_WP_BOUNDS, sizeof(AutoPilotMsgWpBounds));
		AutoPilotMsgWpBounds data;
		for (int i=0; i<AP_PROT_DIRECTIONS; ++i)
		{
			data.MinRadius[i] = 3*i;
			data.MinSpeed[i] = 3*i+1;
			data.MaxSpeed[i] = 3*i+2;
		}
		SendData((char*)&data, sizeof(AutoPilotMsgWpBounds));
//		Serial->write((char*)&data, sizeof(AutoPilotMsgWpBounds));
//		for (int i=0; i<sizeof(AutoPilotMsgWpBounds); ++i)
//			CheckSumOut+=((char*)&data)[i];
//		Serial->write(&CheckSumOut, sizeof(CheckSumOut));
		break;
	}
	case AP_PROT_REQ_XBEE_MSG:
	{
		AutoPilotMsgXBeeMsgReq data;
		if (!ReadData((char*)&data, sizeof(AutoPilotMsgXBeeMsgReq)))
			return;
		std::cout << "Received xbee msg request:" << data << std::endl;
		break;
	}
	default:
	{
		Synchronize = true;
		std::cout << "Received unknown msg type from gumstix: " << LastReadHeader << std::endl;
		break;
	}
	}
}

bool CTests::ReadData(char* data, size_t size)
{
	if (size != LastReadHeader.DataSize)
	{
		std::cout << "Error datasize: expected=" << size << " DataSize=" << +LastReadHeader.DataSize << std::endl;
		Synchronize = true;
		return false;
	}

	if (size > 0)
		Serial->read(data, size);
	char checkSum1;
	Serial->read(&checkSum1, 1);


	printf("Read:");
	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		printf(" %X", (uint8_t)(((char*)&LastReadHeader)[i]));
	for (int i=0; i<size; ++i)
		printf(" %X", (uint8_t)(data[i]));
	printf(" %X", (uint8_t)checkSum1);
	printf("\n");

	char checkSum2 = 0;
	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		checkSum2 += ((char*)&LastReadHeader)[i];
	for (int i=0; i<size; ++i)
		checkSum2 += data[i];
	//std::cout << "Checksums: " << +checkSum1 << " vs " << +checkSum2 << std::endl;
	if (checkSum1 != checkSum2)
	{
		std::cout << "Error checksum: " << +checkSum1 << " vs " << +checkSum2 << std::endl;
		return false;
	}
	return true;
}


bool CTests::SendHeader(EAutoPilotMsgType type, uint8_t dataSize)
{
	CheckSumOut = 0;
	AutoPilotMsgHeader msgHdr;
	msgHdr.Header = AP_PROT_HEADER;
	msgHdr.MsgType = type;
	msgHdr.TimeStamp = get_cur_1ms();
	msgHdr.DataSize = dataSize;

//	std::cout << "Writing header: " << msgHdr << std::endl;
	try
	{
		Serial->write((char*)&msgHdr, sizeof(AutoPilotMsgHeader));
	}
	catch (boost::system::system_error& e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		delete Serial;
		throw;
	}

	for (int i=0; i<sizeof(AutoPilotMsgHeader); ++i)
		CheckSumOut+=((char*)&msgHdr)[i];

	//std::cout << "CheckSum=" << +CheckSumOut << std::endl;
	if (dataSize == 0)
		Serial->write(&CheckSumOut, sizeof(CheckSumOut));

	return true;
}

bool CTests::SendData(char* data, size_t size)
{
	for (int i=0; i<size; ++i)
		CheckSumOut+=data[i];
	Serial->write(data, size);
	Serial->write(&CheckSumOut, sizeof(CheckSumOut));

//	printf("Written:");
//	for (int i=0; i<size; ++i)
//		printf(" %X", ((uint8_t*)data)[i]);
//	printf(" %X", (uint8_t)CheckSumOut);
//	printf("\n");
	return true;
}

