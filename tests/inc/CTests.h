/**
 * @brief 
 * @file CTests.h
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

#ifndef CTESTS_H_
#define CTESTS_H_

#include "tests.h"
#include "StructToFromCont.h"
#include "Print.hpp"

#include "AutoPilotProt.h"
#include "BufferedAsyncSerial.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

#define BLA(n) \
	unsigned _##n##_a : 2; \
	signed _##n##_b : 3;

#define FOO(n) \
	unsigned _##n##_c : 1; \
	unsigned _##n##_d : 2;

#define BLA_GET(struc, n, var) struc._##n##_##var

#pragma pack(1)

struct BitTest
{
	BLA(0)
	BLA(1)
	BLA(2)
	BLA(3)
	FOO(4)
	FOO(5)
	FOO(6)
	FOO(7)
};


#pragma pack()

struct TestConfig
{
	long TickTime;
	bool Debug;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("test.TickTime");
		Debug = pt.get<bool>("test.Debug");
//		BOOST_FOREACH(ptree::value_type &v, pt.get_child("debug.modules"))
//			m_modules.insert(v.second.data());
	}

	void save(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		pt.put("test.TickTime", TickTime);
		pt.put("test.Debug", Debug);
//		BOOST_FOREACH(const std::string &name, m_modules)
//			pt.add("debug.modules.module", name);
		write_json(filename, pt);
	}

};

class CTests : public tests
{
	private:
		BufferedAsyncSerial *Serial;
		bool Synchronize;
		char CheckSumOut;	// Sum of bytes of whole msg written
		AutoPilotMsgHeader LastReadHeader;
		bool LastReadHeaderUsed; // True when new header should be read, false if we should use LastReadHeader
		long LastWriteTime;
		TestConfig config;
		bool SynchronizeUart(AutoPilotMsgHeader& msgHdr);
		void ReadUart();
		bool ReadData(char* data, size_t size);
		bool SendHeader(EAutoPilotMsgType type, uint8_t dataSize);
		bool SendData(char* data, size_t size);

	public:
		~CTests();
		void Init(std::string module_id);
		void Tick();

};

template<class T>
static void Check(T& struc, std::vector<int>& vec, std::vector<int>& vec2, std::string str)
{
	std::cout << "\nChecking " << str << " size=" << sizeof(T) << std::endl;
	std::vector<int>::iterator iter = FromCont(struc, vec.begin(), vec.end());
	dobots::print(vec.begin(), vec.end());
	if (iter != vec.end())
		std::cout << ">>>> ERROR <<<< in vector to " << str << " " << iter-vec.end() << std::endl;
	else
	{
		std::cout << struc << std::endl;
		vec2.clear();
		ToCont(struc, vec2);
		if (vec != vec2)
		{
			std::cout << ">>>> ERROR <<<< in " << str << " to vector" << std::endl;
			dobots::print(vec2.begin(), vec2.end());
		}
		else
			std::cout << str << " OK :D" << std::endl;
	}
}

}
#endif // CTESTS_H_
