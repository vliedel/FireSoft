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

CRadio::~CRadio()
{
}

void CRadio::Tick()
{
//	std::vector<int>* pMsg = readToBuffer(false);
//	if (pMsg != 0)
//	{
//		//sendBuffer.push_back(*pMsg);
//		//std::vector<int> msg = *p;
//		std::vector<int>::iterator it;
//		printf("msg: ");
//		for (it = pMsg->begin(); it != pMsg->end(); ++it)
//		{
//			printf("%i ", *it);
//		}
//		printf("\n");
//	}
//
//	//if ()
//	std::vector<int> bla;
//	bla.push_back(1);
//	bla.push_back(2);
//	writeFromBuffer(bla);

}

void CRadio::ReadFromRadio()
{

}

void CRadio::WriteToRadio()
{

}

void CRadio::UpdateState()
{

}
