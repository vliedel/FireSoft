/**
 * @brief 
 * @file CTemplateModule.cpp
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

#include "CTemplateModule.h"
#include "Protocol.h"

using namespace rur;

CTemplateModule::~CTemplateModule()
{

}

void CTemplateModule::Init(std::string module_id)
{
	templateModule::Init(module_id);
	config.load("config.json");
}

void CTemplateModule::Tick()
{
	templateModule::Tick();
	usleep(config.TickTime);
}
