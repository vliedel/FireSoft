/**
 * @brief 
 * @file test.cpp
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
 * @date          Aug 28, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */


#include "CAutoPilotSim.h"
#include <cstdlib>
#include <fstream>

using namespace rur;

int main(int argc, char *argv[])
{
	//if (argc < 2) {
	//	printf("Use an identifier as argument for this instance\n");
	//	return EXIT_FAILURE;
	//}
	CAutoPilotSim* autoPilotSim = new CAutoPilotSim;
	//std::string identifier = argv[1];
	//autoPilotSim->Init(identifier);
	autoPilotSim->Init("1");
	std::ofstream file;
	file.open("output");


	WayPoint wp;
	wp.to.x() = 0;
	wp.to.y() = 400;
	wp.to.z() = 20;
	autoPilotSim->AddWayPoint(wp);
	for (int i=0; i<150; ++i)
	{
		autoPilotSim->TimeStep(0.1);
		file << autoPilotSim->Geom.Pos.x() << " ";
		file << autoPilotSim->Geom.Pos.y() << " ";
		file << autoPilotSim->Geom.Pos.z() << " ";
		file << autoPilotSim->Heading << " ";
		//file << autoPilotSim->RollAngle.angle();
		file << std::endl;
		std::cout << "----- time step -----" << std::endl;
		std::cout << *autoPilotSim << std::endl;
	}
	//do {
	//	autoPilotSim->Tick();
	//} while (true);

	autoPilotSim->Close();
	delete autoPilotSim;

	return EXIT_SUCCESS;
}
