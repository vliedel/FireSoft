/**
 * @brief 
 * @file main.cpp
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

//#define SIM_TIME_STEP 0.1
//#define SIM_RADIO_ROUND_TIME 0.25
//#define SIM_RADIO_DIST 1000

#include "CSim.h"
//#include <cstdlib>
//#include <unistd.h>

using namespace rur;

int main(int argc, char *argv[])
{
	if (argc < 5) {
		printf("Usage: %s <module_id> <number_of_UAVs_with_sim_autopilot> <number_of_UAVs_with_sim_radio> <seconds_to_simulate>\n", argv[0]);
		return EXIT_FAILURE;
	}

	CSim* sim = new CSim;
	std::string identifier = argv[1];
	sim->Init(identifier, atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); // Maybe use strtol() instead?

	do {
		sim->Tick();
//		usleep(1000000 * SIM_TIME_STEP / 1000); // 1000 ticks per simulated second, should make it possible to simulate 1000 * real time, if not for the radio acks
		//usleep(1000000 * SIM_TIME_STEP / 100); // 100 ticks per simulated second, should make it possible to simulate 100 * real time, if not for the radio acks
//		usleep(1*1000); // 1000 micro seconds to respond
	} while (true);

	sim->Close();
	delete sim;

	return EXIT_SUCCESS;
}
