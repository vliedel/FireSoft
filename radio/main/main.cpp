/**
 * @brief Radio program: can receive and send messages via the radio.
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
 * @date          Apr 24, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */


#include "CRadio.h"
#include <csignal>
//#include <cstdlib>
//#include <unistd.h>

using namespace rur;

CRadio* Radio;

// Function to handle termination signals
void SignalHandler(int signum)
{
	// Cleanup and close up stuff here
	if (Radio != NULL)
		Radio->Close();
	delete Radio;

	// Terminate program
	exit(signum);
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("Use an identifier as argument for this instance\n");
		return EXIT_FAILURE;
	}

	// Register signal and signal handler
	signal(SIGINT, SignalHandler);
	signal(SIGTERM, SignalHandler);

	// Initialize the module
	Radio = new CRadio;
	std::string identifier = argv[1];
	Radio->Init(identifier);

	// Run the module
	do {
		Radio->Tick();
	} while (true);

	// We never get here :)
	Radio->Close();
	delete Radio;
	return EXIT_SUCCESS;
}
