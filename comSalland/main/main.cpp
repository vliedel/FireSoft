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

#include "CComSalland.h"
#include <csignal>
//#include <cstdlib>
//#include <unistd.h>

using namespace rur;

CComSalland* ComSalland;

// Function to handle termination signals
void SignalHandler(int signum)
{
	// Cleanup and close up stuff here
	if (ComSalland != NULL)
		ComSalland->Close();
	delete ComSalland;

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
	ComSalland = new CComSalland;
	std::string identifier = argv[1];
	ComSalland->Init(identifier);

	// Run the module
	do {
		ComSalland->Tick();
	} while (true);

	// We never get here :)
	ComSalland->Close();
	delete ComSalland;
	return EXIT_SUCCESS;
}
