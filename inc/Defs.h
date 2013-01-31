/**
 * @brief 
 * @file Typedefs.h
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
 * @date          Sep 18, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <vector>
#include <cmath>

typedef std::vector<float> VecMsgType;

#define GRAVITY 9.81
#define CRUISE_SPEED 22.0
#define ROLLANGLE_MAX 30.0/180*M_PI
#define VERT_SPEED_MAX 3.0



#endif /* TYPEDEFS_H_ */
