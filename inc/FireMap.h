/**
 * @brief 
 * @file FireMap.h
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
 * @date          Dec 7, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef FIREMAP_H_
#define FIREMAP_H_

#include "MapFunctions.h"

enum EFireSource
{
	FIRE_SRC_TPA=0,
	FIRE_SRC_CAM,
	FIRE_SRC_CO,
	FIRE_SRC_TPA_FRONT,
	FIRE_SRC_CAM_FRONT,
	FIRE_SRC_NUM
};

class MapFireStruct : public Gaussian2D
{
public:
	//Gaussian2D Gaussian;
	float Probability[FIRE_SRC_NUM];
	bool Seen;
	bool Sent;
	//int bla;

	MapFireStruct(Position& center, float amplitude, float sigmaX, float sigmaY, float rotation):
		Gaussian2D::Gaussian2D(center, amplitude, sigmaX, sigmaY, rotation)
		//bla(0)
	{
		//Probability[0] = 0;
	}

};


#endif /* FIREMAP_H_ */
