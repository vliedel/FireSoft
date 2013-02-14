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

#include "Fire.h"
#include "ShMemTypedefs.h"

class MapFireType
{
public:
	// Data
	FireMapXType MapX;
	MapVoidAllocatorType VoidAllocator;

	// Constructors
	MapFireType(const MapVoidAllocatorType &voidAllocator) :
		MapX(std::less<FireMapKeyType>(), voidAllocator),
		VoidAllocator(voidAllocator)
	{}

	// Functions
	void AddGaussian(MapFireStruct &fire)
	{
		int x = fire.Fire.Center.x();
		int y = fire.Fire.Center.y();

		FireMapIterXType itX = MapX.find(x);
		if (itX == MapX.end())
		{
			FireMapYType mapY(std::less<FireMapKeyType>(), VoidAllocator);
			mapY.insert(FireMapValueYType(y, fire));
			MapX.insert(FireMapValueXType(x, mapY));
			std::cout << "Added fire at pos [" << x << " " << y << "]" << std::endl;
		}
		else
		{
			FireMapIterYType itY;
			itY = itX->second.find(y);
			if (itY == itX->second.end())
			{
				itX->second.insert(FireMapValueYType(y, fire));
				std::cout << "Added fire at pos [" << x << " " << y << "]" << std::endl;
			}
			else
			{
				// Update fire!
				if ((itY->second.Fire.Amplitude == fire.Fire.Amplitude)
						&& (itY->second.Fire.Probability[FIRE_SRC_CAM] == fire.Fire.Probability[FIRE_SRC_CAM])
						&& (itY->second.Fire.Probability[FIRE_SRC_CAM] == fire.Fire.Probability[FIRE_SRC_TPA])
						&& (itY->second.Fire.Probability[FIRE_SRC_CAM] == fire.Fire.Probability[FIRE_SRC_CO]))
				{
					// Ignore?
				}
				else
				{
					// Update
					itY->second = fire;
				}
			}
		}
	}

	void Get(std::vector<MapFireStruct> &result, float minX, float maxX, float minY, float maxY)
	{
		FireMapIterXType itX;
		FireMapIterYType itY;
		for (itX = MapX.lower_bound(minX); itX != MapX.upper_bound(maxX); ++itX)
		{
			for (itY=itX->second.lower_bound(minY); itY != itX->second.upper_bound(maxY); ++itY)
			{
				result.push_back(itY->second);
			}
		}
	}

};


#endif /* FIREMAP_H_ */
