/**
 * @brief 
 * @file FitnessMap.h
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
 * @date          Oct 16, 2012
 * @project       
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef FITNESSMAP_H_
#define FITNESSMAP_H_

#include "Fitness.h"
#include "ShMemTypedefs.h"

class FitnessMapType
{
	public:
		// Data
		FitnessMapXType MapX;
		MapVoidAllocatorType VoidAllocator;

		// Constructors
		FitnessMapType(const MapVoidAllocatorType &voidAllocator) :
			MapX(std::less<FitnessMapKeyType>(), voidAllocator),
			VoidAllocator(voidAllocator)
		{}

		// Functions
		void AddGaussian(FitnessGaussian2D &gaussian)
		{
			int x = gaussian.Center.x();
			int y = gaussian.Center.y();

			FitnessMapIterXType itX = MapX.find(x);
			if (itX == MapX.end())
			{
				FitnessMapYType mapY(std::less<FitnessMapKeyType>(), VoidAllocator);
				mapY.insert(FitnessMapValueYType(y, gaussian));
				MapX.insert(FitnessMapValueXType(x, mapY));
				std::cout << "Added gaussian at pos [" << x << " " << y << "]" << std::endl;
			}
			else
			{
				FitnessMapIterYType itY;
				itY = itX->second.find(y);
				if (itY == itX->second.end())
				{
					itX->second.insert(FitnessMapValueYType(y, gaussian));
					std::cout << "Added gaussian at pos [" << x << " " << y << "]" << std::endl;
				}
				else
				{
					// What else?
				}
			}
		}

		void Get(std::vector<FitnessGaussian2D> &result, float minX, float maxX, float minY, float maxY)
		{
			FitnessMapIterXType itX;
			FitnessMapIterYType itY;
			for (itX = MapX.lower_bound(minX); itX != MapX.upper_bound(maxX); ++itX)
			{
				for (itY=itX->second.lower_bound(minY); itY != itX->second.upper_bound(maxY); ++itY)
				{
					result.push_back(itY->second);
				}
			}
		}

		float GetValue(const Position& pos, float minX, float maxX, float minY, float maxY, float maxVal=FLT_MAX)
		{
			float fit = 0;
			FitnessMapIterXType itX;
			FitnessMapIterYType itY;
			for (itX = MapX.lower_bound(minX); itX != MapX.upper_bound(maxX); ++itX)
			{
				if (itX == MapX.end())
					std::cout << "Error: itX = MapX.end()   size=" << MapX.size();
				for (itY=itX->second.lower_bound(minY); itY != itX->second.upper_bound(maxY); ++itY)
				{
					fit += itY->second.GetValue(pos);
					if (fit > maxVal)
						return maxVal;
				}
			}
			return fit;
		}
};


#endif /* FITNESSMAP_H_ */
