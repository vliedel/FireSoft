/**
 * @brief Generic structures
 * @file Fitness.h
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

#ifndef FITNESS_H_
#define FITNESS_H_

#include "MapFunctions.h"
//#include <climits>
#include <cfloat>


enum FitnessSource {
	FITSRC_COLLISION=0,
	FITSRC_BATTERY,
	FITSRC_EDGE,
	FITSRC_CONNECTIVITY,
	FITSRC_FORWARDFIRE,
	FITSRC_DOWNWARDFIRE,
	FITSRC_COVERAGE,
	FITSRC_STATIC,
	FITSRC_NUM
};

//#define FIT_CRIT_LEVEL 1.0 // If critical fitness (like collision) is above this level, then we can ignore other fitness
//#define FIT_COL_AMPLITUDE 5.0 // Influence reaches about 2 sigma around gaussian (there the fitness is 1)
//#define FIT_COL_SIGMA_HOR 10.0
//#define FIT_COL_SIGMA_VERT 5.0

//#define FIT_COV_AMPLITUDE 1.0
//#define FIT_COV_SIGMA_X 10.0 // Aligned with heading
//#define FIT_COV_SIGMA_Y 10.0

struct FitnessSums {
	float Sums[FITSRC_NUM];
};
//static const struct FitnessSums FitnessSumsZero;

struct Fitness {
	float fitness;
	FitnessSource source;
};

class FitnessGaussian2D : public Gaussian2D
{
	public:
		// Data
		int ID;
		FitnessSource Source;
		float MaxVal;
		float MinVal;

		// Constructors
		FitnessGaussian2D(int id, FitnessSource source, Position& center, float amplitude, float sigmaX, float sigmaY, float rotation, float minVal=0, float maxVal=0):
			Gaussian2D::Gaussian2D(center, amplitude, sigmaX, sigmaY, rotation),
			ID(id),
			Source(source)
		{
			MinVal = (minVal != 0) ? minVal : FLT_MIN;
			MaxVal = (maxVal != 0) ? maxVal : FLT_MAX;
		}

		// Functions
		float GetValue(const Position& pos)
		{
			float val = Gaussian2D::GetValue(pos);
			if (Amplitude > 0)
			{
				if (val > MaxVal)
					return MaxVal;
				if (val < MinVal)
					return 0;
			}
			else
			{
				if (val < -MaxVal)
					return -MaxVal;
				if (val > -MinVal)
					return 0;
			}
			return val;
		}

		friend std::ostream& operator<<(std::ostream& os, const FitnessGaussian2D& struc)
		{
			os << static_cast<const Gaussian2D&>(struc);
			os << "ID=" << struc.ID << " Source=" << struc.Source << " MinVal=" << struc.MinVal << " MaxVal=" << struc.MaxVal;
			return os;
		}
};

class FitnessGaussian3D : public Gaussian3D
{
	public:
		// Data
		int ID;
		FitnessSource Source;
		float MaxVal;
		float MinVal;

		// Constructors
		FitnessGaussian3D(int id, FitnessSource source, Position& center, float amplitude, float sigmaX, float sigmaY, float sigmaZ, float rotation, float minVal=0, float maxVal=0):
			Gaussian3D::Gaussian3D(center, amplitude, sigmaX, sigmaY, sigmaZ, rotation),
			ID(id),
			Source(source)
		{
			MinVal = (minVal != 0) ? minVal : FLT_MIN;
			MaxVal = (maxVal != 0) ? maxVal : FLT_MAX;
		}

		// Functions
		float GetValue(const Position& pos)
		{
			float val = Gaussian3D::GetValue(pos);
			if (Amplitude > 0)
			{
				if (val > MaxVal)
					return MaxVal;
				if (val < MinVal)
					return 0;
			}
			else
			{
				if (val < -MaxVal)
					return -MaxVal;
				if (val > -MinVal)
					return 0;
			}
			return val;
		}

		friend std::ostream& operator<<(std::ostream& os, const FitnessGaussian3D& struc)
		{
			os << static_cast<const Gaussian3D&>(struc);
			os << "ID=" << struc.ID << " Source=" << struc.Source << " MinVal=" << struc.MinVal << " MaxVal=" << struc.MaxVal;
			return os;
		}
};

class FitnessQuadraticWall : public QuadraticWall
{
public:
	// Data
	int ID;
	FitnessSource Source;

	FitnessQuadraticWall(int id, FitnessSource src, Position& p0, Position& p1, float amplitude):
		QuadraticWall::QuadraticWall(p0, p1, amplitude),
		ID(id),
		Source(src)
	{}
};


class FitnessPlane : public Plane
{
public:
	int ID;
	FitnessSource Source;

	FitnessPlane(int id, FitnessSource src, Position& origin, Position& x, Position& y):
		Plane(origin, x, y),
		ID(id),
		Source(src)
	{}

};


#endif // FITNESS_H_
