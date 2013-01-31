/**
 * @brief Generic structures
 * @file Geometry.h
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

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

//#include <cstdio>
#include <iostream>

#define EIGEN_DONT_ALIGN
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include <Eigen/Geometry>

typedef Eigen::Vector3f Position;
//typedef Eigen::Vector3f SpeedType;
typedef float SpeedType;
//typedef Eigen::Quaternionf Rotation;

class Rotation : public Eigen::Quaternionf
{
	friend std::ostream& operator<<(std::ostream& os, const Rotation& rot)
	{
		os << "[" << rot.w() << " " << rot.x() << " " << rot.y() << " " << rot.z() << "]";
		return os;
	}
};

typedef Eigen::Matrix3f RotationMatrix;

typedef Eigen::Rotation2D<float> Rotation2DType;
//class Rotation2DType : private Eigen::Rotation2Df
//{
//	Rotation2DType(float angle) { this->m_angle(0); }
//	friend std::ostream& operator<<(std::ostream& os, const Rotation2DType& rot)
//	{
//		os << rot.angle();
//		return os;
//	}
//};


/*
class Position {
	public:
		int x;
		int y;
		int z;

		friend std::ostream& operator<<(std::ostream& os, const Position& pos)
		{
			os << "[" << pos.x << " " << pos.y << " " << pos.z << "]";
			return os;
		}

};
*/

/*
class Speed {
	public:
		Position dPos;
		int dt; // Do we need this?

		friend std::ostream& operator<<(std::ostream& os, const Speed& speed)
		{
			os << speed.dPos << " in " << speed.dt << "s";
			return os;
		}
};
*/

/*
class Rotation {
	public:
		float r[4];

		friend std::ostream& operator<<(std::ostream& os, const Rotation& rot)
		{
			os << "[" << rot.r[0] << " " << rot.r[1] << " " << rot.r[2] << " " << rot.r[3] << "]";
			return os;
		}

};
*/

/*
class RotationMatrix {
	public:
		float r[9];

		friend std::ostream& operator<<(std::ostream& os, const RotationMatrix& rot)
		{
			os << "[";
			os << rot.r[0] << " " << rot.r[1] << " " << rot.r[2] << "; ";
			os << rot.r[3] << " " << rot.r[4] << " " << rot.r[5] << "; ";
			os << rot.r[6] << " " << rot.r[7] << " " << rot.r[8] << "]";
			return os;
		}
};
*/

#endif // GEOMETRY_H_
