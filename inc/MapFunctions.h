/**
 * @brief 
 * @file MapFunctions.h
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

#ifndef MAPFUNCTIONS_H_
#define MAPFUNCTIONS_H_

#include "Geometry.h"
#include <cmath>

class Gaussian2D
{
	public:
		// Data
		Position Center;
		float Amplitude;
		float A;
		float B;
		float C;

		// Constructors
//		Gaussian2D(Position& center, float amplitude, float a, float b, float c):
//			Center(center),
//			A(a),
//			B(b),
//			C(c),
//			Amplitude(amplitude)
//		{
//		}

		Gaussian2D(Position& center, float amplitude, float sigmaX, float sigmaY, float rotation):
			Center(center),
			Amplitude(amplitude)
		{
			// From http://en.wikipedia.org/wiki/Gaussian_function#Meaning_of_parameters_for_the_general_equation
			double cosine = std::cos(rotation);
			double sinus = std::sin(rotation);
			double sinus2 = std::sin(2*rotation);
			double sx2 = sigmaX*sigmaX;
			double sy2 = sigmaY*sigmaY;
			A = cosine*cosine / (2*sx2) + sinus*sinus / (2*sy2);
			B = sinus2 / (-4*sx2) + sinus2 / (4*sy2);
			C = cosine*cosine / (2*sy2) + sinus*sinus / (2*sx2);
		}

		~Gaussian2D() {}


		// Functions
		float GetValue(const Position& pos)
		{
			//A*exp( - (a*(X-x0).^2 + 2*b*(X-x0).*(Y-y0) + c*(Y-y0).^2)) ;
			float dx = pos.x() - Center.x();
			float dy = pos.y() - Center.y();
			return Amplitude * std::exp( -(A*dx*dx + 2*B*dx*dy + C*dy*dy));
		}

		friend std::ostream& operator<<(std::ostream& os, const Gaussian2D& struc)
		{
			os << "Center=" << struc.Center.transpose() << " amplitude=" << struc.Amplitude
					<< " A=" << struc.A << " B=" << struc.B << " C=" << struc.C;
			return os;
		}
};


class Gaussian3D : public Gaussian2D
{
	public:
		// Data
		float D;

		Gaussian3D(Position& center, float amplitude, float sigmaX, float sigmaY, float sigmaZ, float rotation):
			Gaussian2D::Gaussian2D(center, amplitude, sigmaX, sigmaY, rotation)
		{
			D = 1.0/sigmaZ/sigmaZ;
		}

		~Gaussian3D() {}


		// Functions
		float GetValue(const Position& pos)
		{
			// f = A*exp(-D*(z - z0)^2)
			float dz = pos.z() - Center.z();

			// We can simply multiply the 2d gaussian by a 1d gaussian since sigma_zx and sigma_zy are 0 (we only rotate around the z axis)
			return Gaussian2D::GetValue(pos) * std::exp(-D*dz*dz);
		}

		friend std::ostream& operator<<(std::ostream& os, const Gaussian3D& struc)
		{
			os << static_cast<const Gaussian2D&>(struc);
			os << " D=" << struc.D;
			return os;
		}
};


// To make this comparable with a gaussian: set the points at 3*sigma away from the center
// and set the amplitude to 1/(3*sigma)^2
// Then values are about y(1sig)=0.11A, y(2sig)=0.44A, y(3sig)=1A
// The order of StartPoints matters (one side will be 0, the other side rises quadratically)
class QuadraticWall
{
public:
	Position StartPoint[2];
	float Amplitude;

	QuadraticWall(Position& p0, Position& p1, float amplitude): Amplitude(amplitude) { StartPoint[0]=p0; StartPoint[1]=p1; }

	// Functions
	float GetValue(const Position& pos)
	{
		Position p(StartPoint[0]);
		p.x() += StartPoint[1].y() - StartPoint[0].y();
		p.y() += StartPoint[0].x() - StartPoint[1].x();
		Position q(pos);
		q.z() = p.z();
		// vector p-StartPoint[0] is now orthogonal to StartPoint[1] - StartPoint[0]
		float d = (p-StartPoint[0]).dot(q-StartPoint[0]) / (p-StartPoint[0]).norm();
		//std::cout << " p=[" << p.transpose() << "] q=[" << q.transpose() << "] d=" << d << " ";
		if (d<0)
			return 0;
		return d*d*Amplitude;
	}
};

// Plane defined by 3 points, z is the value
class Plane
{
public:
	Eigen::Vector2f Zero; // Position where x=y=0
	Eigen::Matrix2f Inv;
	float Ax, Ay, Offset;

	Plane(Position& origin, Position& x, Position& y)
	{
		Ax=x.z()-origin.z();
		Ay=y.z()-origin.z();
		Offset=origin.z();
		Zero.x() = origin.x();
		Zero.y() = origin.y();
		Eigen::Matrix2f dXY;
		dXY(0,0) = x.x()-Zero.x();	dXY(1,0) = x.y()-Zero.y();
		dXY(0,1) = y.x()-Zero.x();	dXY(1,1) = y.y()-Zero.y();
		Inv = dXY.inverse();
	}

	float GetValue(const Position& pos)
	{
		// Solve: [dXY]*a = pos-O
		// Solution: a = inv(dXY) * (pos-O)
		Eigen::Vector2f dP = -Zero;
		dP.x()+=pos.x();
		dP.y()+=pos.y();
		Eigen::Vector2f a = Inv*dP;
		return a.x()*Ax + a.y()*Ay + Offset;
	}
};

#endif /* MAPFUNCTIONS_H_ */
