/**
 * @brief Generic structures
 * @file Waypoint.h
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

#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include "Geometry.h"
#include <iostream>
#include "Defs.h"
#include <cfloat>

//#define WP_FREE_REACH_TIME 0.5 // If waypoint is 0.5 seconds ahead, it's considered reached
//#define WP_LINE_AHEAD_TIME 5 // How far ahead the carrot should be on the line

enum WayPointMode {
	WP_LINE=0,
	WP_FREE,
	WP_CIRCLE,
	WP_ARC,
	WP_INVALID
};

enum VerticalMode {
	VERTMODE_ALT=0,
	VERTMODE_THROTTLE,
	VERTMODE_CLIMB,
	VERTMODE_PITCH,
	VERTMODE_NUMMODES
};

struct WayPoint {
	WayPointMode wpMode;
	Position from;
	Position to; // Also used as circle/arc center
	float Radius;
	float AngleStart; // 0 to 2PI
	float AngleArc; // -2PI to 2PI
	float VerticalSpeed;
	float ETA;
	//uint32_t ID;
	int ID;

//	VerticalMode vMode;
//	int altitude;
//	int pitch;
//	float throttle;
//	float climbRate;


	// Inits data with default value (mostly zeros)
	void Init()
	{
		wpMode = WP_LINE;
		from << 0,0,0;
		to << 0,0,0;
		Radius = 0;
		AngleStart = 0;
		AngleArc = 0;
		VerticalSpeed = 0;
		ETA = 0;
		ID = 0;
	}

	void GetStartPos(Position& startPos)
	{
		switch (wpMode)
		{
			case WP_LINE:
			{
				startPos = from;
				break;
			}
			case WP_FREE:
			{
				// TODO: what to do here?
				break;
			}
			case WP_CIRCLE:
			{
				// TODO: what to do here?
				startPos = to;
				//startPos.x() += Radius;
				break;
			}
			case WP_ARC:
			{
				startPos = to;
				startPos.x() += Radius*cos(AngleStart);
				startPos.y() += Radius*sin(AngleStart);
				break;
			}
		}
	}

	void GetEndPos(Position& endPos)
	{
		switch (wpMode)
		{
			case WP_LINE:
			{
				endPos = to;
				break;
			}
			case WP_FREE:
			{
				endPos = to;
				break;
			}
			case WP_CIRCLE:
			{
				// uhhhh
				endPos = to;
				//startPos.x() += Radius;
				break;
			}
			case WP_ARC:
			{
				endPos = to;
				endPos.x() += Radius*cos(AngleStart+AngleArc);
				endPos.y() += Radius*sin(AngleStart+AngleArc);
				break;
			}
		}
	}

	// Returns the distance left of the waypoint path
	// If curPos is given, path starts at that point, so it must be on the waypoint path.
	float GetDistanceLeft(Position* curPos=NULL)
	{
		Position pos;

		switch (wpMode)
		{
			case WP_LINE:
			{
				Position diff;
				if (curPos == NULL)
					pos = from;
				else
					pos = *curPos; // Assume we're on the line already
				diff = to - pos;
				Position dxy(diff);
				dxy.z() = 0;
				return dxy.norm();

				break;
			}
			case WP_CIRCLE:
			{
				return FLT_MAX;
				break;
			}
			case WP_ARC:
			{
				float alpha;
				if (curPos == NULL)
				{
					alpha = AngleStart; // 0 to 2PI
				}
				else
				{
					Position diff = *curPos - to; // Assume we're on the arc already
					alpha = atan2(diff.y(), diff.x()); // -PI to PI
					if (alpha < 0)
						alpha += 2*M_PI; // 0 to 2PI
				}

				float angleEnd = AngleStart + AngleArc; // -2PI to 4PI
				if (AngleArc > 0)
				{
					// LEFT
					if (angleEnd-alpha > 2*M_PI)
						alpha+=2*M_PI;
					return (angleEnd - alpha)*Radius;
				}
				else
				{
					// RIGHT
					if (alpha-angleEnd > 2*M_PI)
						alpha-=2*M_PI;
					return (alpha - angleEnd)*Radius;
				}
				break;
			}
		}
		return 0;
	}

	// Fills up posList with points on the waypoint path, with stepDistance meters in between.
	// Fills distanceLeft with how long that path is.
	// If curPos is given, path starts at that point, so it must be on the waypoint path.
	void GetPath(std::vector<Position>& posList, float& distanceLeft, float stepDistance, Position* curPos=NULL)
	{
		// TODO: set the correct Z
		Position pos;

		switch (wpMode)
		{
			case WP_LINE:
			{

				Position diff;
				if (curPos == NULL)
					pos = from;
				else
					pos = *curPos; // Assume we're on the line already
				diff = to - pos;
				Position dxy(diff);
				dxy.z() = 0;
				distanceLeft = dxy.norm();
				dxy.z() = diff.z(); // Assume constant vertical speed
				dxy /= distanceLeft;

				for (float dl=0; dl<distanceLeft; dl+=stepDistance)
					posList.push_back(pos + dxy*dl);
				break;
			}
			case WP_CIRCLE:
			{
				// Assume curPos is on the circle
				float alpha = 0;
				float alphaEnd = 2*M_PI;
				if (AngleArc < 0)
					alphaEnd = -2*M_PI;
				if (curPos != NULL)
				{
					Position diff = *curPos - to;
					alpha = atan2(diff.y(), diff.x()); // -PI to PI
					if (AngleArc > 0)
						//alphaEnd = alpha + 0.5; // TODO: very magic number
						alphaEnd = alpha + 2*M_PI;
					else
						alphaEnd = alpha - 2*M_PI;
				}

				Position pos = to;
				float angleStep = stepDistance / Radius;
				if (AngleArc > 0)
				{
					// LEFT
					for (; alpha<alphaEnd; alpha+=angleStep)
					{
						pos.x() = to.x() + Radius*cos(alpha);
						pos.y() = to.y() + Radius*sin(alpha);
						posList.push_back(pos);
					}
				}
				else
				{
					// RIGHT
					for (; alpha>alphaEnd; alpha-=angleStep)
					{
						pos.x() = to.x() + Radius*cos(alpha);
						pos.y() = to.y() + Radius*sin(alpha);
						posList.push_back(pos);
					}
				}
				distanceLeft = FLT_MAX;
				break;
			}
			case WP_ARC:
			{
				float alpha;
				if (curPos == NULL)
				{
					alpha = AngleStart; // 0 to 2PI
				}
				else
				{
					Position diff = *curPos - to; // Assume we're on the arc already
					alpha = atan2(diff.y(), diff.x()); // -PI to PI
					if (alpha < 0)
						alpha += 2*M_PI; // 0 to 2PI
				}

				float angleEnd = AngleStart + AngleArc; // -2PI to 4PI
				float angleStep = stepDistance / Radius;
				if (AngleArc > 0)
				{
					// LEFT
					if (angleEnd-alpha > 2*M_PI)
						alpha+=2*M_PI;
					distanceLeft = (angleEnd - alpha)*Radius;
					for (; alpha<angleEnd; alpha+=angleStep)
					{
						pos = to;
						pos.x() += Radius*cos(alpha);
						pos.y() += Radius*sin(alpha);
						posList.push_back(pos);
					}
				}
				else
				{
					// RIGHT
					if (alpha-angleEnd > 2*M_PI)
						alpha-=2*M_PI;
					distanceLeft = (alpha - angleEnd)*Radius;
					for (; alpha>angleEnd; alpha-=angleStep)
					{
						pos = to;
						pos.x() += Radius*cos(alpha);
						pos.y() += Radius*sin(alpha);
						posList.push_back(pos);
					}
				}
				break;
			}
		}
	}


	friend std::ostream& operator<<(std::ostream& os, const WayPoint& struc)
	{
		os << "Mode=" << struc.wpMode;
		switch (struc.wpMode)
		{
			case WP_LINE:
				os << " from=" << struc.from.transpose() << " to=" << struc.to.transpose();
				break;
			case WP_FREE:
				os << " to=" << struc.to.transpose();
				break;
			case WP_CIRCLE:
				os << " center=" << struc.to.transpose() << " Radius=" << struc.Radius << " AngleArc=" << struc.AngleArc;
				break;
			case WP_ARC:
				os << " center=" << struc.to.transpose() << " Radius=" << struc.Radius;
				os << " AngleStart=" << struc.AngleStart << " AngleArc=" << struc.AngleArc;
				break;
		}
		os << " VerticalSpeed=" << struc.VerticalSpeed << " ETA=" << struc.ETA << " ID=" << struc.ID;;
		//os << "Mode=" << struc.wpMode << " from=" << struc.from.transpose() << " to=" << struc.to.transpose() << " vMode=" << struc.vMode;
		//os << " alt=" << struc.altitude << " pitch=" << struc.pitch << " throttle=" << struc.throttle << " climbrate=" << struc.climbRate;
		return os;
	}
};

class WayPointsStruct
{
	public:
		int WayPointsNum;
		WayPoint WayPoints[MAPSELF_MAX_WAYPOINTS];

		WayPointsStruct(): WayPointsNum(0) {}
		void clear() { WayPointsNum = 0; }
		void push_back(WayPoint& wp)
		{
			WayPoints[WayPointsNum++] = wp;
		}
		void pop_back()
		{
			if (WayPointsNum > 0)
				--WayPointsNum;
		}
		void pop_front()
		{
			if (WayPointsNum > 0)
			{
				for (int i=0; i<WayPointsNum-1; ++i)
				{
					WayPoints[i] = WayPoints[i+1];
				}
				--WayPointsNum;
			}
		}

		void remove(int index)
		{
			if (WayPointsNum > 0)
			{
				for (int i=index; i<WayPointsNum-1; ++i)
				{
					WayPoints[i] = WayPoints[i+1];
				}
				--WayPointsNum;
			}
		}

		bool empty() { return (WayPointsNum == 0); }
		int size() { return WayPointsNum; }

		WayPoint& operator[] (unsigned i) { return WayPoints[i]; }

		friend std::ostream& operator<<(std::ostream& os, const WayPointsStruct& struc)
		{
			os << "{";
			for (int i=0; i<struc.WayPointsNum-1; ++i)
				os << struc.WayPoints[i] << "} {";
			os << struc.WayPoints[struc.WayPointsNum-1] << "}";
			return os;
		}
};




#endif // WAYPOINT_H_
