/**
 * @brief 
 * @file CWpPlanner.cpp
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

//#define WPPLAN_CHOICES_NUM 3 // Number of choices per path segment, must be uneven
#define WPPLAN_CHOICES_NUM 5 // Number of choices per path segment, must be uneven
//#define WPPLAN_SEGMENTS_NUM 4 // Number of path segments to plan
//#define WPPLAN_SEGMENT_DT 3.0 // Time that each path segment takes

#include "CWpPlanner.h"
#include "Protocol.h"
#include "Print.hpp"
#include "CTime.h"
#include "AutoPilotProt.h"

using namespace rur;

CWpPlanner::~CWpPlanner()
{
	delete ShMemFitness;
	delete ShMemSelf;
	delete ShMemUavs;
	FileOut.close();
}

void CWpPlanner::Init(std::string module_id)
{
	wpPlanner::Init(module_id);
	ShMemNameFitness = "mapFitness_" + module_id;
	ShMemNameSelf = "mapSelf_" + module_id;
	ShMemNameUavs = "mapUAV_" + module_id;
	UavId = atoi(module_id.c_str());
	srand(UavId+2); // seed of 0 and 1 are special cases

	config.load("config.json");

	LastPlanTime = get_cur_1ms();

	try
	{
		// Open the shared memory
		ShMemFitness = new MapShMemType(boost::interprocess::open_only, ShMemNameFitness.c_str());
		ShMemSelf = new MapShMemType(boost::interprocess::open_only, ShMemNameSelf.c_str());
		ShMemUavs = new MapShMemType(boost::interprocess::open_only, ShMemNameUavs.c_str());

		// Find the maps and mutexes using the c-string name
		FitnessCollision =		ShMemFitness->find<Fitness3DVecType>("Collision").first;
		FitnessBattery =		ShMemFitness->find<FitnessVecType>("Battery").first;
		FitnessConnectivity =	ShMemFitness->find<FitnessVecType>("Connectivity").first;
		FitnessFires =			ShMemFitness->find<FitnessVecType>("Fires").first;
		FitnessStatic =			ShMemFitness->find<FitnessVecType>("Static").first;
		FitnessWall =			ShMemFitness->find<FitnessWallVecType>("Wall").first;
		FitnessCoverage =		ShMemFitness->find<FitnessMapType>("Coverage").first;

		MutexCollision = 		ShMemFitness->find<MapMutexType>("MutexCollision").first;
		MutexBattery = 			ShMemFitness->find<MapMutexType>("MutexBattery").first;
		MutexConnectivity = 	ShMemFitness->find<MapMutexType>("MutexConnectivity").first;
		MutexFires = 			ShMemFitness->find<MapMutexType>("MutexFires").first;
		MutexStatic = 			ShMemFitness->find<MapMutexType>("MutexStatic").first;
		MutexWall = 			ShMemFitness->find<MapMutexType>("MutexWall").first;
		MutexCoverage = 		ShMemFitness->find<MapMutexType>("MutexCoverage").first;

		MapSelf = 			ShMemSelf->find<MapSelfStruct>("Map").first;
		MutexSelf = 		ShMemSelf->find<MapMutexType>("Mutex").first;

		MapUavs =			ShMemUavs->find<MapUavType>("Map").first;
		MutexUavs = 		ShMemUavs->find<MapMutexType>("Mutex").first;
	}
	catch(...)
	{
		FitnessCollision = NULL;
		FitnessBattery = NULL;
		FitnessConnectivity = NULL;
		FitnessFires = NULL;
		FitnessStatic = NULL;
		FitnessWall = NULL;
		FitnessCoverage = NULL;

		MutexCollision = NULL;
		MutexBattery = NULL;
		MutexConnectivity = NULL;
		MutexFires = NULL;
		MutexStatic = NULL;
		MutexWall = NULL;
		MutexCoverage = NULL;

		MapSelf = NULL;
		MutexSelf = NULL;

		MapUavs = NULL;
		MutexUavs = NULL;

		delete ShMemFitness;
		delete ShMemSelf;
		delete ShMemUavs;
		std::cout << "Error in " << ShMemNameFitness << " or " << ShMemSelf << std::endl;
		throw;
	}

	if (config.SaveFitnessMap)
	{
		//std::string fname = "fitness_map_" + module_id;
		std::string fname = config.FitnessMapFileName + module_id;
		FileOut.open(fname.c_str());
	}

}

void CWpPlanner::Tick()
{
//	int* cmd = readCommand(false);
//	if (cmd != NULL)
//	{
//
//	}

	if (get_duration(LastPlanTime, get_cur_1ms()) > config.PlanIntervalTime)
	{
		LastPlanTime = get_cur_1ms();
		long start = get_cur_1us();
		Plan();
		std::cout << get_cur_1ms() << " Planning took " << get_duration(start, get_cur_1us())/1000 << " ms" << std::endl;

		if (config.SaveFitnessMap)
			WriteFitnessToFile();

	}
}


void CWpPlanner::Plan()
{
	std::vector<FitnessGaussian2D> coverage;
	float heightAdjustment(0);
	std::vector<int> bestChoices;
	std::vector<WayPoint> bestChoicesWp;

	float heading;
	Position pos;
	WayPointsStruct curWps;
	int id;
	UAVState state;
	UAVState oldState;
	float batteryTimeLeft;
	LandingStruct landing;
	float minHeight;
	float maxHeight;
	bool enablePlanner;
	{
		// Copy current states
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
		heading = MapSelf->UavData.Geom.Heading.angle();
		pos = MapSelf->UavData.Geom.Pos;
		curWps = MapSelf->WayPoints;
		id = MapSelf->UavData.UavId;
		state = MapSelf->UavData.State;
		oldState = MapSelf->PreviousState;
		batteryTimeLeft = MapSelf->UavData.BatteryTimeLeft;
		landing = MapSelf->GsCmd.Landing;
		minHeight = MapSelf->GsCmd.HeightMin;
		maxHeight = MapSelf->GsCmd.HeightMax;
		enablePlanner = MapSelf->GsCmd.EnablePlanner;
	}
	if (!enablePlanner)
	{
		std::cout << get_cur_1ms() << " wp planner is disabled" << std::endl;
		return;
	}

	UAVState newState = state;
	UAVState newOldState = oldState;
/*
	{
		// Copy relevant gaussians to vector, so we don't lock for too long
		// Take an area with radius: max_path_radius + 3*sigma coverage
		// This is oversized, take the heading into account
		float radius = config.NumWayPointPerPlan * config.WayPointDT * CRUISE_SPEED + 3*FIT_COV_SIGMA_X;
		boost::interprocess::scoped_lock<MapMutexType> lockCoverage(*MutexCoverage);
		FitnessCoverage->Get(coverage, pos.x()-radius, pos.x()+radius, pos.y()-radius, pos.y()+radius);
	}
*/

	if (config.Debug)
	{
		std::cout << "Curwp num=" << curWps.WayPointsNum << " pos=[" << pos.transpose() << "]"
				<< " state=" << state << " oldState=" << oldState;
	}

	// When flying normally: check current plan for collisions etc. And check if the plan is still long enough.
	// If no collisions etc. and plan is long enough, we don't need a new plan.
	bool plan = true;
	if (newState != UAVSTATE_WAITING_TO_LAND &&
			newState != UAVSTATE_LANDING &&
			newState != UAVSTATE_LANDED &&
			newState != UAVSTATE_TAKING_OFF)
	{
		if (CheckCurPlan(heightAdjustment, newState, newOldState, pos, heading, id, curWps))
		{
			plan = false;
			//return;
		}
		if (newState == UAVSTATE_COLLISION_AVOIDING && state != UAVSTATE_COLLISION_AVOIDING)
			newOldState = state;
	}
	if (config.Debug)
		std::cout << std::endl;

	// Update state in map self
	if (state != newState || oldState != newOldState)
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
		MapSelf->UavData.State = newState;
		MapSelf->PreviousState = newOldState;
		state = newState;
		oldState = newOldState;
	}

	// When flying normally: check batteries
	if (newState != UAVSTATE_GOING_HOME &&
			newState != UAVSTATE_WAITING_TO_LAND &&
			newState != UAVSTATE_LANDING &&
			newState != UAVSTATE_LANDED &&
			newState != UAVSTATE_TAKING_OFF)
	{
		float distLeft = pos.x() - landing.Pos.x();
		distLeft += pos.y() - landing.Pos.y();
		float timeNeeded = config.BatteryCriticalTime + distLeft/config.CruiseSpeed;
		if (batteryTimeLeft < timeNeeded)
		{
			if (newState == UAVSTATE_COLLISION_AVOIDING)
				newOldState = UAVSTATE_GOING_HOME;
			else
				newState = UAVSTATE_GOING_HOME;
		}
	}


	// Fly height: default height + adjustment for collision avoidance
	pos.z() = minHeight + id * (maxHeight-minHeight) / (UAVS_NUM-1);
	pos.z() += heightAdjustment;


	switch (newState)
	{
		case UAVSTATE_GOING_HOME:
		{
			// Fly directly to land circle
			WayPoint wp;
			GetLandWp(wp, landing, pos.z());
			bestChoicesWp.push_back(wp);

			// Check if we are at the land circle yet
			Position dPos(pos);
			dPos.x() -= wp.to.x();
			dPos.y() -= wp.to.y();
			dPos.z() = 0;
			if (dPos.norm() < landing.Radius + 50) // TODO: magic number
				newState = UAVSTATE_WAITING_TO_LAND;
			break;
		}
		case UAVSTATE_WAITING_TO_LAND:
		{
			// Check if we can land: no other uavs below us landing or waiting to land
			bool land=true;
			{
				boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);
				MapUavIterType it;
				for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
				{
					if ((it->second.data.UavId < id && it->second.data.State == UAVSTATE_WAITING_TO_LAND)
							|| it->second.data.State == UAVSTATE_LANDING)
					{
						std::cout << "Waiting for uav " << it->second.data.UavId << " to land" << std::endl;
						land = false;
						break;
					}
				}
			}
			if (land)
			{
				std::cout << "My turn to land" << std::endl;
				newState = UAVSTATE_LANDING;
				VecMsgType vecMsg;
				vecMsg.push_back(PROT_AP_SET_MODE);
				vecMsg.push_back(AP_PROT_MODE_LAND);
				writeToAutoPilot(vecMsg);
			}
			break;
		}
		case UAVSTATE_LANDING:
		{
			// Landing is performed by auto pilot
			// Check if no other UAVs below us are landing, if so: abort landing!

			bool land=true;
			{
				boost::interprocess::scoped_lock<MapMutexType> lockUavs(*MutexUavs);
				MapUavIterType it;
				for (it = MapUavs->begin(); it != MapUavs->end(); ++it)
				{
					if (it->second.data.State == UAVSTATE_LANDING)
					{
						std::cout << "Waiting for uav " << it->second.data.UavId << " to land" << std::endl;
						land = false;
						break;
					}
				}
			}
			if (!land)
			{
				std::cout << "Aborting landing" << std::endl;
				newState = UAVSTATE_WAITING_TO_LAND;
				VecMsgType vecMsg;
				vecMsg.push_back(PROT_AP_SET_MODE);
				vecMsg.push_back(AP_PROT_MODE_WP);
				writeToAutoPilot(vecMsg);

				// TODO: Assume the waypoint is still in autopilot memory or set it again?
				WayPoint wp;
				GetLandWp(wp, landing, pos.z());
				bestChoicesWp.push_back(wp);
			}
			break;
		}
		case UAVSTATE_LANDED:
		{
			// Can't do much now :)
			break;
		}
		default:
		{
			if (plan)
				// Make a new plan
				PlanStep(bestChoicesWp, bestChoices, config.NumWayPointPerPlan, config.WayPointDT, pos, heading, coverage);
			break;
		}
	}

	// Update state in map self
	if (state != newState || oldState != newOldState)
	{
		boost::interprocess::scoped_lock<MapMutexType> lockSelf(*MutexSelf);
		MapSelf->UavData.State = newState;
		MapSelf->PreviousState = newOldState;
		if (newState == UAVSTATE_LANDING)
			MapSelf->RequestedAPModeByWP = AP_PROT_MODE_LAND;
		else
			MapSelf->RequestedAPModeByWP = AP_PROT_MODE_WP;
	}

	// When avoiding collision, set max vertical speed
	if ((newState == UAVSTATE_COLLISION_AVOIDING) && (!bestChoices.empty()))
	{
		bestChoicesWp.rbegin()->VerticalSpeed = (heightAdjustment>0) ? config.MaxVertSpeed : -config.MaxVertSpeed;
	}

	// Send new waypoints to the autopilot
	if (!bestChoicesWp.empty())
	{
		WayPointsStruct wps;
		std::vector<WayPoint>::reverse_iterator itChoicesWp;
		for (itChoicesWp = bestChoicesWp.rbegin(); itChoicesWp != bestChoicesWp.rend(); ++itChoicesWp)
			wps.push_back(*itChoicesWp);

		std::cout << "bestchoices=";
		dobots::print(bestChoices.rbegin(), bestChoices.rend());
		dobots::print(bestChoicesWp.rbegin(), bestChoicesWp.rend());

		VecMsgType vecMsg;
		vecMsg.push_back(PROT_AP_SET_WAYPOINTS);
		ToCont(wps, vecMsg);
		writeToAutoPilot(vecMsg);
	}
}



// Check if the current plan does not get the uav into trouble (collisions etc)
// and if current plan is almost reached yet.
// Returns true if current plan can still be executed.
// Sets curState and oldState.
// Sets dz in case a collision has to be avoided.
bool CWpPlanner::CheckCurPlan(float& dz, UAVState& curState, UAVState& oldState, const Position& pos, float heading, int id, WayPointsStruct& wps)
{
	// TODO: lots of similar code
	if (wps.WayPointsNum < 1)
		return false;

	std::vector<Position> checkPos;
	Position curPos(pos);
	float distanceLeft;
	float stepDist = 2*std::min(config.CollisionSigmaX, config.CollisionSigmaY);
	wps[0].GetPath(checkPos, distanceLeft, stepDist, &curPos);

	if (config.Debug)
		std::cout << " wp=" << wps[0];
	std::vector<Position>::iterator itPos;
	int uavId;
	for (itPos = checkPos.begin(); itPos != checkPos.end(); ++itPos)
	{
		if (GetColVal(uavId, *itPos) > config.CriticalFitness)
		{
			if (id > uavId)
				dz = config.CollisionAvoidHeightDiff;
			else
				dz = -config.CollisionAvoidHeightDiff;
			if (curState != UAVSTATE_COLLISION_AVOIDING)
			{
				oldState = curState;
				curState = UAVSTATE_COLLISION_AVOIDING;
			}
			return false;
		}
	}

	if (config.Debug)
		std::cout << " distanceLeft=" << distanceLeft;
	// If the current waypoint is far from done, we don't need to check the next one
	// TODO: magic number
	if (distanceLeft > config.CruiseSpeed * 2)
	{
		// If state is still collision avoid, but shouldn't be anymore
		if (curState == UAVSTATE_COLLISION_AVOIDING)
		{
			curState = oldState;
			return false; // We need a new waypoint that is at the normal height again
		}
		return true;
	}

	if (wps.WayPointsNum < 2)
		//return false; // Why?
		return true;

	if (config.Debug)
		std::cout << " wp=" << wps[1];
	checkPos.clear();
	wps[1].GetPath(checkPos, distanceLeft, stepDist);
	for (itPos = checkPos.begin(); itPos != checkPos.end(); ++itPos)
	{
		if (GetColVal(uavId, *itPos) > config.CriticalFitness) // Path is inside the 2 sigma radius
		{
			if (id > uavId)
				dz = config.CollisionAvoidHeightDiff;
			else
				dz = -config.CollisionAvoidHeightDiff;
			if (curState != UAVSTATE_COLLISION_AVOIDING)
			{
				oldState = curState;
				curState = UAVSTATE_COLLISION_AVOIDING;
			}
			return false;
		}
	}

	// If state is still collision avoid, but shouldn't be anymore
	if (curState == UAVSTATE_COLLISION_AVOIDING)
	{
		curState = oldState;
		return false; // We need a new waypoint that is at the normal height again
	}
	return true;
}



float CWpPlanner::PlanStep(std::vector<WayPoint>& bestChoicesWp, std::vector<int>& bestChoices, int stepsLeft, float dt, Position& pos, float heading, std::vector<FitnessGaussian2D>& coverage)
{
	float speed = config.CruiseSpeed; // TODO: should be adjustable
	float minR = speed*speed/(GRAVITY*tan(ROLLANGLE_MAX)); // Minimum radius we can fly
//	float circle_time = min_r*2*M_PI/speed; // Time it takes to fly a circle
	//float rot_step = speed/min_r;
	//minR = 120; // This is a guaranteed radius we can fly
	minR = config.MinTurnRadius;
	float distStep = dt*speed;
	Position posCheck;
	WayPoint wp;

	int numAngles = WPPLAN_CHOICES_NUM/2;
//	int numAngles = config.ChoicesPerWayPoint/2;

	float bestFit = FLT_MAX;
	//int bestChoice = 0;
	WayPoint bestChoiceWp;
	int bestChoice;
	std::vector<WayPoint> bestChoicesWpChildren[WPPLAN_CHOICES_NUM];
	std::vector<int> bestChoicesChildren[WPPLAN_CHOICES_NUM];
//	std::vector<WayPoint> *bestChoicesWpChildren = new std::vector<WayPoint> [config.ChoicesPerWayPoint];
//	std::vector<int> *bestChoicesChildren = new std::vector<int> [config.ChoicesPerWayPoint];

	int steps = config.StepsPerWayPoint; // Steps to check path for fitness

	// Each step 3 choices: left, forward, right
	for (int i=-numAngles; i<=numAngles; ++i)
	{
		float fit = 0;
		float rot = 0;

//		std::cout << stepsLeft << " heading=" << heading << " pos=[" << pos.x() << " " << pos.y() << "]";


		if (i==0) // Straight
		{
//			std::cout << " STRAIGHT:";
			float stepX = distStep/steps*cos(heading);
			float stepY = distStep/steps*sin(heading);
			for (int j=1; j<=steps; ++j)
			{
				posCheck = pos;
				posCheck.x() += j*stepX;
				posCheck.y() += j*stepY;
				//std::cout << " [" << posCheck.x() << " " << posCheck.y() << "]";
				if (OutOfBounds(posCheck))
				{
					//fit = FLT_MAX;
					break;
				}
				fit += GetVal(posCheck, coverage);
//				fit *= config.StraightPreference; // Preference for going straight
				fit -= config.StraightPreference; // Preference for going straight
			}
			wp.from = pos;
			wp.to = posCheck;
			wp.wpMode = WP_LINE;
		}
		else
		{
			float r = i/numAngles*minR; // This might not be the best choice, seems good though
			float angleStart = heading - M_PI/2; // -1.5PI to 0.5PI
			float angleStep = dt*speed/r/steps;
			if (i<0)
			{
				r *= -1;
				angleStart += M_PI; // -0.5PI to 1.5PI
//				std::cout << " RIGHT:";
			}
			else
			{
//				std::cout << " LEFT:";
			}
			Position Center = pos;
			Center.x() -= r*cos(angleStart);
			Center.y() -= r*sin(angleStart);
			for (int j=1; j<=steps; ++j)
			{
				rot = j*angleStep;
				posCheck = Center;
				posCheck.x() += r*cos(angleStart + rot);
				posCheck.y() += r*sin(angleStart + rot);
				//std::cout << " [" << posCheck.x() << " " << posCheck.y() << "]";
				if (OutOfBounds(posCheck))
				{
					//fit = FLT_MAX;
					break;
				}
				fit += GetVal(posCheck, coverage);
			}
			//wp.from << heading + M_PI/2, angleStep*steps, minR;
			if (angleStart < 0)
				angleStart += 2*M_PI;
//			if (angleStart > 2*M_PI) // Shouldn't happen
//				angleStart -= 2*M_PI;
			wp.AngleStart = angleStart;
			wp.AngleArc = angleStep*steps;
			wp.Radius = r;
			wp.to = Center;
			wp.wpMode = WP_ARC;
		}

		if (OutOfBounds(posCheck))
		{
//			std::cout << " out of bounds!" << std::endl;
			continue;
		}

//		std::cout << " partial fit=" << fit << std::endl;
		if (stepsLeft > 1)
			fit += PlanStep(bestChoicesWpChildren[i+numAngles], bestChoicesChildren[i+numAngles], stepsLeft-1, dt, posCheck, heading+rot, coverage);

//		std::cout << stepsLeft << " summed fit=" << fit << std::endl;
		if (fit < bestFit)
		{
			bestFit = fit;
			bestChoiceWp = wp;
			bestChoice = i;
		}
	} // End of loop over choices

	if (stepsLeft > 1)
	{
		// Swapping is a lot faster and can be used since we don't need bestChoicesChildren anymore
		bestChoices.swap(bestChoicesChildren[bestChoice+numAngles]);
		bestChoicesWp.swap(bestChoicesWpChildren[bestChoice+numAngles]);
//		bestChoices.assign(bestChoicesChildren[bestChoice+numAngles].begin(), bestChoicesChildren[bestChoice+numAngles].end());
//		bestChoicesWp.assign(bestChoicesWpChildren[bestChoice+numAngles].begin(), bestChoicesWpChildren[bestChoice+numAngles].end());
	}
	bestChoices.push_back(bestChoice);
	bestChoicesWp.push_back(bestChoiceWp);
	return bestFit;
}


float CWpPlanner::GetVal(const Position& pos, std::vector<FitnessGaussian2D>& coverage)
{
	// Start at fitness of 1, so that the straight preference actually works?
	float fit(0);
/*
	std::vector<FitnessGaussian2D>::iterator itCov;
	for (itCov = coverage.begin(); itCov != coverage.end(); ++itCov)
	{
		if (fit>1)
			return 1;
		fit += itCov->GetValue(pos);
	}
*/
	{
		// Get coverage fitness, sum up coverage gaussians that are close, cap at value of 1
		boost::interprocess::scoped_lock<MapMutexType> lockCoverage(*MutexCoverage);
		float r = 2*std::max(config.CoverageSigmaX, config.CoverageSigmaY);
		fit += FitnessCoverage->GetValue(pos, pos.x()-r, pos.x()+r, pos.y()-r, pos.y()+r, config.CoverageMaxSum);
	}

	//fit += GetColVal(pos);
	{
		Fitness3DVecIterType itCol;
		boost::interprocess::scoped_lock<MapMutexType> lockCollision(*MutexCollision);
		for (itCol = FitnessCollision->begin(); itCol != FitnessCollision->end(); ++itCol)
			fit += itCol->GetValue(pos);
	}
	{
		FitnessVecIterType itStat;
		boost::interprocess::scoped_lock<MapMutexType> lockStatic(*MutexStatic);
		for (itStat = FitnessStatic->begin(); itStat != FitnessStatic->end(); ++itStat)
			fit += itStat->GetValue(pos);
	}
	{
		FitnessVecIterType itBatt;
		boost::interprocess::scoped_lock<MapMutexType> lockBattery(*MutexBattery);
		if (!FitnessBattery->empty())
			for (itBatt = FitnessBattery->begin(); itBatt != FitnessBattery->end(); ++itBatt)
				fit += itBatt->GetValue(pos);
	}
	{
		FitnessWallVecIterType itWall;
		boost::interprocess::scoped_lock<MapMutexType> lockWall(*MutexWall);
		for (itWall = FitnessWall->begin(); itWall != FitnessWall->end(); ++itWall)
		{
			fit += itWall->GetValue(pos);
			//std::cout << "pos=[" << pos.transpose() << "] fitWall=" << itWall->GetValue(pos) << std::endl;
		}
	}

	return fit;
}


float CWpPlanner::GetColVal(int& uavId, const Position& pos)
{
	float fit[UAVS_NUM] = {0};
	{
		Fitness3DVecIterType itCol;
		boost::interprocess::scoped_lock<MapMutexType> lockCollision(*MutexCollision);
		for (itCol = FitnessCollision->begin(); itCol != FitnessCollision->end(); ++itCol)
			fit[itCol->ID] += itCol->GetValue(pos);
	}
	float maxFit = 0;
	for (int i=0; i<UAVS_NUM; ++i)
	{
		if (fit[i] > maxFit)
		{
			maxFit = fit[i];
			uavId = i;
		}
	}

	if (config.Debug)
		std::cout << " checkPos=[" << pos.transpose() << "]" << " fit=" << maxFit;
	return maxFit;
}


bool CWpPlanner::OutOfBounds(Position& pos)
{
	return false;
}

void CWpPlanner::GetLandWp(WayPoint& wp, const LandingStruct& landing, float height)
{
	wp.to = landing.Pos;
	wp.to.z() = height;
	wp.to.x() += landing.Length * cos(landing.Heading.angle()+M_PI);
	wp.to.y() += landing.Length * sin(landing.Heading.angle()+M_PI);
	if (landing.LeftTurn)
	{
		wp.to.x() += landing.Radius * cos(landing.Heading.angle()+0.5*M_PI);
		wp.to.y() += landing.Radius * sin(landing.Heading.angle()+0.5*M_PI);
		wp.AngleArc = 1.0;
	}
	else
	{
		wp.to.x() += landing.Radius * cos(landing.Heading.angle()-0.5*M_PI);
		wp.to.y() += landing.Radius * sin(landing.Heading.angle()-0.5*M_PI);
		wp.AngleArc = -1.0;
	}

	wp.wpMode = WP_CIRCLE;
	wp.Radius = landing.Radius;
}

void CWpPlanner::WriteFitnessToFile()
{
	/*
		{
			boost::interprocess::scoped_lock<MapMutexType> lockCollision(*MutexCollision);
			boost::interprocess::scoped_lock<MapMutexType> lockCoverage(*MutexCoverage);
			for (int y=0; y<=1000; y+=10)
			{
				posCheck.y() = y;
				for (int x=0; x<=1000; x+=10)
				{
					float fit=0;
					posCheck.x() = x;
					for (itCol = FitnessCollision->begin(); itCol != FitnessCollision->end(); ++itCol)
					{
						fit += itCol->GetValue(posCheck);
					}

					FitnessMapIterXType itX;
					FitnessMapIterYType itY;
					for (itX = FitnessCoverage->MapX.begin(); itX != FitnessCoverage->MapX.end(); ++itX)
					{
						for (itY = itX->second.begin(); itY != itX->second.end(); ++itY)
						{
							fit += itY->second.GetValue(posCheck);
	//						//FileOut << itX->first << " " << itY->first << " " << itY->second.Amplitude << std::endl;
						}
					}

					FileOut << fit << " ";
				}
				FileOut << std::endl;
			}
		}
	 */


	// Write gaussians to text file
	FileOut << get_cur_1us();

	{
		boost::interprocess::scoped_lock<MapMutexType> lockCollision(*MutexCollision);
		Fitness3DVecIterType itCol;
		for (itCol = FitnessCollision->begin(); itCol != FitnessCollision->end(); ++itCol)
		{
			FileOut << " " << itCol->Center.x();
			FileOut << " " << itCol->Center.y();
			//			FileOut << " " << itCol->Fitness2D.Center.z();
			FileOut << " " << itCol->Amplitude;
			FileOut << " " << itCol->A;
			FileOut << " " << itCol->B;
			FileOut << " " << itCol->C;
			//			FileOut << " " << itCol->D;
			FileOut << " " << itCol->MinVal;
			FileOut << " " << itCol->MaxVal;
		}
	}
	{
		boost::interprocess::scoped_lock<MapMutexType> lockCoverage(*MutexCoverage);
		FitnessMapIterXType itX;
		FitnessMapIterYType itY;
		for (itX = FitnessCoverage->MapX.begin(); itX != FitnessCoverage->MapX.end(); ++itX)
		{
			for (itY = itX->second.begin(); itY != itX->second.end(); ++itY)
			{
				//FileOut << itX->first << " " << itY->first << " " << itY->second.Amplitude << std::endl;
				FileOut << " " << itY->second.Center.x();
				FileOut << " " << itY->second.Center.y();
				FileOut << " " << itY->second.Amplitude;
				FileOut << " " << itY->second.A;
				FileOut << " " << itY->second.B;
				FileOut << " " << itY->second.C;
				FileOut << " " << itY->second.MinVal;
				FileOut << " " << itY->second.MaxVal;
			}
		}
	}
	{
		boost::interprocess::scoped_lock<MapMutexType> lockStatic(*MutexStatic);
		FitnessVecIterType itStat;
		for (itStat = FitnessStatic->begin(); itStat != FitnessStatic->end(); ++itStat)
		{
			FileOut << " " << itStat->Center.x();
			FileOut << " " << itStat->Center.y();
			FileOut << " " << itStat->Amplitude;
			FileOut << " " << itStat->A;
			FileOut << " " << itStat->B;
			FileOut << " " << itStat->C;
			FileOut << " " << itStat->MinVal;
			FileOut << " " << itStat->MaxVal;
		}
	}

	{
		boost::interprocess::scoped_lock<MapMutexType> lockBattery(*MutexBattery);
		FitnessVecIterType itBatt;
		for (itBatt = FitnessBattery->begin(); itBatt != FitnessBattery->end(); ++itBatt)
		{
			FileOut << " " << itBatt->Center.x();
			FileOut << " " << itBatt->Center.y();
			FileOut << " " << itBatt->Amplitude;
			FileOut << " " << itBatt->A;
			FileOut << " " << itBatt->B;
			FileOut << " " << itBatt->C;
			FileOut << " " << itBatt->MinVal;
			FileOut << " " << itBatt->MaxVal;
		}
	}
	FileOut << std::endl;
}

