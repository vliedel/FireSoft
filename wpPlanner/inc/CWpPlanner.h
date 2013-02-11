/**
 * @brief 
 * @file CWpPlanner.h
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

#ifndef CWPPLANNER_H_
#define CWPPLANNER_H_

#include "wpPlanner.h"
#include "Defs.h"
#include "ShMemTypedefs.h"
#include "FitnessMap.h"
#include "StructToFromCont.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace rur {

struct WpPlannerConfig
{
	bool Debug;
	long PlanIntervalTime; // Time to wait between checking plan / making a new plan
	int NumWayPointPerPlan; // Number of waypoints per plan
//	int ChoicesPerWayPoint; // Number of choices per waypoint
	float WayPointDT; // Time of one waypoint
	int StepsPerWayPoint; // Number of points to check per waypoint path
	float StraightPreference; // Multiply fitness with this number
	float CollisionAvoidHeightDiff; // When avoiding a uav, how much higher/lower should it fly?
	bool SaveFitnessMap; // Save fitness maps to file?
	std::string FitnessMapFileName; // File name to save the fitness map to
//	float MaxX, MaxY; // Size of the field
//	float HomeX, HomeY;
//	float LandWaitX, LandWaitY;
//	float MinHeight, MaxHeight;
	float CruiseSpeed;
	float MinTurnRadius;
	float MaxVertSpeed;
	float BatteryCriticalTime;
	float CoverageSigmaX, CoverageSigmaY, CoverageMaxSum;
	float CollisionSigmaX, CollisionSigmaY, CollisionSigmaZ;
	float CriticalFitness;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		Debug 						= pt.get<bool>("wpPlanner.Debug");
		PlanIntervalTime 			= pt.get<long>("wpPlanner.PlanIntervalTime");
		NumWayPointPerPlan 			= pt.get<int>("wpPlanner.NumWayPointPerPlan");
//		ChoicesPerWayPoint 			= pt.get<int>("wpPlanner.ChoicesPerWayPoint");
		WayPointDT 					= pt.get<float>("wpPlanner.WayPointDT");
		StepsPerWayPoint 			= pt.get<int>("wpPlanner.StepsPerWayPoint");
		StraightPreference 			= pt.get<float>("wpPlanner.StraightPreference");
		CollisionAvoidHeightDiff 	= pt.get<float>("wpPlanner.CollisionAvoidHeightDiff");
		SaveFitnessMap 				= pt.get<bool>("wpPlanner.SaveFitnessMap");
		FitnessMapFileName 			= pt.get<std::string>("wpPlanner.FitnessMapFileName");
//		MinHeight			= pt.get<float>("UAV.MinHeight");
//		MaxHeight			= pt.get<float>("UAV.MaxHeight");
		CruiseSpeed			= pt.get<float>("UAV.CruiseSpeed");
		MinTurnRadius		= pt.get<float>("UAV.MinTurnRadius");
		MaxVertSpeed		= pt.get<float>("UAV.MaxVertSpeed");
		BatteryCriticalTime	= pt.get<float>("UAV.BatteryCriticalTime");
//		MaxX		= pt.get<float>("field.MaxX");
//		MaxY		= pt.get<float>("field.MaxY");
//		HomeX		= pt.get<float>("field.HomeX");
//		HomeY		= pt.get<float>("field.HomeY");
//		LandWaitX	= pt.get<float>("field.LandWaitX");
//		LandWaitY	= pt.get<float>("field.LandWaitY");
		CoverageSigmaX		= pt.get<float>("fitness.CoverageSigmaX");
		CoverageSigmaY		= pt.get<float>("fitness.CoverageSigmaY");
		CoverageMaxSum		= pt.get<float>("fitness.CoverageMaxSum");
		CollisionSigmaX		= pt.get<float>("fitness.CollisionSigmaX");
		CollisionSigmaY		= pt.get<float>("fitness.CollisionSigmaY");
		CollisionSigmaZ		= pt.get<float>("fitness.CollisionSigmaZ");
		CriticalFitness		= pt.get<float>("fitness.CriticalFitness");
	}

//	void save(const std::string &filename)
//	{
//		boost::property_tree::ptree pt;
//		pt.put("wpPlanner.Debug", Debug);
//		write_json(filename, pt);
//	}
};

class CWpPlanner : public wpPlanner
{
	public:
		~CWpPlanner();
		void Init(std::string module_id);
		void Tick();

	private:
		std::string ShMemNameFitness;
		MapShMemType* ShMemFitness;
		MapMutexType* MutexCollision;
		MapMutexType* MutexBattery;
		MapMutexType* MutexConnectivity;
		MapMutexType* MutexFires;
		MapMutexType* MutexStatic;
		MapMutexType* MutexWall;
		MapMutexType* MutexCoverage;
		Fitness3DVecType*	FitnessCollision;
		FitnessVecType*		FitnessBattery;
		FitnessVecType*		FitnessConnectivity;
		FitnessVecType*		FitnessFires;
		FitnessVecType*		FitnessStatic;
		FitnessWallVecType* FitnessWall;
		FitnessMapType*		FitnessCoverage;

		std::string ShMemNameSelf;
		MapShMemType* ShMemSelf;
		MapMutexType* MutexSelf;
		MapSelfStruct* MapSelf;

		std::string ShMemNameUavs;
		MapShMemType* ShMemUavs;
		MapUavType* MapUavs;
		MapMutexType* MutexUavs;

		WpPlannerConfig config;

		std::ofstream FileOut;
		std::string ModuleId;
		int UavId;

		int* IntMsg;
		std::vector<float>* VecMsg;

		//int TickDivider;
		long LastPlanTime;
//		int LastChoice;
//		float LastHeading;
//		long LastChoiceTime;

		void Plan();
		bool CheckCurPlan(float& dz, UAVState& curState, UAVState& oldState, const Position& pos, float heading, int id, WayPointsStruct& wps);
		float PlanStep(std::vector<WayPoint>& bestChoicesWp, std::vector<int>& bestChoices, int stepsLeft, float dt, Position& pos, float heading, std::vector<FitnessGaussian2D>& coverage);
		float GetVal(const Position& pos, std::vector<FitnessGaussian2D>& coverage);
		float GetColVal(int& uavId, const Position& pos);
		bool OutOfBounds(Position& pos);
		void GetLandWp(WayPoint& wp, const LandingStruct& landing, float height);
		void WriteFitnessToFile();
};

}
#endif // CWPPLANNER_H_
