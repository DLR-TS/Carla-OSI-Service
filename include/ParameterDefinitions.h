
/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef PARAMETERDEFINITONS_H
#define PARAMETERDEFINITONS_H

#include <string>

#include <osi_sensorviewconfiguration.pb.h>

#include "carla_osi/Geometry.h"

struct CityObjectLabel {
	bool None = false;
	bool Buildings = false;
	bool Fences = false;
	bool Other = false;
	//bool Pedestrians = false; no static object
	bool Poles = false;
	bool RoadLines = false;
	bool Roads = false;
	bool Sidewalks = false;
	bool TrafficSigns = false;
	bool Vegetation = false;
	//bool Vehicles = false; no static object
	bool Walls = false;
	//bool Sky = false;
	bool Ground = false;
	bool Bridge = false;
	bool RailTrack = false;
	bool GuardRail = false;
	bool TrafficLight = false;
	bool Static = false;
	//bool Dynamic = false;
	bool Water = false;
	bool Terrain = false;
	bool Any = false;
};

enum SENSORTYPES
{
	GENERIC,
	ULTRASONIC,
	RADAR,
	LIDAR,
	CAMERA
};

struct OSTARSensorConfiguration {
    SENSORTYPES type;
    osi3::SensorViewConfiguration sensorViewConfiguration;
    std::string prefixed_fmu_variable_name;
	std::string parent;
    uint64_t id;
	bool spawned = false;
};

typedef uint64_t OSIVehicleID;

struct ReplayParameter {
	bool enabled = false;

	bool UTMOutput = false;
	MapOffset mapOffset;
	float spawnHeight_Z = 0;

	double weightLength_X = 1;
	double weightWidth_Y = 1;
	double weightHeight_Z = 1;

	std::string spawnCarByName;
};

struct ScenarioRunnerSettings {
	//nearly stock scenario runner implementation
	bool doesTick = false;
	//specific scenario runner implementation in SetLevel
	bool doesTickSL = false;
};

struct RuntimeParameter {
	bool sync = true;
	bool verbose = false;
	bool filter = false;
	std::string filterString = "";
	bool log = false;
	std::string ego = "hero";
	std::string logFileName = "";
	int resumeCarlaAsyncSeconds = 0;
	bool carlaSensors = false;
	std::set<SENSORTYPES> carlasensortypes;
	//parsing options
	CityObjectLabel options;
	bool mapNetworkInGroundTruth = false;

	ReplayParameter replay;
	ScenarioRunnerSettings scenarioRunner;

	std::string carlaHost = "localhost";
	int carlaPort;
	float transactionTimeout;
	float deltaSeconds;
};

#endif //!PARAMETERDEFINITONS_H
