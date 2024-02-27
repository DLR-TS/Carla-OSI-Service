/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>

#include "ParameterDefinitions.h"

struct spawnedActors {
	uint32_t vehicle;
	std::vector<uint32_t> sensors;
};

class CARLAInterface{

    RuntimeParameter runtimeParameter;
	
	//settings are applied for 1 day
	std::chrono::duration<int> settingsDuration{ 60 * 60 * 24 };// 86400s

public:

	std::unique_ptr<carla::client::Client> client;
    std::unique_ptr<carla::client::World> world;
    carla::SharedPtr<carla::client::Map> map;

    //Vehicles are spawned and destroyed by TrafficUpdater.
    //Information about vehicles is needed by GroundTruth creation.
	//So they are stored here.
    std::map<OSIVehicleID, spawnedActors> spawnedVehiclesByCarlaOSIService; //ID in Carla

	std::vector<uint32_t> spawnedSensorsOnExternalSpawnedVehicles;

	/**
	* initialise the interface with the given parameters and connect to the carla server
	* \var runtimeParameter
	* parameters set by start of program
	* \return Success status.
	*/
    int initialise(RuntimeParameter& runtimeParams);

	/**
	* Load the world from carla (world and map)
	*/
	void loadWorld();

	/**
	* Reset specific settings of the world
	*/
	void resetWorldSettings();

	/**
	Perform a simulation step. Will perform a tick of deltaSeconds, as given in the configuration
	\return Time in seconds advanced during step
	*/
	double doStep();

private:

	/**
	* Apply specific settings to the world
	*/
	void applyWorldSettings();
};

#endif //!CARLAINTERFACE_H
