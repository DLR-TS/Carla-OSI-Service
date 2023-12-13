/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLATRAFFICUPDATE_H
#define CARLATRAFFICUPDATE_H

#include <memory>

#include "osi_trafficupdate.pb.h"

#include "carla/client/BlueprintLibrary.h"

#include "CARLA_Module.h"
#include "CARLA_Interface.h"
#include "Utility.h"
#include "ParameterDefinitions.h"


class TrafficUpdater : public CARLAModule {

	std::vector<std::tuple<std::string, carla::geom::Vector3D>> replayVehicleBoundingBoxes;

public:

	virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla) override;

	/**
	Read traffic update message from traffic participant and update CARLA actor accordingly
	\return success indicator
	*/
	int receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate);

	/**
	Delete spawned vehicles by Carla OSI Service
	*/
	void deleteSpawnedVehicles();

private:
	/**
	Spawn all vehicle actors and save their bounding boxes for a most realistic playback of a scenario via trafficUpdate messages
	*/
	void saveBoundingBoxesOfAvailableVehicles();

	/**
	Determine vehicle to spawn best matching vehicle
	\return vehicle name in CARLA
	*/
	std::string determineVehicleName(const osi3::MovingObject& update);
	
	/**
	Determine transform (location and rotation) to spawn vehicle
	\return transform (location and rotation)
	*/
	carla::geom::Transform determineTransform(const osi3::MovingObject& update);

	/**
	Spawn vehicle in Carla
	\return actor if newly spawned or already existing
	*/
	carla::SharedPtr<carla::client::Actor> spawnVehicleIfNeeded(const osi3::MovingObject& update, carla::ActorId& ActorID);

	/**
	Apply Traffic Update to existing vehicle in Carla
	*/
	void applyTrafficUpdateToActor(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor);

	/**
	Remove all spawned vehicles by TrafficUpdate if id is not in given list
	*/
	void removeSpawnedVehiclesIfNotUpdated(std::vector<int>& listOfUpdatedVehicles);
};

#endif //!CARLATRAFFICUPDATE_H
