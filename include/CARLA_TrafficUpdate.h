/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLATRAFFICUPDATE_H
#define CARLATRAFFICUPDATE_H

#include <memory>

#include "osi_trafficupdate.pb.h"

#include "carla/client/BlueprintLibrary.h"

#include "CARLA_Interface.h"
#include "Utility.h"
#include "ParameterDefinitions.h"


class TrafficUpdater{

	std::shared_ptr<CARLAInterface> carla;

	std::vector<std::tuple<std::string, carla::geom::Vector3D>> replayVehicleBoundingBoxes;

	// Parameters set by runtime
    RuntimeParameter runtimeParameter;
public:

	/**
	* initialise the interface with the given parameters and connect to the carla server
	* \var runtimeParams
	* \var carla
	* parameters set by start of program
	* \return Success status.
	*/
    int initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla);

	/**
	Spawn all vehicle actors and save their bounding boxes for a most realistic playback of a scenario via trafficUpdate messages.
	*/
	void fillBoundingBoxLookupTable();

	/**
	Read traffic update message from traffic participant and update position, rotation, velocity and lightstate of CARLA actor.
	\return success indicator
	*/
	int receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate);

	void replayTrafficUpdate(const osi3::TrafficUpdate& update, carla::ActorId& ActorID);

	/**
	Apply Traffic Update to existing vehicle in Carla
	*/
	void applyTrafficUpdate(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor);

	/**
	Delete spawned vehicles from replay.
	*/
	void deleteSpawnedVehicles();

};

#endif //!CARLATRAFFICUPDATE_H
