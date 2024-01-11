/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLAGROUNDTRUTH_H
#define CARLAGROUNDTRUTH_H

#include <execution>

#include <carla/client/ActorList.h>

#include "CARLA_Module.h"
#include "CARLA_Interface.h"
#include "carla_osi/Lanes.h"
#include "carla_osi/TrafficSignals.h"

#include "pugixml.hpp"

class GroundTruthCreator : public CARLAModule {

	// ground truth basis created during initialise(), contains all ground truth fields that won't change during the simulation (roads and lanes, buildings, ...)
	std::unique_ptr<osi3::GroundTruth> staticMapTruth;
	// latest world ground truth, calculated during doStep()
	std::shared_ptr<osi3::GroundTruth> latestGroundTruth;
	// invalid latest ground truth
	bool validLatestGroundTruth = false;

public:
	/**
	Retrieve ground truth message generated during last step
	\return Latest world state as osi3::GroundTruth
	*/
	std::shared_ptr<const osi3::GroundTruth> getLatestGroundTruth();

	/**
	Prepare a GroundTruth object with values from the current map which won't change
	*/
	void parseStationaryMapObjects();

	/**
	Invalidate latest ground truth. The next getLatestGroundTruth() shall return new retrieved data from carla.
	*/
	void invalidateLatestGroundTruth() { validLatestGroundTruth = false; }

private:
	/**
	Parse CARLA world to update latestGroundTruth. Called during doStep()
	*/
	std::shared_ptr<osi3::GroundTruth> parseWorldToGroundTruth();

    /**
	Filter objects by carla::rpc::CityObjectLabel 
	return List of 
	*/
    std::vector<carla::rpc::EnvironmentObject> filterEnvironmentObjects();

};

#endif //!CARLAGROUNDTRUTH_H