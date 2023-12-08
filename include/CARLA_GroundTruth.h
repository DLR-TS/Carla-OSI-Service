/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLAGROUNDTRUTH_H
#define CARLAGROUNDTRUTH_H

#include <execution>

#include "CARLA_Module.h"

#include "carla_osi/Lanes.h"
#include "carla_osi/TrafficSignals.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"

#include "osi_groundtruth.pb.h"

/////////////BEGING
#include <carla/client/Actor.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/SensorData.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/Timestamp.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector3D.h>
#include <carla/geom/Rotation.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/road/Lane.h>
#include <carla/rpc/ObjectLabel.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

//uncomment include if needed
#include "osi_common.pb.h"
//#include "osi_datarecording.pb.h"
//#include "osi_detectedlane.pb.h"
//#include "osi_detectedobject.pb.h"
//#include "osi_detectedoccupant.pb.h"
//#include "osi_detectedroadmarking.pb.h"
//#include "osi_detectedtrafficlight.pb.h"
//#include "osi_detectedtrafficsign.pb.h"
//#include "osi_environment.pb.h"
#include "osi_featuredata.pb.h"
#include "osi_groundtruth.pb.h"
//#include "osi_hostvehicledata.pb.h"
//#include "osi_lane.pb.h"
//#include "osi_object.pb.h"
//#include "osi_occupant.pb.h"
//#include "osi_roadmarking.pb.h"
//#include "osi_sensordata.pb.h"
//#include "osi_sensorspecific.pb.h"
#include "osi_sensorview.pb.h"
//#include "osi_sensorviewconfiguration.pb.h"
#include "osi_trafficcommand.pb.h"
//#include "osi_trafficlight.pb.h"
//#include "osi_trafficsign.pb.h"
//#include "osi_trafficupdate.pb.h"
//#include "osi_version.pb.h"

#include "CARLA_Module.h"
#include "CARLA_Interface.h"
#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"

#include "pugixml.hpp"

//////////////END

class GroundTruthCreator : public CARLAModule {

	// ground truth basis created during initialise(), contains all ground truth fields that won't change during the simulation (roads and lanes, buildings, ...)
	std::unique_ptr<osi3::GroundTruth> staticMapTruth;
	// latest world ground truth, calculated during doStep()
	std::shared_ptr<osi3::GroundTruth> latestGroundTruth;
	// invalid latest ground truth
	bool validLatestGroundTruth = false;
    //hero id
	uint64_t trafficCommandMessageHeroId = 0;

public:
	/**
	Invalidate latest ground truth. The next getLatestGroundTruth() shall return new retrieved data from carla.
	*/
	void invalidateLatestGroundTruth() { validLatestGroundTruth = false; }

	/**
	Retrieve ground truth message generated during last step
	\return Latest world state as osi3::GroundTruth
	*/
	std::shared_ptr<const osi3::GroundTruth> getLatestGroundTruth();

	// prepare a GroundTruth object with values from the current map which won't change 
	void parseStationaryMapObjects();

	// parse CARLA world to update latestGroundTruth. Called during doStep()
	std::shared_ptr<osi3::GroundTruth> parseWorldToGroundTruth();

    /**
	Returns the hero id.
	\return hero id
	*/
	uint64_t getHeroId() { return trafficCommandMessageHeroId; }

private:
    //TODO December add documentation
    std::vector<carla::rpc::EnvironmentObject> filterEnvironmentObjects();

    /*
	Checks if vehicle is spawned by Carla_OSI_Service
	return 0 if not
	*/
	OSIVehicleID vehicleIsSpawned(boost::shared_ptr<const carla::client::Vehicle> vehicle);

};

#endif //!CARLAGROUNDTRUTH_H