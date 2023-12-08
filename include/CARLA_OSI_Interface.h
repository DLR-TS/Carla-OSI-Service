/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef CARLAOSIINTERFACE_H
#define CARLAOSIINTERFACE_H

#include <charconv>

#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <execution>
#include <stdexcept>

#define _USE_MATH_DEFINES
#include <math.h>

#include <boost/bimap.hpp>
#include <boost/foreach.hpp>

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


class CARLAOSIInterface : public CARLAModule
{

	// contains actor ids an the value of their role_name attribute. Does not contain actors without a role. Role names are used as variable name to identify OSI messages
	boost::bimap<std::string, carla::ActorId> actorRole2IDMap;
	std::shared_mutex actorRole2IDMap_mutex;
	// contains OSI messages (values) for variable names / actor roles (keys) that can not always be retrieved, such as the sensor messages, which originate from CARLA sensor events and don't have to occur on every tick. 
	std::map<int, std::shared_ptr<osi3::SensorView>> sensorCache;
	std::shared_mutex sensorCache_mutex;
	// contains all actor ids reported by Carla during the last tick
	std::set<carla::ActorId> activeActors;
	// ground truth basis created during initialise(), contains all ground truth fields that won't change during the simulation (roads and lanes, buildings, ...)
	std::unique_ptr<osi3::GroundTruth> staticMapTruth;
	// latest world ground truth, calculated during doStep()
	std::shared_ptr<osi3::GroundTruth> latestGroundTruth;
	// invalid latest ground truth
	bool validLatestGroundTruth = false;
	// OpenDRIVE xml representation of the map (cached in initialise(), shouldn't change during the simulation)
	//pugi::xml_document xodr;
	//settings are applied for 1 day
	std::chrono::duration<int> settingsDuration{ 60 * 60 * 24 };// 86400s

	// Parameters set by runtime
	RuntimeParameter runtimeParameter;
public:

	/**
	* Fetch the actors in carla and update cache.
	* Should be called after a doStep()
	*/
	void fetchActorsFromCarla();

	/**
	Retrieve CARLA Sensor output from the sensor with the given index. Messages are cached and updated during a sensor's tick.
	\param sensor OSMPSensorView + index
	\return The sensor's latest output as osi3::SensorView, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorView> getSensorView(const std::string& sensor);

	/**
	Send applied SensorViewConfiguration for sensor.
	\param sensor OSMPSensorViewConfiguration + index
	\return The sensor's configuration as osi3::SensorViewConfiguration, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorViewConfiguration> getSensorViewConfiguration(const std::string& sensor);

	/**
	Receive configuration of SensorViewConfiguration message
	\return success indicator
	*/
	int receiveSensorViewConfigurationRequest(osi3::SensorViewConfiguration& sensorViewConfiguration);

private:

	/**
	Clear mapping data and preparsed messages and reparse environment objects.
	Mapping data consists of the actor to role name mapping and sensor to sensor message mappings.
	The preparsed message describes theenvironment objects and will be reconstructed.

	Called during doStep when the world's id has changed.
	*/
	virtual void clearData();

	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData, int index);
};

#endif //!CARLAOSIINTERFACE_H
