/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <charconv>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <execution>
#include <stdexcept>

#include <boost/bimap.hpp>
#include <boost/foreach.hpp>

#include <carla/client/Actor.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/SensorData.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/Timestamp.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/World.h>
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
#include "osi_trafficupdate.pb.h"
//#include "osi_version.pb.h"

#include "Utility.h"
#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"
#include "carla_osi/Lanes.h"
#include "carla_osi/TrafficSignals.h"

#include "pugixml.hpp"
#define _USE_MATH_DEFINES
#include <math.h>


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

struct RuntimeParameter {
	bool sync = true;
	bool verbose = false;
	bool scenarioRunnerDoesTick = false;
	bool filter = false;
	std::string filterString = "";
	bool log = false;
	std::string ego = "hero";
	std::string logFileName = "";
	int resumeCarlaAsyncSeconds = 0;
	bool carlaSensors = false;
	//parsing options
	CityObjectLabel options;
	bool mapNetworkInGroundTruth = false;

	std::string carlaHost = "localhost";
	int carlaPort;
	float transactionTimeout;
	float deltaSeconds;
};

class CARLA2OSIInterface
{
	typedef std::variant<std::shared_ptr<osi3::SensorView>, std::shared_ptr<osi3::FeatureData>, std::shared_ptr<osi3::TrafficCommand>> cachedOSIMessageType;

	std::unique_ptr<carla::client::World> world;
	std::unique_ptr<carla::client::Client> client;
	carla::SharedPtr<carla::client::Map> map;
	// contains actor ids an the value of their role_name attribute. Does not contain actors without a role. Role names are used as variable name to identify OSI messages
	boost::bimap<std::string, carla::ActorId> actorRole2IDMap;
	std::shared_mutex actorRole2IDMap_mutex;
	// contains OSI messages (values) for variable names / actor roles (keys) that can not always be retrieved, such as the sensor messages, which originate from CARLA sensor events and don't have to occur on every tick. 
	std::map<std::string, cachedOSIMessageType> varName2MessageMap;
	std::shared_mutex varName2MessageMap_mutex;
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
	//hero id
	uint64_t heroId = 0;
	//delta seconds in each time step
	float deltaSeconds;
	//settings are applied for 1 day
	std::chrono::duration<int> settingsDuration{ 60 * 60 * 24 };// 86400s

public:
	// Parameters set by runtime
	RuntimeParameter runtimeParameter;

	~CARLA2OSIInterface() {
		if (world) {
			// prevent destruction during grpc accesses
			std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
			//Unfreeze the server and Unreal Editor, useful for development
			auto settings = world->GetSettings();
			settings.synchronous_mode = false;
			world->ApplySettings(settings, settingsDuration);
			world->Tick(client->GetTimeout());
			std::cout << "CARLA2OSIInterface: Disabled synchronous mode of Carla server" << std::endl;
		}
	};

	/**
	* initialise the interface with the given parameters and connect to the carla server
	* \var runtimeParameter
	* parameters set by start of program
	* \return Success status.
	*/
	int initialise(RuntimeParameter& runtimeParameter);

	/**
	Perform a simulation step. Will perform a tick of deltaSeconds, as given in the configuration
	\return Time in seconds advanced during step
	*/
	double doStep();

	/**
	* Fetch the actors in carla and update cache.
	* Should be called after a doStep()
	*/
	void fetchActorsFromCarla();

	/**
	* Load the world from carla (world and map)
	*/
	void loadWorld();

	/**
	* Apply specific settings to the world
	*/
	void applyWorldSettings();

	/**
	* Reset specific settings of the world
	*/
	void resetWorldSettings();

	/**
	Retrieve ground truth message generated during last step
	\return Latest world state as osi3::GroundTruth
	*/
	std::shared_ptr<const osi3::GroundTruth> getLatestGroundTruth();

	/**
	Retrieve CARLA Sensor output from the sensor with the given role name. Messages are cached and updated during a sensor's tick.
	\param sensor_role_name Role attribute of the carla::client::Sensor
	\return The sensor's latest output as osi3::SensorView, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorView> getSensorView(std::string sensor_role_name);

	/**
	Read traffic update message from traffic participant and update position, rotation, velocity and lightstate of CARLA actor.
	\param actorId
	\return success indicator
	*/
	int receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate);

	//Helper function for our client
	/**
	Try to parse the given osi::Identifier to its corresponding actor id and retrieve the actors role_name attribute

	\return the role name of the actor with the given id, or empty string if the actor has no role name or doesn't exist

	Throws std::bad_variant_access if id doesn't correspond to a regular carla actor
	*/
	std::string actorIdToRoleName(const osi3::Identifier& id);

	// prepare a GroundTruth object with values from the current map which won't change 
	void parseStationaryMapObjects();

	std::vector<carla::rpc::EnvironmentObject> filterEnvironmentObjects();

	/**
	Retruns the stepsize.
	\return step size
	*/
	float getDeltaSeconds() { return deltaSeconds; }

	/**
	Returns the hero id.
	\return hero id
	*/
	uint64_t getHeroId() { return heroId; }

	/**
	Invalidate latest ground truth. The next getLatestGroundTruth() shall return new retrieved data from carla.
	*/
	void invalidateLatestGroundTruth() { validLatestGroundTruth = false; }

	/**
	Write the log.
	*/
	void writeLog();

private:

	std::ofstream logFile;
	struct logData { std::string id; double x, y, yaw; };

	osi3::Timestamp* parseTimestamp();
	// parse CARLA world to update latestGroundTruth. Called during doStep()
	std::shared_ptr<osi3::GroundTruth> parseWorldToGroundTruth();

	/**
	Clear mapping data and preparsed messages and reparse environment objects.
	Mapping data consists of the actor to role name mapping and sensor to sensor message mappings.
	The preparsed message describes theenvironment objects and will be reconstructed.

	Called during doStep when the world's id has changed.
	*/
	virtual void clearData();

	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData);

	//output
	void sendTrafficCommand(carla::ActorId actorId);

};

#endif //!CARLAINTERFACE_H
