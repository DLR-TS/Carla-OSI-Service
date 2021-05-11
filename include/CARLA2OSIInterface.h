#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <charconv>
#include <variant>
#include <string>
#include <mutex>
#include <shared_mutex>

#include "pugixml.hpp"
#include <boost/bimap.hpp>
#include <boost/foreach.hpp>

#include <carla/client/Actor.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/SensorData.h>

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
#include "sl45_motioncommand.pb.h"
#include "sl45_vehiclecommunicationdata.pb.h"

class CARLA2OSIInterface
{
	typedef std::variant<std::shared_ptr<osi3::SensorView>, std::shared_ptr<osi3::FeatureData>, std::shared_ptr<osi3::TrafficCommand>> cachedOSIMessageType;

	std::unique_ptr<carla::client::World> world;
	std::unique_ptr<carla::client::Client> client;
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
	// OpenDRIVE xml representation of the map (cached in initialise(), shouldn't change during the simulation)
	pugi::xml_document xodr;

public:

	~CARLA2OSIInterface() {
		if (world) {
			// prevent destruction during grpc accesses
			std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
			//Unfreeze the server and Unreal Editor, useful for development
			auto settings = world->GetSettings();
			settings.synchronous_mode = false;
			world->ApplySettings(settings);
			world->Tick(client->GetTimeout());
			std::cout << "CARLA2OSIInterface: Disabled synchronous mode of Carla server" << std::endl;
		}
	};

	/**
	* initialise the interface with the given parameters and connect to the carla server
	* \var host
	* host name or ip of the carla server
	* \var port
	* port of the carla server
	* \var transactionTimeout
	* transaction timeout in seconds
	* \var deltaSeconds
	* simulation time delta per tick
	* \return Success status.
	*/
	virtual int initialise(std::string host, uint16_t port, double transactionTimeout, double deltaSeconds);

	/**
	Perform a simulation step. Will perform a tick of deltaSeconds, as given in the configuration
	\return Time in seconds advanced during step
	*/
	virtual double doStep();

	/**
	Retrieve ground truth message generated during last step
	\return Latest world state as osi3::GroundTruth
	*/
	virtual std::shared_ptr<const osi3::GroundTruth> getLatestGroundTruth();

	/**
	Retrieve CARLA Sensor output from the sensor with the given role name. Messages are cached and updated during a sensor's tick.
	\param sensor_role_name Role attribute of the carla::client::Sensor
	\return The sensor's latest output as osi3::SensorView, or nullptr if no sensor with given name is found
	*/
	virtual std::shared_ptr<const osi3::SensorView> getSensorView(std::string sensor_role_name);

	/**
	Read traffic update message from traffic participant and update position, rotation, velocity and lightstate of CARLA actor.
	\param actorId
	\return success indicator
	*/
	int receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate);
private:

	std::string_view getPrefix(std::string_view name);
	osi3::Timestamp* parseTimestamp();
	// prepare a GroundTruth object with values from the current map which won't change 
	//TODO return type
	void parseStationaryMapObjects();
	// parse CARLA world to update latestGroundTruth. Called during doStep()
	std::shared_ptr<osi3::GroundTruth> parseWorldToGroundTruth();

	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData);

	//output
	void sendTrafficCommand(carla::ActorId actorId);

	//input

	/**
	Read motion command message from ego vehicle and update position, rotation and velocity of CARLA actor.
	\param actorId
	\return success indicator
	*/
	int receiveMotionCommand(setlevel4to5::MotionCommand& motionCommand);
};

#endif //!CARLAINTERFACE_H