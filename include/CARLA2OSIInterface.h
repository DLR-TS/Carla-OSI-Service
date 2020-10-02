#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <charconv>
#include <variant>
#include <string>

#include "Utility.h"

#include "pugiXML.hpp"
#include <boost/bimap.hpp>
#include <boost/foreach.hpp>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/road/Lane.h>
#include <carla/sensor/data/Image.h>

#include "osi_common.pb.h"
#include "osi_datarecording.pb.h"
#include "osi_detectedlane.pb.h"
#include "osi_detectedobject.pb.h"
#include "osi_detectedoccupant.pb.h"
#include "osi_detectedroadmarking.pb.h"
#include "osi_detectedtrafficlight.pb.h"
#include "osi_detectedtrafficsign.pb.h"
#include "osi_environment.pb.h"
#include "osi_featuredata.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_hostvehicledata.pb.h"
#include "osi_lane.pb.h"
#include "osi_object.pb.h"
#include "osi_occupant.pb.h"
#include "osi_roadmarking.pb.h"
#include "osi_sensordata.pb.h"
#include "osi_sensorspecific.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficlight.pb.h"
#include "osi_trafficsign.pb.h"
#include "osi_trafficupdate.pb.h"
#include "osi_version.pb.h"
#include "sl45_motioncommand.pb.h"
#include "sl45_vehiclecommunicationdata.pb.h"

/**
* \var host
* host name or ip
* \var port
* port
* \var transactionTimeout
* transaction timeout in seconds
* \var deltaSeconds
* simulation time delta per tick
*/
struct CARLA2OSIInterfaceConfig {
public:
	std::string host;
	uint16_t port;
	double transactionTimeout;
	double deltaSeconds;
};

class CARLA2OSIInterface
{
	std::string host;
	uint16_t port;
	std::unique_ptr<carla::client::World> world;
	std::unique_ptr<carla::client::Client> client;
	// contains actor ids an the value of their role_name attribute. Does not contain actors without a role
	boost::bimap<std::string, carla::ActorId> actorRole2IDMap;
	// contains OSI messages (values) for variable names (keys). Can be used for output->input chaining without translating a message into Carla's world first if no corresponding role_name is present
	std::map<std::string, std::string> varName2MessageMap;
	// contains all actor ids reported by Carla
	std::set<carla::ActorId> activeActors;
	carla::time_duration transactionTimeout;
	double deltaSeconds;
	std::shared_ptr<osi3::GroundTruth> mapTruth;
	// OpenDRIVE xml representation of the map
	pugi::xml_document xodr;

public:

	~CARLA2OSIInterface() {
		if (world) {
			//Unfreeze the server and Unreal Editor, useful for development
			auto settings = world->GetSettings();
			settings.synchronous_mode = false;
			world->ApplySettings(settings);
			world->Tick(transactionTimeout);
		}
	};

	/**
	Read configuration for this base simulator interface.
	\param config the decoding struct
	\return success status
	*/
	virtual int readConfiguration(CARLA2OSIInterfaceConfig& config);
	/**
	Connect with host/port information from corresponding fields
	\return Success status.
	*/
	virtual int initialise();
	/**
	Perform a simulation step. Will perform a tick of deltaSeconds, as given in the configuration
	\return Time in seconds advanced during step
	*/
	virtual double doStep();

	virtual int getIntValue(std::string base_name);
	virtual bool getBoolValue(std::string base_name);
	virtual float getFloatValue(std::string base_name);
	virtual double getDoubleValue(std::string base_name);
	virtual std::string getStringValue(std::string base_name);

	virtual int setIntValue(std::string base_name, int value);
	virtual int setBoolValue(std::string base_name, bool value);
	virtual int setFloatValue(std::string base_name, float value);
	virtual int setDoubleValue(std::string base_name, double value);
	virtual int setStringValue(std::string base_name, std::string value);

private:

	std::string_view getPrefix(std::string_view name);
	osi3::Timestamp* parseTimestamp();
	// prepare a GroundTruth object with values from the current map which won't change 
	//TODO return type
	void parseStationaryMapObjects();
	osi3::GroundTruth* parseWorldToGroundTruth();

	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData);

	//output
	void sendTrafficCommand(carla::ActorId actorId);

	//input
	/**
	Read traffic update message from traffic participant and update position, rotation, velocity and lightstate of CARLA actor.
	\param actorId
	\return success indicator
	*/
	int receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate);

	/**
	Read motion command message from ego vehicle and update position, rotation and velocity of CARLA actor.
	\param actorId
	\return success indicator
	*/
	int receiveMotionCommand(setlevel4to5::MotionCommand& motionCommand);
};

#endif CARLAINTERFACE_H
