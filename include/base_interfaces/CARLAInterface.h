#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <variant>
#include <string>
#include "base_interfaces/BaseSystemInterface.h"
#include "configreader/BaseConfigVariants.h"

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
#include "osi_trafficlight.pb.h"
#include "osi_trafficsign.pb.h"
#include "osi_version.pb.h"


class CARLAInterface : public BaseSystemInterface
{
	std::string host;
	uint16_t port;
	std::unique_ptr<carla::client::World> world;
	std::unique_ptr<carla::client::Client> client;
	std::map<std::string, carla::ActorId> actorRole2IDMap;
	std::set<carla::ActorId> activeActors;
	carla::time_duration transactionTimeout;
	double deltaSeconds;
	std::shared_ptr<osi3::GroundTruth> mapTruth;

public:

	/**
	Read configuration for this base simulator interface.
	\param config the decoding struct
	\return success status
	*/
	virtual int readConfiguration(baseConfigVariants_t config) override;
	/**
	Connect with host/port information from corresponding fields
	\return Success status.
	*/
	virtual int initialise() override;
	/**
	Perform a simulation step. Will perform a tick of deltaSeconds, as given in the configuration
	\return Time in seconds advanced during step
	*/
	virtual double doStep() override;

	virtual int getIntValue(std::string base_name) override;
	virtual bool getBoolValue(std::string base_name) override;
	virtual float getFloatValue(std::string base_name) override;
	virtual double getDoubleValue(std::string base_name) override;
	virtual std::string getStringValue(std::string base_name) override;

	virtual int setIntValue(std::string base_name, int value) override;
	virtual int setBoolValue(std::string base_name, bool value) override;
	virtual int setFloatValue(std::string base_name, float value) override;
	virtual int setDoubleValue(std::string base_name, double value) override;
	virtual int setStringValue(std::string base_name, std::string value) override;

private:

	std::string_view getPrefix(std::string_view name);
	osi3::Timestamp parseTimestamp();
	// prepare a GroundTruth object with values from the current map which won't change 
	//TODO return type
	void parseStationaryMapObjects();
	osi3::GroundTruth parseWorldToGroundTruth();
	std::vector<osi3::SensorView> parseSensorActors();

};

#endif // !DOMINIONINTERFACE_H