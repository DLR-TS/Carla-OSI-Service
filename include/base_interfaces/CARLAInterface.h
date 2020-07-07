#ifndef CARLAINTERFACE_H
#define CARLAINTERFACE_H

#include <variant>
#include <string>
#include "base_interfaces/BaseSystemInterface.h"
#include "configreader/BaseConfigVariants.h"

#include <carla/client/ActorBlueprint.h>
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

class CARLAInterface : public BaseSystemInterface
{
	std::string host;
	uint16_t port;
	std::unique_ptr<carla::client::World> world;
	std::unique_ptr<carla::client::Client> client;
	carla::time_duration transactionTimeout;

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
	virtual int connect() override;
	/**
	Disconnect the previously established connection.
	\return Success status.
	*/
	virtual int disconnect() override;

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


};

#endif // !DOMINIONINTERFACE_H