/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLASENSORVIEWCONFIGURATION_H
#define CARLASENSORVIEWCONFIGURATION_H

#include <osi_sensorviewconfiguration.pb.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>

#include "boost/bimap.hpp"

#include "CARLA_Module.h"
#include "carla_osi/Geometry.h"

class SensorViewConfiger : public CARLAModule {
public:

	/**
	Theses sensors are defined by the sensorview configuration of a FMU.
	They need an answer in form of a osi3::sensorviewconfiguration before the simulation starts.
	It is possible, that the sensors can not be spawned instantly, since the ego vehicle itself might be spawned later.
	*/
    std::vector<OSTARSensorConfiguration> sensorsByFMU;
	/**
	Theses sensors are defined by the configuration file in CoSiMa.
	They do not need an answer in form of a osi3::sensorviewconfiguration.
	It is possible, that the sensors can not be spawned instantly, since the ego vehicle itself might be spawned later.
	*/
	std::vector<OSTARSensorConfiguration> sensorsByUser;

	/**
	Send applied SensorViewConfiguration for sensor.
	\return The sensor's configuration as osi3::SensorViewConfiguration, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<osi3::SensorViewConfiguration> getLastSensorViewConfiguration();

	/*
	Destroy all sensors spawmed om external spawned vehicles
	*/
	void deleteSpawnedSensorsOnExternalSpawnedVehicles();

	/*
	Finds any mounting position of a generic sensor.
	Sensor postions by users will be preferred before sensors by fmus.
	*/
	const osi3::MountingPosition getMountingPositionOfAnyGenericSensor();

	bool getActorIdFromName(const std::string& roleName, carla::ActorId& actorId);
	
	/*
	Return name of sensor in Carla.
	*/
	std::string matchSensorType(const SENSORTYPES& type, const std::string& name);

	/*
	return false, if sensor is not attached to self spawned vehicle
	*/
	bool addSensorIdToStorage(const carla::ActorId& vehicle, const carla::ActorId& sensorId);
	/*
	Configure a blueprint depending on the given sensor configuration
	*/
	void configureBP(carla::client::ActorBlueprint& sensorBP, const OSTARSensorConfiguration& sensorConfig);

private:
	void configureBPCamera(carla::client::ActorBlueprint& sensorBP, const OSTARSensorConfiguration& sensorConfig);
	void configureBPLidar(carla::client::ActorBlueprint& sensorBP, const OSTARSensorConfiguration& sensorConfig);
	void configureBPRadar(carla::client::ActorBlueprint& sensorBP, const OSTARSensorConfiguration& sensorConfig);
};

#endif //!CARLASENSORVIEWCONFIGURATION_H
