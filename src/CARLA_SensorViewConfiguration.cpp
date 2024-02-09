#include "CARLA_SensorViewConfiguration.h"

std::shared_ptr<osi3::SensorViewConfiguration> SensorViewConfiger::getLastSensorViewConfiguration()
{
	std::shared_ptr<osi3::SensorViewConfiguration> appliedSensorConfig = std::make_shared<osi3::SensorViewConfiguration>();
	auto sensorConfiguration = sensorsByFMU.at(sensorsByFMU.size() - 1);
	switch (sensorConfiguration.type) {
	case GENERIC:
	{
		auto* c = appliedSensorConfig->add_generic_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case RADAR:
	{
		auto* c = appliedSensorConfig->add_radar_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case LIDAR:
	{
		auto* c = appliedSensorConfig->add_lidar_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case CAMERA:
	{
		auto* c = appliedSensorConfig->add_camera_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case ULTRASONIC:
	{
		auto* c = appliedSensorConfig->add_ultrasonic_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	}
	return appliedSensorConfig;
}

void SensorViewConfiger::trySpawnSensors(std::shared_ptr<SensorViewer> sensorViewer) {
	for (auto& sensor : sensorsByFMU) {
		if (!sensor.spawned) {
			sensor.spawned = trySpawnSensor(sensorViewer, sensor);
		}
	}
	for (auto& sensor : sensorsByUser) {
		if (!sensor.spawned) {
			sensor.spawned = trySpawnSensor(sensorViewer, sensor);
		}
	}
}

bool SensorViewConfiger::trySpawnSensor(std::shared_ptr<SensorViewer> sensorViewer, const Sensor& sensor) {
	auto actorId = getActorIdFromName(runtimeParameter.ego);
	if (actorId == -1) {//actor is not spawned
		return false;
	}
	carla::client::Actor* parent = carla->world->GetActor(actorId).get();

	std::string sensorType = matchSensorType(sensor.type, sensor.prefixed_fmu_variable_name);
	auto blueprintLibrary = carla->world->GetBlueprintLibrary();
	auto sensorBP = blueprintLibrary->Find(sensorType);

	carla::geom::Transform transform;
	//TODO set transform
	//sensor.sensorViewConfiguration

	auto actor = carla->world->TrySpawnActor(*sensorBP, transform, parent);
	if (actor == nullptr) {
		std::cerr << "Could not spawn sensor and attach to actor. Actor Id: " << actorId
			<< " SensorBP: " << (*sensorBP).GetId() << std::endl;
		return false;
	}
	auto sensorActor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);

	std::string index = sensor.prefixed_fmu_variable_name;
	sensorViewer->sensorCache.emplace(index, nullptr);
	sensorActor->Listen(
		[this, sensorViewer, sensorActor, index]
	(carla::SharedPtr<carla::sensor::SensorData> sensorData) {
		sensorViewer->sensorEventAction(sensorActor, sensorData, index);
	}
	);
	return true;
}

carla::ActorId SensorViewConfiger::getActorIdFromName(const std::string& roleName) {
	uint32_t actorId;
	if (isSpawnedID(roleName, actorId)) {
		return actorId;
	}
	
	//if not spawned by Carla-OSI-Service, check all actors with role_name attribute

	auto worldActors = carla->world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);
			vehicleActor->GetAttributes();
			for (auto& attribute : vehicleActor->GetAttributes()) {
				if ("role_name" == attribute.GetId()) {
					if (attribute.GetValue() == roleName) {
						return actor->GetId();
					}
				}
			}
		}
	}
	return -1;
}

std::string SensorViewConfiger::matchSensorType(const SENSORTYPES type, const std::string& name) {
	switch (type) {
	case SENSORTYPES::CAMERA:
		return "sensor.camera.rgb";
	case SENSORTYPES::LIDAR:
		return "sensor.lidar.ray_cast";
	case SENSORTYPES::RADAR:
		return "";
	case SENSORTYPES::ULTRASONIC:
		return "";
	case SENSORTYPES::GENERIC:
		return "";
	}
	return "";
}

bool SensorViewConfiger::isSpawnedID(const std::string& roleName, uint32_t& actorId) {
	if (isNumeric(roleName)) {
		auto vehicleID = carla->spawnedVehiclesByCarlaOSIService.find(std::stoi(roleName));
		if (vehicleID != carla->spawnedVehiclesByCarlaOSIService.end()) {
			actorId = vehicleID->second;
			return true;
		}
	}
	return false;
}

bool SensorViewConfiger::isNumeric(const std::string& str) {
	for (char c : str) {
		if (!std::isdigit(c)) {
			return false;
		}
	}
	return true;
}
