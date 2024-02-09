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
	carla::ActorId actorId;
	if (!getActorIdFromName(runtimeParameter.ego, actorId)) {//actor is not spawned yet
		return false;
	}
	carla::client::Actor* parent = carla->world->GetActor(actorId).get();

	std::string sensorType = matchSensorType(sensor.type, sensor.prefixed_fmu_variable_name);
	auto blueprintLibrary = carla->world->GetBlueprintLibrary();
	auto sensorBP = blueprintLibrary->Find(sensorType);

	carla::geom::Transform transform = determineTransform(sensor.type, sensor.sensorViewConfiguration);

	auto actor = carla->world->TrySpawnActor(*sensorBP, transform, parent);
	if (actor == nullptr) {
		std::cerr << "Could not spawn sensor and attach to actor. Actor Id: " << actorId
			<< " SensorBP: " << (*sensorBP).GetId() << std::endl;
		return false;
	}
	auto sensorActor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
	if (!addSensorIdToStorage(actorId, sensorActor->GetId())) {
		carla->spawnedSensorsOnExternalSpawnedVehicles.push_back(sensorActor->GetId());
	}

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

bool SensorViewConfiger::getActorIdFromName(const std::string& roleName, carla::ActorId& actorId) {

	if (isSpawnedId(roleName, actorId)) {
		return true;
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
						actorId = actor->GetId();
						return true;
					}
				}
			}
		}
	}
	return false;
}

std::string SensorViewConfiger::matchSensorType(const SENSORTYPES& type, const std::string& name) {
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

carla::geom::Transform SensorViewConfiger::determineTransform(const SENSORTYPES& type, const osi3::SensorViewConfiguration& config) {
	osi3::MountingPosition mountingPosition;
	int size(0);
	switch (type) {
	case SENSORTYPES::CAMERA:
		size = config.camera_sensor_view_configuration().size();
		if (size) {
			mountingPosition = config.camera_sensor_view_configuration().at(0).mounting_position();
		}
		break;
	case SENSORTYPES::LIDAR:
		size = config.lidar_sensor_view_configuration().size();
		if (size) {
			mountingPosition = config.lidar_sensor_view_configuration().at(0).mounting_position();
		}
		break;
	case SENSORTYPES::RADAR:
		size = config.radar_sensor_view_configuration().size();
		if (size) {
			mountingPosition = config.radar_sensor_view_configuration().at(0).mounting_position();
		}
		break;
	case SENSORTYPES::ULTRASONIC:
		size = config.ultrasonic_sensor_view_configuration().size();
		if (size) {
			mountingPosition = config.ultrasonic_sensor_view_configuration().at(0).mounting_position();
		}
		break;
	case SENSORTYPES::GENERIC:
		size = config.generic_sensor_view_configuration().size();
		if (size) {
			mountingPosition = config.generic_sensor_view_configuration().at(0).mounting_position();
		}
		break;
	}
	return Geometry::getInstance()->toCarla(mountingPosition);
}

bool SensorViewConfiger::addSensorIdToStorage(const carla::ActorId& actorId, const carla::ActorId& sensorId) {
	for (auto& vehicle : carla->spawnedVehiclesByCarlaOSIService) {
		if (vehicle.second.vehicle == actorId) {
			vehicle.second.sensors.push_back(sensorId);
			return true;
		}
	}
	return false;
}

void SensorViewConfiger::deleteSpawnedSensorsOnExternalSpawnedVehicles() {
	for (auto& sensorId : carla->spawnedSensorsOnExternalSpawnedVehicles) {
		auto sensorActor = carla->world->GetActor(sensorId);
		if (sensorActor != nullptr) { sensorActor->Destroy(); };
	}
	carla->spawnedSensorsOnExternalSpawnedVehicles.clear();
}
