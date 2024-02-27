#include "CARLA_SensorViewConfiguration.h"

std::shared_ptr<osi3::SensorViewConfiguration> SensorViewConfiger::getLastSensorViewConfiguration()
{
	std::shared_ptr<osi3::SensorViewConfiguration> appliedSensorConfig = std::make_shared<osi3::SensorViewConfiguration>();
	auto sensorConfiguration = sensorsByFMU.at(sensorsByFMU.size() - 1);
	switch (sensorConfiguration.type) {
	case GENERIC:
	{
		auto* c = appliedSensorConfig->add_generic_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration.generic_sensor_view_configuration()[0]);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case RADAR:
	{
		auto* c = appliedSensorConfig->add_radar_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration.radar_sensor_view_configuration()[0]);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case LIDAR:
	{
		auto* c = appliedSensorConfig->add_lidar_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration.lidar_sensor_view_configuration()[0]);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case CAMERA:
	{
		auto* c = appliedSensorConfig->add_camera_sensor_view_configuration();;
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration.camera_sensor_view_configuration()[0]);
		c->mutable_sensor_id()->set_value(sensorConfiguration.id);
		break;
	}
	case ULTRASONIC:
	{
		auto* c = appliedSensorConfig->add_ultrasonic_sensor_view_configuration();
		c->CopyFrom(sensorConfiguration.sensorViewConfiguration.ultrasonic_sensor_view_configuration()[0]);
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
	if (sensor.type == SENSORTYPES::GENERIC) {
		//no specific geometrical sensor can be spawned. information will be provided by a (subset of) groundtruth.
		return false;
	}

	carla::ActorId actorId;
	if (!getActorIdFromName(runtimeParameter.ego, actorId)) {//actor is not spawned yet
		return false;
	}
	carla::client::Actor* parent = carla->world->GetActor(actorId).get();

	std::string sensorType = matchSensorType(sensor.type, sensor.prefixed_fmu_variable_name);
	carla::SharedPtr<carla::client::BlueprintLibrary> blueprintLibrary = carla->world->GetBlueprintLibrary();
	auto bp = blueprintLibrary->Find(sensorType);
	carla::client::ActorBlueprint sensorBP = (carla::client::ActorBlueprint) *bp;

	configureBP(sensorBP, sensor);

	carla::geom::Transform transform = Geometry::getInstance()->toCarla(sensor.sensorViewConfiguration.mounting_position());

	auto actor = carla->world->TrySpawnActor(sensorBP, transform, parent);
	if (actor == nullptr) {
		std::cerr << "Could not spawn sensor and attach to actor. Actor Id: " << actorId
			<< " SensorBP: " << (sensorBP).GetId() << std::endl;
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

void SensorViewConfiger::configureBP(carla::client::ActorBlueprint& sensorBP, const Sensor& sensor) {
	switch (sensor.type) {
	case SENSORTYPES::CAMERA:
		configureBPCamera(sensorBP, sensor);
		break;
	case SENSORTYPES::LIDAR:
		configureBPLidar(sensorBP, sensor);
		break;
	case SENSORTYPES::RADAR:
		configureBPRadar(sensorBP, sensor);
		break;
	case SENSORTYPES::ULTRASONIC:

		break;
	case SENSORTYPES::GENERIC:

		break;
	default:
		std::cerr << "Type not supported!" << std::endl;
	}
}

void SensorViewConfiger::configureBPCamera(carla::client::ActorBlueprint& sensorBP, const Sensor& sensor) {
	osi3::CameraSensorViewConfiguration camConf = sensor.sensorViewConfiguration.camera_sensor_view_configuration()[0];
	if (camConf.has_field_of_view_horizontal()) {
		//Horizontal field of view in degrees.
		sensorBP.SetAttribute("fov", std::to_string(Geometry::getInstance()->toCarla(camConf.field_of_view_horizontal())));
	}
	if (camConf.has_number_of_pixels_horizontal()) {
		//Image width in pixels.
		sensorBP.SetAttribute("image_size_x", std::to_string(camConf.number_of_pixels_horizontal()));
	}
	if (camConf.has_number_of_pixels_vertical()) {
		//Image width in pixels.
		sensorBP.SetAttribute("image_size_y", std::to_string(camConf.number_of_pixels_vertical()));
	}
}

void SensorViewConfiger::configureBPRadar(carla::client::ActorBlueprint& sensorBP, const Sensor& sensor) {
	osi3::RadarSensorViewConfiguration radarConf = sensor.sensorViewConfiguration.radar_sensor_view_configuration()[0];
	if (radarConf.has_field_of_view_horizontal()) {
		//Horizontal field of view in degrees, 0 - 360.
		sensorBP.SetAttribute("horizontal_fov", std::to_string(Geometry::getInstance()->toCarla(radarConf.field_of_view_horizontal())));
	}
	if (radarConf.has_field_of_view_vertical()) {
		//Vertical field of view in degrees.
		sensorBP.SetAttribute("vertical_fov", std::to_string(Geometry::getInstance()->toCarla(radarConf.field_of_view_vertical() * 0.5)));
	}
	if (radarConf.has_emitter_frequency()) {
		//Points generated by all lasers per second.
		sensorBP.SetAttribute("points_per_second", std::to_string(radarConf.emitter_frequency()));
	}
}

void SensorViewConfiger::configureBPLidar(carla::client::ActorBlueprint& sensorBP, const Sensor& sensor) {
	osi3::LidarSensorViewConfiguration lidarConf = sensor.sensorViewConfiguration.lidar_sensor_view_configuration()[0];
	if (lidarConf.has_field_of_view_horizontal()) {
		//Range: [0.0, 1.0]
		sensorBP.SetAttribute("horizontal_fov", std::to_string(Geometry::getInstance()->toCarla(lidarConf.field_of_view_horizontal())));
	}
	if (lidarConf.has_field_of_view_vertical()) {
		//Angle in degrees of the highest laser.
		sensorBP.SetAttribute("upper_fov", std::to_string(Geometry::getInstance()->toCarla(lidarConf.field_of_view_vertical() * 0.5)));
		//Angle in degrees of the lowest laser.
		sensorBP.SetAttribute("lower_fov", std::to_string(Geometry::getInstance()->toCarla(lidarConf.field_of_view_vertical() * -0.5)));
	}
	if (lidarConf.has_emitter_frequency()) {
		//Points generated by all lasers per second.
		sensorBP.SetAttribute("points_per_second", std::to_string(lidarConf.emitter_frequency()));
	}
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
		return "sensor.other.radar";
	case SENSORTYPES::ULTRASONIC:
		return "";
	case SENSORTYPES::GENERIC:
		return "";
	}
	return "";
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
