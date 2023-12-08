#include "CARLA_OSI_Interface.h"

void CARLAOSIInterface::fetchActorsFromCarla() {

	// track actors added/removed inside Carla
	std::set<carla::ActorId> worldActorIDs, addedActors, removedActors;
	auto worldActors = carla->world->GetActors();
	// compare actor ids, not actors
	for (auto actor : *worldActors)
	{
		worldActorIDs.insert(actor->GetId());
	}
	CarlaUtility::twoWayDifference(
		activeActors.begin(), activeActors.end(),
		worldActorIDs.begin(), worldActorIDs.end(),
		std::inserter(addedActors, addedActors.end()),
		std::inserter(removedActors, removedActors.end()));

	//TODO implement reactions
	{//mutex scope
		// using a unique lock - access not limited to read
		std::unique_lock lock(actorRole2IDMap_mutex);
		for (auto removedActor : removedActors) {
			if (actorRole2IDMap.right.count(removedActor)) {
				actorRole2IDMap.right.erase(removedActor);
			}
			activeActors.erase(removedActor);
		}
		for (auto addedActor : addedActors) {
			activeActors.insert(addedActor);
			auto actor = carla->world->GetActor(addedActor);
			auto attributes = actor->GetAttributes();
			for (auto attribute : attributes) {
				if ("role_name" == attribute.GetId()) {
					if (!std::empty(attribute.GetValue())) {
						auto value = boost::bimap<std::string, carla::ActorId>::value_type(attribute.GetValue(), addedActor);
						actorRole2IDMap.insert(value);

						// if actor is of type sensor, add sensor update listener to receive latest sensor data
						if (runtimeParameter.carlaSensors && 0 == actor->GetTypeId().rfind("sensor.", 0)) {
							std::cout << "add sensor "  << actor->GetTypeId() << std::endl;
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							int index = (int)sensorCache.size();
							sensorCache.emplace(index, nullptr);
							sensor->Listen([this, sensor, index](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData, index); });
						}
					}
					break;
				}
			}
		}
	}
}

std::shared_ptr<const osi3::SensorViewConfiguration> CARLAOSIInterface::getSensorViewConfiguration(const std::string& sensor)
{
	//string has format of: OSMPSensorViewConfigurationX
	std::string index_string(&sensor[27]);
	int index = std::stoi(index_string);
	//todo a
	return nullptr;
}

std::shared_ptr<const osi3::SensorView> CARLAOSIInterface::getSensorView(const std::string& sensor)
{
	//string has format of: OSMPSensorViewX
	std::string index_string(&sensor[14]);
	int index = std::stoi(index_string);
	{
		// mutex scope: using a shared lock - read only access
		std::shared_lock lock(sensorCache_mutex);
		auto iter = sensorCache.find(index);
		if (iter == sensorCache.end()) {
			return nullptr;
		}
		return iter->second;
	}
}


void CARLAOSIInterface::clearData()
{
	if (!carla->world) {
		std::cerr << "No world" << std::endl;
		throw new std::exception();
	}
	{//mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex, sensorCache_mutex);
		actorRole2IDMap.clear();
		sensorCache.clear();
	}
	staticMapTruth->Clear();
	//TODO December
	//parseStationaryMapObjects();
}

void CARLAOSIInterface::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData, int index)
{
	if (!carla->world) {
		// Local world object has been destroyed and thus probably also the CARLA OSI interface, but client is still sending
		// Stop listening to this sensor
		sensor->Stop();
		return;
	}
	if (carla->world->GetId() != sensor->GetWorld().GetId()) {
		std::cerr << __FUNCTION__ << ": received event for wrong world" << std::endl;
		return;
	}

	std::unique_ptr<osi3::SensorView> sensorView = std::make_unique<osi3::SensorView>();

	auto typeID = sensor->GetTypeId();
	//substring of typeID: sensor.camera.rgb -> camera.rgb
	std::string_view sensorType(&typeID[7]);

	if (0 == sensorType.rfind("camera.rgb", 0))
	{
		if (runtimeParameter.carlasensortypes.find(CAMERA) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(sensorData);
		auto cameraSensorView = CarlaUtility::toOSICamera(sensor, image);
		sensorView->mutable_camera_sensor_view()->AddAllocated(cameraSensorView);
	}
	else if (0 == sensorType.rfind("lidar.ray_cast", 0))
	{
		if (runtimeParameter.carlasensortypes.find(LIDAR) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(sensorData);
		auto lidarSensorView = CarlaUtility::toOSILidar(sensor, measurement);
		sensorView->mutable_lidar_sensor_view()->AddAllocated(lidarSensorView);
	}
	else if (0 == sensorType.rfind("other.radar", 0))
	{
		if (runtimeParameter.carlasensortypes.find(RADAR) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::RadarMeasurement>(sensorData);
		auto radarSensorView = CarlaUtility::toOSIRadar(sensor, measurement);
		sensorView->mutable_radar_sensor_view()->AddAllocated(radarSensorView);
	}
	else if (runtimeParameter.verbose) {
		std::cerr << "CARLAOSIInterface::sensorEventAction called for unsupported sensor type" << std::endl;
	}
	sensorCache[index] = std::move(sensorView);
	if (runtimeParameter.verbose) {
		std::cout << "Update " << sensorType << " with index " << index << "." << std::endl;
	}
}



int CARLAOSIInterface::receiveSensorViewConfigurationRequest(osi3::SensorViewConfiguration& sensorViewConfiguration) {
	//todo a
	for (auto& cameraSensorConfiguration : sensorViewConfiguration.camera_sensor_view_configuration()) {
		// todo
	}
	for (auto& lidarSensorConfiguration : sensorViewConfiguration.lidar_sensor_view_configuration()) {
		// todo
	}
	for (auto& radarSensorConfiguration : sensorViewConfiguration.radar_sensor_view_configuration()) {
		// todo
	}
	for (auto& ultrasonicSensorConfiguration : sensorViewConfiguration.ultrasonic_sensor_view_configuration()) {
		// todo
	}
	for (auto& genericSensorConfiguration : sensorViewConfiguration.generic_sensor_view_configuration()) {
		// todo
	}
	//sensorViewConfiguration.field_of_view_horizontal
	//sensorViewConfiguration.field_of_view_vertical
	//sensorViewConfiguration.update_cycle_offset
	//sensorViewConfiguration.mounting_position
	//sensorViewConfiguration.mounting_position_rmse
	//sensorViewConfiguration.range
	//sensorViewConfiguration.update_cycle_time
	//sensorViewConfiguration.update_cycle_offset

	//spawn sensor and attach to vehicle, vehicle should have name: runtimeparameter.ego
	//add cache entry from fetchActorsFromCarla() and remove that function and its then useless subfunctions
	//save applied sensorviewconfiguration so that getSensorViewConfiguration() can retrieve the information
	return 0;
}
