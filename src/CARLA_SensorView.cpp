#include "CARLA_SensorView.h"

std::shared_ptr<osi3::SensorView> SensorViewer::getSensorViewGroundTruth(const std::string& varName) {
	// create empty sensor view
	auto sensorView = std::make_shared<osi3::SensorView>();
	// create empty ground truth as part of sensor view
	auto groundTruth = sensorView->mutable_global_ground_truth();
	// copy latest ground truth into previously created ground truth
	groundTruth->MergeFrom(*groundTruthCreator->getLatestGroundTruth());

	//host_vehicle_id
	if (groundTruth->has_host_vehicle_id()) {
		sensorView->mutable_host_vehicle_id()->set_value(groundTruth->host_vehicle_id().value());
	}

	// if defined, set sensor mounting positions
	//TODO move this code to somewhere else
	/*auto iter = sensorMountingPositionMap.find(varName);
	if (sensorMountingPositionMap.end() != iter) {
		if (runtimeParameter.verbose)
		{
			std::cout << "Searched successfully for sensor " << varName << " Copy mounting position to sensorview message." << std::endl;
		}
		copyMountingPositions(iter->second, sensorView);
	}
	else if (runtimeParameter.verbose)
	{
		std::cout << "No sensor found with name: " << varName << " Can not set mounting position.\n";
		if (sensorMountingPositionMap.size() != 0) {
			std::cout << "Available sensors are: ";
			for (auto& positions : sensorMountingPositionMap) {
				std::cout << positions.first << " ";
			}
			std::cout << std::endl;
		}
		else {
			std::cout << "No sensor positions are configured!" << std::endl;
		}
	}
	*/

	// find or generate id for the named sensor
	auto id = sensorIds.find(varName);
	if (sensorIds.end() != id) {
		sensorView->mutable_sensor_id()->set_value(id->second);
	}
	else {
		carla_osi::id_mapping::IDUnion sensorId;
		//TODO make sure the type is not defined in CarlaUtility::CarlaUniqueID_e
		sensorId.type = 100;
		sensorId.id = (uint32_t)sensorIds.size() + 1;
		sensorIds[varName] = sensorId.value;
		sensorView->mutable_sensor_id()->set_value(sensorId.value);
	}

	if (runtimeParameter.verbose) {
		std::cout << sensorView->DebugString() << std::endl;
	}
	return sensorView;
}

std::shared_ptr<const osi3::SensorView> SensorViewer::getSensorView(const std::string& sensorName)
{
	// mutex scope: using a shared lock - read only access
	std::unique_lock<std::mutex> lock(sensorCache_mutex);
	auto iter = sensorCache.find(sensorName);
	if (iter == sensorCache.end()) {
		return nullptr;
	}
	return iter->second;
}

void SensorViewer::fetchActorsFromCarla() {

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
		std::unique_lock<std::mutex> lock(actorRole2IDMap_mutex);
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
					if (!attribute.GetValue().empty()) {
						auto value = boost::bimap<std::string, carla::ActorId>::value_type(attribute.GetValue(), addedActor);
						actorRole2IDMap.insert(value);

						// if actor is of type sensor, add sensor update listener to receive latest sensor data
						if (runtimeParameter.carlaSensors && 0 == actor->GetTypeId().rfind("sensor.", 0)) {
							std::cout << "Add already spawned sensor in Carla"  << actor->GetTypeId() << std::endl;
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							std::string index =  "OSMPSensorView" + sensorCache.size();
							std::cout << "Sensorview can be requested by base_name: " << index << std::endl;
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

void SensorViewer::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData, std::string index)
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
	std::string sensorType(&typeID[7]);

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
		std::cerr << "SensorEventAction called for unsupported sensor type" << std::endl;
	}
	sensorCache[index] = std::move(sensorView);
	if (runtimeParameter.verbose) {
		std::cout << "Update " << sensorType << " with index " << index << "." << std::endl;
	}
}
