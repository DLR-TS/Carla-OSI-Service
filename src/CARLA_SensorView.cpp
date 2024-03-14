#include "CARLA_SensorView.h"

std::shared_ptr<osi3::SensorView> SensorViewer::getSensorViewGroundTruth(const std::string& varName) {

	auto sensorView = std::make_shared<osi3::SensorView>();
	auto groundTruth = sensorView->mutable_global_ground_truth();

	auto latestgt = groundTruthCreator->getLatestGroundTruth();
	groundTruth->CopyFrom(*latestgt);

	//host_vehicle_id
	if (groundTruth->has_host_vehicle_id()) {
		sensorView->mutable_host_vehicle_id()->set_value(groundTruth->host_vehicle_id().value());
	}

	// if any, set sensor mounting positions
	osi3::MountingPosition ms = sensorViewConfiger->getMountingPositionOfAnyGenericSensor();
	sensorView->mutable_mounting_position()->CopyFrom(ms);

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

	if (runtimeParameter->verbose) {
		std::cout << sensorView->DebugString() << std::endl;
	}
	return sensorView;
}

std::shared_ptr<const osi3::SensorView> SensorViewer::getSensorView(const std::string& sensorName)
{
	// mutex scope: using a shared lock - read only access
	std::unique_lock<std::mutex> lock(sensorCache_mutex);
	if (runtimeParameter->verbose) {
		for (auto& s : sensorCache) {
			std::cout << "SensorCache: " << s.first << std::endl;
		}
	}

	auto iter = sensorCache.find(sensorName);
	if (iter == sensorCache.end() || iter->second == nullptr) {
		if (runtimeParameter->verbose) {
			std::cout << __FUNCTION__ << "No sensorview by carla sensors. Use generic sensor view." << std::endl;
		}
		return getSensorViewGroundTruth(sensorName);
	}

	std::shared_ptr<osi3::SensorView> sv = iter->second;
	auto gt = sv->mutable_global_ground_truth();
	auto latestgt = groundTruthCreator->getLatestGroundTruth();
	gt->CopyFrom(*latestgt);

	sv->mutable_timestamp()->CopyFrom(latestgt->timestamp());
	sv->mutable_host_vehicle_id()->CopyFrom(latestgt->host_vehicle_id());
	return sv;
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
						if (runtimeParameter->carlaSensors && !isSpawnedActorId(addedActor) && 0 == actor->GetTypeId().rfind("sensor.", 0)) {
							std::cout << "Add already spawned sensor in Carla" << actor->GetTypeId() << std::endl;
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							std::string index = "OSMPSensorView" + sensorCache.size();
							std::cout << "Sensorview can be requested by base_name: " << index << std::endl;
							OSTARSensorConfiguration sensorConfig;
							sensorConfig.prefixed_fmu_variable_name = index;
							sensorCache.emplace(index, nullptr);
							sensor->Listen([this, sensor, sensorConfig](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData, sensorConfig); });
						}
					}
					break;
				}
			}
		}
	}
}

void SensorViewer::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData, const OSTARSensorConfiguration sensorConfig)
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

	if (sensorConfig.prefixed_fmu_variable_name == "") {
		std::cerr << __FUNCTION__ << ": received event without a name to save the data to. Sensor: " << sensor->GetTypeId() << std::endl;
		return;
	}

	std::unique_ptr<osi3::SensorView> sensorView = std::make_unique<osi3::SensorView>();

	auto typeID = sensor->GetTypeId();
	//substring of typeID: sensor.camera.rgb -> camera.rgb
	std::string sensorType(&typeID[7]);

	if (0 == sensorType.rfind("camera.rgb", 0))
	{
		if (runtimeParameter->carlasensortypes.find(CAMERA) == runtimeParameter->carlasensortypes.end()) {
			return;
		}
		auto cameraSensorView = CarlaUtility::toOSICamera(sensor, sensorData, sensorConfig);
		sensorView->mutable_camera_sensor_view()->AddAllocated(cameraSensorView);
	}
	else if (0 == sensorType.rfind("lidar.ray_cast_semantic", 0))
	{
		if (runtimeParameter->carlasensortypes.find(LIDAR) == runtimeParameter->carlasensortypes.end()) {
			return;
		}
		auto lidarSensorView = CarlaUtility::toOSILidar(sensor, sensorData, sensorConfig);
		sensorView->mutable_lidar_sensor_view()->AddAllocated(lidarSensorView);
	}
	else if (0 == sensorType.rfind("other.radar", 0))
	{
		if (runtimeParameter->carlasensortypes.find(RADAR) == runtimeParameter->carlasensortypes.end()) {
			return;
		}
		auto radarSensorView = CarlaUtility::toOSIRadar(sensor, sensorData, sensorConfig);
		sensorView->mutable_radar_sensor_view()->AddAllocated(radarSensorView);
	}
	else if (runtimeParameter->verbose) {
		std::cerr << "SensorEventAction called for unsupported sensor type" << std::endl;
	}
	sensorCache[sensorConfig.prefixed_fmu_variable_name] = std::move(sensorView);
	if (runtimeParameter->verbose) {
		std::cout << "Update " << sensorType << " with index " << sensorConfig.prefixed_fmu_variable_name << "." << std::endl;
	}
}

void SensorViewer::trySpawnSensors() {
	for (auto& sensorConfig : sensorViewConfiger->sensorsByFMU) {
		if (!sensorConfig.spawned) {
			//sensorConfig.spawned = trySpawnSensor(sensorViewer, sensorConfig);
		}
	}
	for (auto& sensorConfig : sensorViewConfiger->sensorsByUser) {
		if (!sensorConfig.spawned) {
			//sensorConfig.spawned = trySpawnSensor(sensorViewer, sensorConfig);
		}
	}
}

bool SensorViewer::trySpawnSensor(const OSTARSensorConfiguration& sensorConfig) {
	if (sensorConfig.type == SENSORTYPES::GENERIC) {
		//no specific geometrical sensor can be spawned. information will be provided by a (subset of) groundtruth.
		return false;
	}

	carla::ActorId actorId;
	std::string parentActorName = "";
	carla::client::Actor* parent = nullptr;

	if (sensorConfig.parent != "world" && sensorConfig.parent != "World") {
		if (sensorConfig.parent == "") {
			parentActorName = runtimeParameter->ego;
		}
		else {
			parentActorName = sensorConfig.parent;
		}

		if (!sensorViewConfiger->getActorIdFromName(parentActorName, actorId)) {//actor is not spawned yet
			return false;
		}
		parent = carla->world->GetActor(actorId).get();
	}
	else if (runtimeParameter->verbose) {
		std::cout << "Spawn sensor as a static sensor in world." << std::endl;
	}

	std::string sensorType = sensorViewConfiger->matchSensorType(sensorConfig.type, sensorConfig.prefixed_fmu_variable_name);
	carla::SharedPtr<carla::client::BlueprintLibrary> blueprintLibrary = carla->world->GetBlueprintLibrary();
	auto bp = blueprintLibrary->Find(sensorType);
	carla::client::ActorBlueprint sensorBP = (carla::client::ActorBlueprint) *bp;

	sensorViewConfiger->configureBP(sensorBP, sensorConfig);

	carla::geom::Transform transform = Geometry::getInstance()->toCarla(sensorConfig.sensorViewConfiguration.mounting_position());

	auto actor = carla->world->TrySpawnActor(sensorBP, transform, parent);
	if (actor == nullptr) {
		std::cerr << "Could not spawn sensor. Attach to actor with Actor Id: " << actorId
			<< " SensorBP: " << (sensorBP).GetId() << std::endl;
		return false;
	}
	auto sensorActor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
	if (!sensorViewConfiger->addSensorIdToStorage(actorId, sensorActor->GetId())) {
		carla->spawnedSensorsOnExternalSpawnedVehicles.push_back(sensorActor->GetId());
	}

	sensorCache.emplace(sensorConfig.prefixed_fmu_variable_name, nullptr);
	sensorActor->Listen(
		[this, sensorActor, sensorConfig]
	(carla::SharedPtr<carla::sensor::SensorData> sensorData) {
		sensorEventAction(sensorActor, sensorData, sensorConfig);
	}
	);
	return true;
}
