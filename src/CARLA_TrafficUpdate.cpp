#include "CARLA_TrafficUpdate.h"

void TrafficUpdater::initialise(std::shared_ptr<RuntimeParameter> runtimeParams, std::shared_ptr<CARLAInterface> carla) {
	CARLAModule::initialise(runtimeParams, carla);
	if (runtimeParameter->replay.enabled) {
		saveBoundingBoxesOfAvailableVehicles();
	}
}

void TrafficUpdater::saveBoundingBoxesOfAvailableVehicles() {
	auto vehicleLibrary = carla->world->GetBlueprintLibrary()->Filter("vehicle.*");
	carla::geom::Transform transform;//location 0,0,0 rotation 0,0,0
	for (auto vehicle : *vehicleLibrary) {
		auto temp_actor = carla->world->TrySpawnActor(vehicle, transform);
		if (temp_actor == nullptr) {//map has object near 0 0 z
			carla::geom::Location location_alternative(0, 0, 10000);
			transform = carla::geom::Transform(location_alternative, carla::geom::Rotation(0, 0, 0));
			temp_actor = carla->world->TrySpawnActor(vehicle, transform);
			if (temp_actor == nullptr) {
				std::cerr << "Can not spawn vehicle. Tried two locations with x,y,z: 0,0,0 and 0,0,10000" << std::endl;
			}
		}
		auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(temp_actor);
		auto bbox = vehicleActor->GetBoundingBox();
		replayVehicleBoundingBoxes.emplace_back(vehicle.GetId(), bbox.extent);
		if (runtimeParameter->verbose) {
			std::cout << "bbox: " << vehicle.GetId() << " Length:" << bbox.extent.x * 2 << " Width:" << bbox.extent.y * 2 << " Height:" << bbox.extent.z * 2 << std::endl;
		}
		carla->world->Tick(carla->client->GetTimeout());
		temp_actor->Destroy();
	}
	if (runtimeParameter->verbose) {
		std::cout << "Number of possible vehicle bounding boxes: " << replayVehicleBoundingBoxes.size() << std::endl;
	}
}

int TrafficUpdater::receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate) {
	if (trafficUpdate.update_size() == 0) {
		std::cerr << __FUNCTION__ << " No update." << std::endl;
		return 1;
	}

	carla::ActorId actorId;
	std::vector<uint32_t> listOfUpdatedVehicles;

	for (auto& update : trafficUpdate.update()) {
#if defined(_WIN32) && (_MSC_VER >= 1910) || defined(__linux__) && __cplusplus >= 201703L
		actorId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
#elif defined(_WIN32) && (_MSC_VER >= 1600) || defined(__linux__) && __cplusplus >= 201103L
		actorId = boost::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
#endif
		auto actor = carla->world->GetActor(actorId);
		bool spawned = false;
		if (runtimeParameter->replay.enabled) {
			std::tie(spawned, actor) = spawnVehicleIfNeeded(update, actorId);
		}
		if (actor == nullptr) {
			std::cout << "Actor not found! No position updates will be done!" << std::endl;
			return 0;
		}
		listOfUpdatedVehicles.push_back(uint32_t(update.id().value()));
		if (!spawned) { //let vehicle spawn and run with doStep, then do normal traffic updates
			applyTrafficUpdateToActor(update, actor, actorId);
		}
	}
	removeSpawnedVehiclesIfNotUpdated(listOfUpdatedVehicles);
	return 0;
}

void TrafficUpdater::removeSpawnedVehiclesIfNotUpdated(std::vector<uint32_t>& listOfUpdatedVehicles) {
	auto it = carla->spawnedVehiclesByCarlaOSIService.begin();
	while (it != carla->spawnedVehiclesByCarlaOSIService.end()) {

		if (std::find(listOfUpdatedVehicles.begin(), listOfUpdatedVehicles.end(), it->first) == listOfUpdatedVehicles.end()) {
			std::cout << "No update for vehicle: " << unsigned(it->first) << " Will stop the display of this vehicle." << std::endl;
			deleteSpawnedVehiclesWithSensors(it->first, it->second);
			it = carla->spawnedVehiclesByCarlaOSIService.erase(it);
		}
		else {
			++it;
		}
	}
}

std::tuple<bool, carla::SharedPtr<carla::client::Actor>> TrafficUpdater::spawnVehicleIfNeeded(const osi3::MovingObject& update, carla::ActorId& actorID) {
	auto spawndVehicleID = carla->spawnedVehiclesByCarlaOSIService.find(update.id().value());

	if (spawndVehicleID != carla->spawnedVehiclesByCarlaOSIService.end()) {
		return { false, carla->world->GetActor(spawndVehicleID->second.vehicle) };
	}
	else {
		std::string vehicleName = determineVehicleName(update);
		carla::geom::Transform transform = determineTransform(update);
		//Never try to merge the following two lines of code!
		//It will break at least TypeID of the new actors, like in carla->world->GetActors();
		//Blueprint library is a shared pointer and needs to be alive for carla->world->TrySpawnActor().
		//auto vehicleBlueprint = carla->world->GetBlueprintLibrary()->Find(vehicleName); does not work
		auto blueprintLibrary = carla->world->GetBlueprintLibrary();
		auto vehicleBlueprint = blueprintLibrary->Find(vehicleName);
		carla::SharedPtr<carla::client::Actor> actor = carla->world->TrySpawnActor(*vehicleBlueprint, transform);
		if (actor == nullptr) {
			std::cerr << "Could not spawn actor!" << std::endl;
		}
		spawnedActors vehicle;
		vehicle.vehicle = actor->GetId();
		carla->spawnedVehiclesByCarlaOSIService.emplace(update.id().value(), vehicle);
		return { true, actor };
	}
}

std::string TrafficUpdater::determineVehicleName(const osi3::MovingObject& update) {
	if (!runtimeParameter->replay.spawnCarByName.empty()) {
		//name set by commandline parameter
		return runtimeParameter->replay.spawnCarByName;
	}
	else if (!update.model_reference().empty()) {
		//name set by TrafficUpdate message
		return std::string(update.model_reference().c_str());
	}
	else {
		//name set by best matching size of vehicle
		return CarlaUtility::findBestMatchingCarToSpawn(
			update.base().dimension(), replayVehicleBoundingBoxes,
			runtimeParameter->replay.weightLength_X, runtimeParameter->replay.weightWidth_Y, runtimeParameter->replay.weightHeight_Z);
	}
}

carla::geom::Transform TrafficUpdater::determineTransform(const osi3::MovingObject& update) {
	auto position = Geometry::getInstance()->toCarla(update.base().position());
	position.z = runtimeParameter->replay.spawnHeight_Z;

	auto optionalPoint = carla->world->GroundProjection(position);
	if (optionalPoint) {
		carla::rpc::LabelledPoint point = *optionalPoint;
		position = point._location;
		position.z += 0.05f;//spawn needs to be a little bit higher
	}
	else {
		std::cout << "No Ground Position!" << std::endl;
	}

	auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
	std::cout << "Spawn vehicle at position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
	return carla::geom::Transform(position, orientation);
}

void TrafficUpdater::applyTrafficUpdateToActor(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor, const carla::ActorId actorId)
{
	//BASE
	if (update.base().has_position() && update.base().has_orientation()) {
		auto position = Geometry::getInstance()->toCarla(update.base().position());
		auto position2 = Geometry::getInstance()->toCarla(update.base().position());
		auto positionProjectionStart = Geometry::getInstance()->toCarla(update.base().position());
		auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());

		//height is controlled by terrain
		position.z = actor->GetLocation().z;//up works, down not
		//std::cout << "location z:" << position.z << std::endl;

		int gravity_mode = 1;
		switch (gravity_mode) {
		case 0:
			//sometimes vehicles stay airborne at slope
			//this is most likely an effect of the start of the projection
			//Problem: Projection starts inside the vehicle and the underbody
			//Encountered problems on trying to solve these problems:
			//- setting the startpoint under the vehicle results in possibility for startpoint under the street
		{
			float ground_z = 0;
			if (position.z != 0) {//condition at beginning of simulation
				auto optionalPoint = carla->world->GroundProjection(positionProjectionStart);
				if (optionalPoint) {
					carla::rpc::LabelledPoint point = *optionalPoint;
					ground_z = point._location.z;
					//std::cout << "label: 0 None, 21 Dynamic, 7 TrafficLight, 10 Terrain: " << unsigned(point._label) << std::endl;
				}
			}
			if (ground_z != position.z) {
				position.z = ground_z;
			}
			break;
		}
		case 1://linear fall per timestep
			//on flat road looks like an overloaded vehicle
			position.z = position.z - 0.01f;
			//position.z -= 0.5 * 9.81 * runtimeParameter->deltaSeconds * runtimeParameter->deltaSeconds;
			break;
		case 2://fall per timestep with g = 9.81
			GravityEntry gravityEntry;
			if (auto search = desiredHeight.find(actorId); search != desiredHeight.end()) {
				if (position.z == search->second.lastPosition) {//exact falling
					gravityEntry.fallingSteps = search->second.fallingSteps + 1;
				}
				else {
					gravityEntry.fallingSteps = 1;
				}
				float currentTime = runtimeParameter->deltaSeconds * gravityEntry.fallingSteps;
				float beforeTime = runtimeParameter->deltaSeconds * gravityEntry.fallingSteps - 1; //is 0 if not falling in last timestep
				float distance = float(0.5 * (9.81 * currentTime * currentTime) - (9.81 * beforeTime * beforeTime));
				position.z -= distance;
				gravityEntry.lastPosition = position.z;
			}
			desiredHeight[actorId] = gravityEntry;
			break;
		}

		//do not set pitch an roll of vehicles in asynchronous mode
		//these would break the visualization
		//Generally you should not set any positions in an asychronous simulation, since the physics will go crazy because of artificial high accelerations
		if (!runtimeParameter->sync) {
			orientation.pitch = actor->GetTransform().rotation.pitch;
			orientation.roll = actor->GetTransform().rotation.roll;
		}
		if (runtimeParameter->verbose) {
			std::cout << "Apply position: " << position.x << ", " << position.y << ", " << position.z << ", Yaw: " << orientation.yaw << std::endl;
		}
		actor->SetTransform(carla::geom::Transform(position, orientation));
	}

	//Velocity
	if (update.base().has_velocity()) {
		actor->SetTargetVelocity(Geometry::getInstance()->toCarlaVelocity(update.base().velocity()));
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//if (update.base().has_acceleration()) {
		//auto acceleration = carla_osi::geometry::toCarla(&update.mutable_base()->acceleration());
	//}

	//Orientation
	if (update.base().has_orientation_rate()) {
		const auto orientationRate = Geometry::getInstance()->toCarla(update.base().orientation_rate());

		//TODO Check if conversion is correct: x should be forward, y should be up, z should be right
		//actor->SetTargetAngularVelocity({ orientationRate.GetForwardVector().Length(), orientationRate.GetUpVector().Length(), orientationRate.GetRightVector().Length() });
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//if (update.mutable_base()->has_orientation_acceleration()){
	//const osi3::Orientation3d* accelerationRoll = update.mutable_base()->mutable_orientation_acceleration();
	//}

	//LIGHTSTATE
	if (update.vehicle_classification().has_light_state()) {
		auto classification = update.vehicle_classification();
		auto light_state = classification.mutable_light_state();
		auto carla_light_state = CarlaUtility::toCarla(light_state);
		//auto indicatorState = CarlaUtility::toCarla(update.vehicle_classification().light_state());
		auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);

		vehicleActor->SetLightState(carla_light_state);
	}
}

void TrafficUpdater::deleteSpawnedVehicles() {
	for (auto& vehicle : carla->spawnedVehiclesByCarlaOSIService) {
		deleteSpawnedVehiclesWithSensors(vehicle.first, vehicle.second);
	}
	carla->spawnedVehiclesByCarlaOSIService.clear();
}

void TrafficUpdater::deleteSpawnedVehiclesWithSensors(const uint32_t& id, const spawnedActors& actors) {
	std::cout << "Remove vehicle: " << id << std::endl;
	for (auto& sensorId : actors.sensors) {
		auto sensorActor = boost::dynamic_pointer_cast<carla::client::Sensor>(carla->world->GetActor(sensorId));
		if (sensorActor != nullptr) { sensorActor->Stop(); sensorActor->Destroy(); };
	}
	auto vehicleActor = carla->world->GetActor(actors.vehicle);
	if (vehicleActor != nullptr) { vehicleActor->Destroy(); };
}
