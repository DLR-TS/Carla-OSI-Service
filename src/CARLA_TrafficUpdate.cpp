#include "CARLA_TrafficUpdate.h"

void TrafficUpdater::initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla) {
	CARLAModule::initialise(runtimeParams, carla);
	if (runtimeParameter.replay.enabled) {
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
			transform = carla::geom::Transform(location_alternative, carla::geom::Rotation(0,0,0));
			temp_actor = carla->world->TrySpawnActor(vehicle, transform);
			if (temp_actor == nullptr) {
				std::cerr << "Can not spawn vehicle. Tried two locations with x,y,z: 0,0,0 and 0,0,10000" << std::endl;
			}
		}
		auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(temp_actor);
		auto bbox = vehicleActor->GetBoundingBox();
		replayVehicleBoundingBoxes.emplace_back(vehicle.GetId(), bbox.extent);
		if (runtimeParameter.verbose) {
			std::cout << "bbox: " << vehicle.GetId() << " Length:" << bbox.extent.x*2 << " Width:" << bbox.extent.y*2 << " Height:" << bbox.extent.z*2 << std::endl;
		}
		carla->world->Tick(carla->client->GetTimeout());
		temp_actor->Destroy();
	}
	if (runtimeParameter.verbose) {
		std::cout << "Number of possible vehicle bounding boxes: " << replayVehicleBoundingBoxes.size() << std::endl;
	}
}

int TrafficUpdater::receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate) {
	if (trafficUpdate.update_size() == 0) {
		std::cerr << __FUNCTION__ << " No update." << std::endl;
		return 1;
	}

	carla::ActorId actorId;
	std::vector<int> listOfUpdatedVehicles;

	for (auto& update : trafficUpdate.update()) {
		//assumption: actor exists in Carla
		actorId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
		auto actor = carla->world->GetActor(actorId);
		if (runtimeParameter.replay.enabled) {
			actor = spawnVehicleIfNeeded(update, actorId);
		}
		if (actor == nullptr) {
			std::cout << "Actor not found! No position updates will be done!" << std::endl;
			return 0;
		}
		listOfUpdatedVehicles.push_back(update.id().value());
		applyTrafficUpdateToActor(update, actor);
	}
	removeSpawnedVehiclesIfNotUpdated(listOfUpdatedVehicles);
	return 0;
}

void TrafficUpdater::removeSpawnedVehiclesIfNotUpdated(std::vector<int>& listOfUpdatedVehicles){
    auto it = carla->spawnedVehiclesByCarlaOSIService.begin();
    while (it != carla->spawnedVehiclesByCarlaOSIService.end()) {

        if (std::find(listOfUpdatedVehicles.begin(), listOfUpdatedVehicles.end(), it->first) == listOfUpdatedVehicles.end()) {
			std::cout << "No update for vehicle: " << it->first << " Will stop the display of this vehicle." << std::endl;
			carla->world->GetActor(it->second)->Destroy();
            it = carla->spawnedVehiclesByCarlaOSIService.erase(it);
        } else {
            ++it;
        }
    }
}

carla::SharedPtr<carla::client::Actor> TrafficUpdater::spawnVehicleIfNeeded(const osi3::MovingObject& update, carla::ActorId& actorID) {
	auto spawndVehicleID = carla->spawnedVehiclesByCarlaOSIService.find(update.id().value());
	carla::SharedPtr<carla::client::Actor> actor;

	if (spawndVehicleID != carla->spawnedVehiclesByCarlaOSIService.end()) {
		actor = carla->world->GetActor(spawndVehicleID->second);
	} else {
		std::string vehicleName = determineVehicleName(update);
		carla::geom::Transform transform = determineTransform(update);
		auto vehicleBlueprint = carla->world->GetBlueprintLibrary()->Find(vehicleName);
		actor = carla->world->TrySpawnActor(*vehicleBlueprint, transform);
		if (actor == nullptr) {
			std::cerr << "Could not spawn actor!" << std::endl;
		}

		carla->spawnedVehiclesByCarlaOSIService.emplace(update.id().value(), actor->GetId());
	}

	return actor;
}

std::string TrafficUpdater::determineVehicleName(const osi3::MovingObject& update){
	if (!runtimeParameter.replay.spawnCarByName.empty()){
		//name set by commandline parameter
		return runtimeParameter.replay.spawnCarByName;
	} else if (!update.model_reference().empty()) {
		//name set by TrafficUpdate message
		return std::string(update.model_reference().c_str());
	} else {
		//name set by best matching size of vehicle
		return CarlaUtility::findBestMatchingCarToSpawn(
			update.base().dimension(), replayVehicleBoundingBoxes,
			runtimeParameter.replay.weightLength_X, runtimeParameter.replay.weightWidth_Y, runtimeParameter.replay.weightHeight_Z);
	}
}

carla::geom::Transform TrafficUpdater::determineTransform(const osi3::MovingObject& update){
	auto position = Geometry::getInstance()->toCarla(update.base().position());
	position.z = runtimeParameter.replay.spawnHeight_Z;

	auto optionalPoint = carla->world->GroundProjection(position);
	if (optionalPoint) {
		carla::rpc::LabelledPoint point = *optionalPoint;
		position = point._location;
		position.z += 0.05;
	} else {
		std::cout << "No Ground Position!" << std::endl;
	}

	auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
	std::cout << "Spawn vehicle at position: " << position.x << ", " << position.y  << ", " << position.z << std::endl;
	return carla::geom::Transform(position, orientation);
}

void TrafficUpdater::applyTrafficUpdateToActor(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor)
{
	//BASE
	if (update.base().has_position() && update.base().has_orientation()) {
		auto position = Geometry::getInstance()->toCarla(update.base().position());
		auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
		if (runtimeParameter.replay.enabled){
			position.z = runtimeParameter.replay.spawnHeight_Z;
		}

		//height is controlled by terrain
		if (actor->GetLocation().z != 0) {
			//position.z = actor->GetLocation().z;
		}
		else {
			//new height is spawnHeight
		}

		//do not set pitch an roll of vehicles in asynchronous mode
		//these would break the visualization
		//Generally you should not set any positions in an asychronous simulation, since the physics will go crazy because of artificial high accelerations
		if (!runtimeParameter.sync) {
			orientation.pitch = actor->GetTransform().rotation.pitch;
			orientation.roll = actor->GetTransform().rotation.roll;
		}
		if (runtimeParameter.verbose) {
			std::cout << "Apply position: " << position.x << ", " << position.y  << ", " << position.z << ", Yaw: " << orientation.yaw << std::endl;
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
		std::cout << "Remove vehicle: " << vehicle.first << std::endl;
		carla->world->GetActor(vehicle.second)->Destroy();
	}
}
