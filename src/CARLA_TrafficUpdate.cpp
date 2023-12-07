#include "CARLA_TrafficUpdate.h"

int TrafficUpdater::initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla) {
	this->runtimeParameter = runtimeParams;
	this->carla = carla;
	return 0;
}

void TrafficUpdater::fillBoundingBoxLookupTable() {
	auto vehicleLibrary = carla->world->GetBlueprintLibrary()->Filter("vehicle.*");
	carla::geom::Location location(0, 0, 0);
	carla::geom::Rotation rotation(0, 0, 0);
	carla::geom::Transform transform(location, rotation);
	for (auto vehicle : *vehicleLibrary) {
		auto temp_actor = carla->world->TrySpawnActor(vehicle, transform);
		if (temp_actor == nullptr) {//map has object near 0 0 z
			carla::geom::Location location_alternative(0, 0, 10000);
			transform = carla::geom::Transform(location_alternative, rotation);
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
	//OSI documentation:
	//Only the id, base member (without dimension and base_polygon),
	//and the vehicle_classification.light_state members are considered in
	//updates, all other members can be left undefined, and will be
	//ignored by the receiver of this message.

	if (trafficUpdate.update_size() == 0) {
		std::cerr << __FUNCTION__ << " No update." << std::endl;
		return 3;
	}

	carla::ActorId actorId;

	if (runtimeParameter.replay.enabled) {
		replayTrafficUpdate(trafficUpdate, actorId);
		return 0;
	}

	for (auto& update : trafficUpdate.update()) {
		//original operation mode with update for exisiting cars

		actorId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
		auto actor = carla->world->GetActor(actorId);
		if (actor == nullptr && !runtimeParameter.replay.enabled) {
			std::cout << "Actor not found! No position updates will be done!" << std::endl;
			return 0;
		}
		applyTrafficUpdate(update, actor);
	}
	return 0;
}

void TrafficUpdater::replayTrafficUpdate(const osi3::TrafficUpdate& trafficUpdate, carla::ActorId& actorID) {
	//check if spawned from Carla-OSI-Service

	for (auto& update : trafficUpdate.update()) {
		auto ActorID = carla->spawnedVehicles.find(update.id().value());
		if (ActorID == carla->spawnedVehicles.end()) {
			//not found --> spawn car
			std::string vehicleName;
			if (!runtimeParameter.replay.spawnCarByName.empty()){
				//name set by commandline parameter
				vehicleName = runtimeParameter.replay.spawnCarByName;
			} else if (!update.model_reference().empty()) {
				//name set by TrafficUpdate message
				vehicleName = std::string(update.model_reference().c_str());
			} else {
				//name set by best matching size of vehicle
				vehicleName = CarlaUtility::findBestMatchingCarToSpawn(update.base().dimension(), replayVehicleBoundingBoxes,
				runtimeParameter.replay.weightLength_X, runtimeParameter.replay.weightWidth_Y, runtimeParameter.replay.weightHeight_Z);
			}
			//calculate transform of vehicle
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

			std::cout << "Spawn vehicle: " << vehicleName << " Position: " << position.x << ", " << position.y  << ", " << position.z << std::endl;

			///END TEST
			auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
			carla::geom::Transform transform(position, orientation);

			auto blueprintlibrary = carla->world->GetBlueprintLibrary();
			auto vehicleBlueprint = blueprintlibrary->Find(vehicleName);
			carla::SharedPtr<carla::client::Actor> actor;
			actor = carla->world->TrySpawnActor(*vehicleBlueprint, transform);
			if (actor == nullptr) {
				std::cerr << "Could not spawn actor!" << std::endl;
				continue;
			}

			spawnedVehicle addedVehicle;
			addedVehicle.idInCarla = actor->GetId();
			carla->spawnedVehicles.emplace(update.id().value(), addedVehicle);
			applyTrafficUpdate(update, actor);
			} else {
				applyTrafficUpdate(update, carla->world->GetActor(ActorID->second.idInCarla));
			}
			//save the Update with current timestamp
			carla->spawnedVehicles[update.id().value()].lastTimeUpdated = trafficUpdate.timestamp().seconds() * 1000000000u + trafficUpdate.timestamp().nanos();
		}

		//deconstruct actors with no update
		for (auto& vehicle : carla->spawnedVehicles) {
			if (vehicle.second.lastTimeUpdated != trafficUpdate.timestamp().seconds() * 1000000000u + trafficUpdate.timestamp().nanos()) {
				std::cout << "No update for vehicle: " << vehicle.first << " Will stop the display of this vehicle." << std::endl;
				auto vehicleActor = carla->world->GetActor(vehicle.second.idInCarla);
				carla->deletedVehicles.emplace(vehicle);
				vehicleActor->Destroy();
			}
		}
	for (auto& deletedVehicleId : carla->deletedVehicles) {
		carla->spawnedVehicles.erase(deletedVehicleId.first);
	}
}

void TrafficUpdater::applyTrafficUpdate(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor)
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
	//deconstruct all spawned actors
	for (auto& vehicle : carla->spawnedVehicles) {
		std::cout << "Remove: " << vehicle.first << std::endl;
		auto vehicleActor = carla->world->GetActor(vehicle.second.idInCarla);
		carla->deletedVehicles.emplace(vehicle);
		vehicleActor->Destroy();
	}
}
