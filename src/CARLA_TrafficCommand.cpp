#include "CARLA_TrafficCommand.h"

OSIVehicleID TrafficCommander::getHeroId() {
	auto worldActors = carla->world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();
		if (typeID.rfind("vehicle", 0) == 0) {
            auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);
			OSIVehicleID spawnedVehicleID = vehicleIsSpawned(vehicleActor);
			if (spawnedVehicleID) {
				if (runtimeParameter.ego == std::to_string(spawnedVehicleID)) {
					return spawnedVehicleID;
				}
			}
            //if not spawned by Carla OSI Service the id of Carla object is used
            for (auto& attribute : vehicleActor->GetAttributes()) {
				if ("role_name" == attribute.GetId()) {
					if (runtimeParameter.ego == attribute.GetValue()) {
						return carla_osi::id_mapping::getOSIActorId(vehicleActor).release()->value();
					}
				}
			}
        }
    }
    std::cout << "No ego vehicle for TrafficCommand message found." << std::endl;
    return 0;
}
