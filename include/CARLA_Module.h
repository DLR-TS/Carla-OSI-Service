/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLAGMODULE_H
#define CARLAGMODULE_H

#include "CARLA_Interface.h"
#include "Utility.h"

class CARLAModule{
private:
	bool isNumeric(const std::string& str) {
		for (char c : str) {
			if (!std::isdigit(c)) {
				return false;
			}
		}
		return true;
	}

protected:
	std::shared_ptr<CARLAInterface> carla;
    RuntimeParameter runtimeParameter;

    /*
	Checks if vehicle is spawned by Carla_OSI_Service
	return 0 if not
	*/
	OSIVehicleID vehicleIsSpawned(boost::shared_ptr<const carla::client::Vehicle> vehicle) {
        for (auto& spawnedVehicle : carla->spawnedVehiclesByCarlaOSIService) {
            if (spawnedVehicle.second.vehicle == vehicle->GetId()) {
                return spawnedVehicle.first;
            }
        }
        return 0;
    }

	/*
	Checks if roleName is a self spawned vehicle.
	return if true actorId is set accordingly
	*/
	bool isSpawnedId(const std::string& roleName, carla::ActorId& actorId) {
		if (isNumeric(roleName)) {
			auto vehicleID = carla->spawnedVehiclesByCarlaOSIService.find(std::stoi(roleName));
			if (vehicleID != carla->spawnedVehiclesByCarlaOSIService.end()) {
				actorId = vehicleID->second.vehicle;
				return true;
			}
		}
		return false;
	}

	bool isSpawnedActorId(const carla::ActorId& actorId) {
		for (auto& spawnedVehicle : carla->spawnedVehiclesByCarlaOSIService) {
			if (spawnedVehicle.second.vehicle == actorId) {
				return true;
			}
		}
		for (auto& spawnedSensor : carla->spawnedSensorsOnExternalSpawnedVehicles) {
			if (spawnedSensor == actorId) {
				return true;
			}
		}
		return false;
	}

public:
    virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla){
        this->runtimeParameter = runtimeParams;
        this->carla = carla;
    }

};
#endif //!CARLAGMODULE_H
