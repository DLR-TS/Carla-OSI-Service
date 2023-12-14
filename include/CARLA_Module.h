/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLAGMODULE_H
#define CARLAGMODULE_H

#include <memory>

#include "CARLA_Interface.h"
#include "Utility.h"
#include "ParameterDefinitions.h"

class CARLAModule{
protected:
	std::shared_ptr<CARLAInterface> carla;
    RuntimeParameter runtimeParameter;

    /*
	Checks if vehicle is spawned by Carla_OSI_Service
	return 0 if not
	*/
	OSIVehicleID vehicleIsSpawned(boost::shared_ptr<const carla::client::Vehicle> vehicle) {
        for (auto& spawnedVehicle : carla->spawnedVehiclesByCarlaOSIService) {
            if (spawnedVehicle.second == vehicle->GetId()) {
                return spawnedVehicle.first;
            }
        }
        return 0;
    }

public:
    virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla){
        this->runtimeParameter = runtimeParams;
        this->carla = carla;
    }
};
#endif //!CARLAGMODULE_H