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

public:
    virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla){
        this->runtimeParameter = runtimeParams;
        this->carla = carla;
    }

};
#endif //!CARLAGMODULE_H