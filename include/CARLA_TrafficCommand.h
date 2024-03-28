/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLATRAFFICCOMMAND_H
#define CARLATRAFFICCOMMAND_H

#include "carla/client/ActorList.h"

#include "CARLA_Module.h"

class TrafficCommander : public CARLAModule {
public:
    OSIVehicleID getHeroId();
};

#endif //!CARLATRAFFICCOMMAND_H
