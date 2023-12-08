/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLASENSORVIEW_H
#define CARLASENSORVIEW_H

#include "CARLA_GroundTruth.h"

class SensorViewer : public CARLAModule {
public:

    std::unique_ptr<GroundTruthCreator> groundTruthCreator = std::make_unique<GroundTruthCreator>();

    virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla) override {
        this->runtimeParameter = runtimeParams;
        this->carla = carla;
        groundTruthCreator->initialise(runtimeParams, carla);
    }

};


#endif //!CARLASENSORVIEW_H
