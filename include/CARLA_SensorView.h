/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLASENSORVIEW_H
#define CARLASENSORVIEW_H

#include <mutex>

#include "boost/bimap.hpp"

#include "CARLA_GroundTruth.h"

class SensorViewer : public CARLAModule {
private:
	// contains OSI messages (values) for variable names / actor roles (keys) that can not always be retrieved, such as the sensor messages, which originate from CARLA sensor events and don't have to occur on every tick. 
	std::mutex sensorCache_mutex;

    // contains actor ids an the value of their role_name attribute. Does not contain actors without a role. Role names are used as variable name to identify OSI messages
	boost::bimap<std::string, carla::ActorId> actorRole2IDMap;
	std::mutex actorRole2IDMap_mutex;

	// contains all actor ids reported by Carla during the last tick
	std::set<carla::ActorId> activeActors;

	// ids for non-carla sensorViews
	std::map<std::string, uint64_t, std::less<>> sensorIds;

public:

    std::unique_ptr<GroundTruthCreator> groundTruthCreator = std::make_unique<GroundTruthCreator>();
	std::map<std::string, std::shared_ptr<osi3::SensorView>> sensorCache;

    virtual void initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla) override {
        this->runtimeParameter = runtimeParams;
        this->carla = carla;
        groundTruthCreator->initialise(runtimeParams, carla);
    }

	// generate a SensorView that holds only ground truth. Can be used as input for osi3::SensorView generating OSI sensors;
	 std::shared_ptr<osi3::SensorView> getSensorViewGroundTruth(const std::string& name);

	/**
	Retrieve CARLA Sensor output from the sensor with the given index. Messages are cached and updated during a sensor's tick.
	\param sensor OSMPSensorView + index
	\return The sensor's latest output as osi3::SensorView, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorView> getSensorView(const std::string& sensorName);

	/**
	* Fetch the actors in carla and update cache.
	* Should be called after a doStep()
	*/
	void fetchActorsFromCarla();

	/**
	* Callback function for all Carla sensors
	*/
	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData, std::string index);
};

#endif //!CARLASENSORVIEW_H
