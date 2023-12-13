/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLASENSORVIEW_H
#define CARLASENSORVIEW_H

#include "boost/bimap.hpp"

#include "CARLA_GroundTruth.h"

class SensorViewer : public CARLAModule {
private:
	// contains OSI messages (values) for variable names / actor roles (keys) that can not always be retrieved, such as the sensor messages, which originate from CARLA sensor events and don't have to occur on every tick. 
	std::map<int, std::shared_ptr<osi3::SensorView>> sensorCache;
	std::shared_mutex sensorCache_mutex;

    // contains actor ids an the value of their role_name attribute. Does not contain actors without a role. Role names are used as variable name to identify OSI messages
	boost::bimap<std::string, carla::ActorId> actorRole2IDMap;
	std::shared_mutex actorRole2IDMap_mutex;

	// contains all actor ids reported by Carla during the last tick
	std::set<carla::ActorId> activeActors;

	// holds sensor position information for non-carla sensors. Maps prefixed_fmu_variable_name to mounting positions
	//TODO December
	//std::map<std::string, CoSiMa::rpc::SensorViewSensorMountingPosition, std::less<>> sensorMountingPositionMap;

	// ids for non-carla sensorViews
	std::map<std::string, uint64_t, std::less<>> sensorIds;

public:

    std::unique_ptr<GroundTruthCreator> groundTruthCreator = std::make_unique<GroundTruthCreator>();

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
	std::shared_ptr<const osi3::SensorView> getSensorView(const std::string& sensor);

	/**
	Send applied SensorViewConfiguration for sensor.
	\param sensor OSMPSensorViewConfiguration + index
	\return The sensor's configuration as osi3::SensorViewConfiguration, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorViewConfiguration> getSensorViewConfiguration(const std::string& sensor);

	/**
	Receive configuration of SensorViewConfiguration message
	\return success indicator
	*/
	int receiveSensorViewConfigurationRequest(osi3::SensorViewConfiguration& sensorViewConfiguration);

	/**
	* Fetch the actors in carla and update cache.
	* Should be called after a doStep()
	*/
	void fetchActorsFromCarla();

    //TODO December add documentation
	void sensorEventAction(carla::SharedPtr<carla::client::Sensor> source, carla::SharedPtr<carla::sensor::SensorData> sensorData, int index);

	//void copyMountingPositions(const CoSiMa::rpc::SensorViewSensorMountingPosition& from, std::shared_ptr<osi3::SensorView> to);

	void addSensorViewMountingPositions(osi3::SensorView& sensorViewMountingPositions){}
};


#endif //!CARLASENSORVIEW_H
