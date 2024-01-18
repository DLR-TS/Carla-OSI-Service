/**
@authors German Aerospace Center: Björn Bahn
*/

#include <osi_sensorviewconfiguration.pb.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorList.h>

#include "boost/bimap.hpp"

#include "CARLA_SensorView.h"

class SensorViewConfiger : public CARLAModule {
public:

	/**
	Theses sensors are defined by the sensorview configuration of a FMU.
	They need an answer in form of a osi3::sensorviewconfiguration before the simulation starts.
	It is possible, that the sensors can not be spawned instantly, since the ego vehicle itself might be spawned later.
	*/
    std::vector<Sensor> sensorsByFMU;
	/**
	Theses sensors are defined by the configuration file in CoSiMa.
	They do not need an answer in form of a osi3::sensorviewconfiguration.
	It is possible, that the sensors can not be spawned instantly, since the ego vehicle itself might be spawned later.
	*/
	std::vector<Sensor> sensorsByUser;
	/**
	If any sensors are defined and not yet spawned, try to spawn them.
	*/
	void trySpawnSensors(std::shared_ptr<SensorViewer> sensorViewer);

	/**
	Send applied SensorViewConfiguration for sensor.
	\return The sensor's configuration as osi3::SensorViewConfiguration, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<osi3::SensorViewConfiguration> getLastSensorViewConfiguration();


private:

	bool trySpawnSensor(std::shared_ptr<SensorViewer> sensorViewer, const Sensor& sensor);

	carla::ActorId getActorIdFromName(std::string roleName);

	std::string matchSensorType(SENSORTYPES type);
};
