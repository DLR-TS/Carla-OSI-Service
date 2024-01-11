/**
@authors German Aerospace Center: Björn Bahn
*/

#include <osi_sensorviewconfiguration.pb.h>

#include "ParameterDefinitions.h"

class SensorViewConfiger {
public:

    std::vector<Sensor> sensorsByUser;
    std::vector<Sensor> sensorsByFMU;

	/**
	Send applied SensorViewConfiguration for sensor.
	\param sensor OSMPSensorViewConfiguration + index
	\return The sensor's configuration as osi3::SensorViewConfiguration, or nullptr if no sensor with given name is found
	*/
	std::shared_ptr<const osi3::SensorViewConfiguration> getSensorViewConfiguration(const std::string& sensor);
};
