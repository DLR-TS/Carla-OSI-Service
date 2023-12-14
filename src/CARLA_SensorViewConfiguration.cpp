#include "CARLA_SensorViewConfiguration.h"


//TODO Decemeber Irgendwann muss der Krams mal ausprobiert und gespawnt werden.
//Vielleicht zwei Methoden um das Ausprobieren vom abfragen zu trennen
	//spawn sensor and attach to vehicle, vehicle should have name: runtimeparameter.ego
	//add cache entry from fetchActorsFromCarla() and remove that function and its then useless subfunctions
	//save applied sensorviewconfiguration so that getSensorViewConfiguration() can retrieve the information

std::shared_ptr<const osi3::SensorViewConfiguration> SensorViewConfiger::getSensorViewConfiguration(const std::string& sensor)
{
	//string has format of: OSMPSensorViewConfigurationX
	std::string index_string(&sensor[27]);
	int index = std::stoi(index_string);
	//todo a
	return nullptr;
}
