#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t config) {

	//todo reading of lo and hi in addressInformation (currently in OSIBridge) from FMI
	
	return 0;
}


void OSIMapper::mapToInternalState(osiMessages_t message, eSupportedMessages messageType) {
	switch (messageType) {
	case SensorViewMessage:
	{
		if (std::get_if<osi3::SensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorView sensorView = std::get<osi3::SensorView>(message);

		//todo
	}
	break;
	case GenericSensorViewMessage:
	{
		if (std::get_if<osi3::GenericSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GenericSensorView genericSensorView = std::get<osi3::GenericSensorView>(message);

		//todo
	}
	break;
	case RadarSensorViewMessage:
	{
		if (std::get_if<osi3::RadarSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RadarSensorView genericSensorView = std::get<osi3::RadarSensorView>(message);

		//todo
	}
	break;
	case LidarSensorViewMessage:
	{
		if (std::get_if<osi3::LidarSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LidarSensorView genericSensorView = std::get<osi3::LidarSensorView>(message);

		//todo
	}
	break;
	case CameraSensorViewMessage:
	{
		if (std::get_if<osi3::CameraSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::CameraSensorView genericSensorView = std::get<osi3::CameraSensorView>(message);

		//todo
	}
	break;
	case UltrasonicSensorViewMessage:
	{
		if (std::get_if<osi3::UltrasonicSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::UltrasonicSensorView genericSensorView = std::get<osi3::UltrasonicSensorView>(message);

		//todo
	}
	break;
	}
}

//osiMessages_t
osiMessages_t OSIMapper::mapFromInternalState(eSupportedMessages messageType) {
	switch (messageType) {
	case SensorViewMessage:
	{
		osi3::SensorView sensorView;

		//todo
		return (const osiMessages_t)sensorView;
	}
	case GenericSensorViewMessage:
	{
		osi3::GenericSensorView genericSensorView;

		//todo
		return (const osiMessages_t)genericSensorView;
	}
	case RadarSensorViewMessage:
	{
		osi3::RadarSensorView radarSensorView;

		//todo
		return (const osiMessages_t)radarSensorView;
	}
	case LidarSensorViewMessage:
	{
		osi3::LidarSensorView lidarSensorView;

		//todo
		return (const osiMessages_t)lidarSensorView;
	}
	case CameraSensorViewMessage:
	{
		osi3::GenericSensorView cameraSensorView;

		//todo
		return (const osiMessages_t)cameraSensorView;
	}
	case UltrasonicSensorViewMessage:
	{
		osi3::GenericSensorView ultrasonicSensorView;

		//todo
		return (const osiMessages_t)ultrasonicSensorView;
	}
	std::cout << "Error: Message Type not implemented in mapFromInternalState()" << std::endl;
	}
}
