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
		osi3::RadarSensorView radarSensorView = std::get<osi3::RadarSensorView>(message);

		//todo
	}
	break;
	case LidarSensorViewMessage:
	{
		if (std::get_if<osi3::LidarSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LidarSensorView lidarSensorView = std::get<osi3::LidarSensorView>(message);

		//todo
	}
	break;
	case CameraSensorViewMessage:
	{
		if (std::get_if<osi3::CameraSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::CameraSensorView cameraSensorView = std::get<osi3::CameraSensorView>(message);

		//todo
	}
	break;
	case UltrasonicSensorViewMessage:
	{
		if (std::get_if<osi3::UltrasonicSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::UltrasonicSensorView ultrasonicSensorView = std::get<osi3::UltrasonicSensorView>(message);

		//todo
	}
	break;
	case SensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::SensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorViewConfiguration sensorViewConfiguration = std::get<osi3::SensorViewConfiguration>(message);

		//todo
	}
	break;
	case GenericSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::GenericSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GenericSensorViewConfiguration genericSensorViewConfiguration = std::get<osi3::GenericSensorViewConfiguration>(message);

		//todo
	}
	break;
	case RadarSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::RadarSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RadarSensorViewConfiguration radarSensorViewConfiguration = std::get<osi3::RadarSensorViewConfiguration>(message);

		//todo
	}
	break;
	case LidarSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::LidarSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LidarSensorViewConfiguration lidarSensorViewConfiguration = std::get<osi3::LidarSensorViewConfiguration>(message);

		//todo
	}
	break;
	case CameraSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::CameraSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::CameraSensorViewConfiguration cameraSensorViewConfiguration = std::get<osi3::CameraSensorViewConfiguration>(message);

		//todo
	}
	break;
	case UltrasonicSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::UltrasonicSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::UltrasonicSensorViewConfiguration ultrasonicSensorViewConfiguration = std::get<osi3::UltrasonicSensorViewConfiguration>(message);

		//todo
	}
	break;
	case GroundTruthMessage:
	{
		if (std::get_if<osi3::GroundTruth>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GroundTruth groundTruth = std::get<osi3::GroundTruth>(message);

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
		osi3::CameraSensorView cameraSensorView;

		//todo
		return (const osiMessages_t)cameraSensorView;
	}
	case UltrasonicSensorViewMessage:
	{
		osi3::UltrasonicSensorView ultrasonicSensorView;

		//todo
		return (const osiMessages_t)ultrasonicSensorView;
	}
	case SensorViewConfigurationMessage:
	{
		osi3::SensorViewConfiguration sensorViewConfiguration;

		//todo
		return (const osiMessages_t)sensorViewConfiguration;
	}
	case GenericSensorViewConfigurationMessage:
	{
		osi3::GenericSensorViewConfiguration genericSensorViewConfiguration;

		//todo
		return (const osiMessages_t)genericSensorViewConfiguration;
	}
	case RadarSensorViewConfigurationMessage:
	{
		osi3::RadarSensorViewConfiguration radarSensorViewConfiguration;

		//todo
		return (const osiMessages_t)radarSensorViewConfiguration;
	}
	case LidarSensorViewConfigurationMessage:
	{
		osi3::LidarSensorViewConfiguration lidarSensorViewConfiguration;

		//todo
		return (const osiMessages_t)lidarSensorViewConfiguration;
	}
	case CameraSensorViewConfigurationMessage:
	{
		osi3::CameraSensorViewConfiguration cameraSensorConfiguration;

		//todo
		return (const osiMessages_t)cameraSensorConfiguration;
	}
	case UltrasonicSensorViewConfigurationMessage:
	{
		osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration;

		//todo
		return (const osiMessages_t)ultrasonicSensorConfiguration;
	}
	case GroundTruthMessage:
	{
		osi3::GroundTruth groundTruth;

		//todo
		return (const osiMessages_t)groundTruth;
	}
	std::cout << "Error: Message Type not implemented in mapFromInternalState()" << std::endl;
	}
}
