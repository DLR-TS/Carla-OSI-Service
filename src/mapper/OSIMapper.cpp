#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t config) {
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
	}
}

//osiMessages_t
int OSIMapper::mapFromInternalState(eSupportedMessages messageType) {
	switch (messageType) {
	case SensorViewMessage:
	{
		osi3::SensorView sensorView;

		//todo
		osiMessages_t message = sensorView;
		//return message;
		return 0;
	}
	//case GenericSensorViewMessage:
	{
		//osi3::GenericSensorView genericSensorView;

		//todo
		//return message;
	}
	}
}
