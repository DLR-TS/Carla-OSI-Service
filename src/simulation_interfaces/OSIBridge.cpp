#include <string>
#include "simulation_interfaces/OSIBridge.h"

//to do reading of lo and hi in addressInformation from fmi

int OSIBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int OSIBridge::connect(std::string) {
	return 0;
}

int OSIBridge::disconnect() {
	return 0;
}

int OSIBridge::writeToInternalState() {

	for (auto info : addressInformation) {
		//parse osi message from string to object

		bool parseSuccess = true;
		switch (info.first) {
		case SensorViewMessage:
			parseSuccess = sensorView.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapToInternalState
			break;
		case GenericSensorViewMessage:
			parseSuccess = genericSensorView.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapToInternalState
			break;
		}
	}

	return 0;
}

int OSIBridge::doStep(double stepSize) {
	return 0;
}

int OSIBridge::readFromInternalState() {
	for (auto info : addressInformation) {
		//parse osi message from string to object

		switch (info.first) {
		case SensorViewMessage:
			//sensorView = mapper->mapFromInternalState
			sensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case GenericSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			genericSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		}
	}

	return 0;
}
