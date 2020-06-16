#include <string>
#include "simulation_interfaces/OSIBridge.h"

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
		//parse osi message from string/array to object

		bool parseSuccess = true;
		switch (info.first) {
		case SensorViewMessage:
			parseSuccess = sensorView.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapToInternalState(sensorView, SensorViewMessage);
			break;
		case SensorViewConfigurationMessage:
			parseSuccess = sensorViewConfiguration.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapToInternalState(sensorView, SensorViewConfigurationMessage);
			break;
		case GroundTruthMessage:
			parseSuccess = groundTruth.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapToInternalState(sensorView, SensorViewConfigurationMessage);
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
			//genericSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case RadarSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//radarSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case LidarSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//lidarSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case CameraSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//cameraSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case UltrasonicSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//ultrasonicSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		}
	}

	return 0;
}
