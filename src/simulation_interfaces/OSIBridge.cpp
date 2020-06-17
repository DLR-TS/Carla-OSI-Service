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

		bool parseSuccess = false;
		switch (info.first) {
		case SensorViewMessage:
			parseSuccess = sensorView.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapOSIToInternalState(sensorView, SensorViewMessage);
			break;
		case SensorViewConfigurationMessage:
			parseSuccess = sensorViewConfiguration.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapOSIToInternalState(sensorView, SensorViewConfigurationMessage);
			break;
		case GroundTruthMessage:
			parseSuccess = groundTruth.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapOSIToInternalState(sensorView, SensorViewConfigurationMessage);
			break;
		case SL45TrafficCommandMessage:
			//parseSuccess = trafficCommand.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapOSIToInternalState(trafficCommand, SL45TrafficCommandMessage);
			break;
		case SL45InVehicleSensorDataMessage:
			//parseSuccess = inVehicleSensorData.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}
			//mapper->mapOSIToInternalState(inVehicleSensorData, SL45InVehicleSensorDataMessage);
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
		switch (info.first) {
		case SensorViewMessage:
			//sensorView = mapper->mapFromInternalState
			sensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case SensorViewConfigurationMessage:
			//sensorViewConfiguration = mapper->mapFromInternalState
			sensorViewConfiguration.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case GroundTruthMessage:
			//groundTruth = mapper->mapFromInternalState
			groundTruth.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case SL45TrafficCommandMessage:
			//trafficCommand = mapper->mapFromInternalState
			//trafficCommand.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case SL45InVehicleSensorDataMessage:
			//inVehicleSensorData = mapper->mapFromInternalState
			//inVehicleSensorData.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		}
	}

	return 0;
}
