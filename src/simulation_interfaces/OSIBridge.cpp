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

	return 0;
}

int OSIBridge::writeToInternalState(address address, eOSIMessage messageType)
{
	bool parseSuccess = false;
	switch (messageType) {
	case SensorViewMessage:
		parseSuccess = sensorView.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//(OSIMapper)mapper->mapOSIToInternalState(sensorView, SensorViewMessage);
		break;
	case SensorViewConfigurationMessage:
		parseSuccess = sensorViewConfiguration.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//(OSIMapper)mapper->mapOSIToInternalState(sensorView, SensorViewConfigurationMessage);
		break;
	case GroundTruthMessage:
		parseSuccess = groundTruth.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//(OSIMapper)mapper->mapOSIToInternalState(sensorView, SensorViewConfigurationMessage);
		break;
	case SL45TrafficCommandMessage:
		//parseSuccess = trafficCommand.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//(OSIMapper)mapper->mapOSIToInternalState(trafficCommand, SL45TrafficCommandMessage);
		break;
	case SL45InVehicleSensorDataMessage:
		//parseSuccess = inVehicleSensorData.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//(OSIMapper)mapper->mapOSIToInternalState(inVehicleSensorData, SL45InVehicleSensorDataMessage);
		break;
	}
	return 0;
}


int OSIBridge::doStep(double stepSize) {
	return 0;
}

int OSIBridge::readFromInternalState()
{
	return 0;
}

int OSIBridge::readFromInternalState(address& address, eOSIMessage messageType) {
	switch (messageType) {
	case SensorViewMessage:
		//sensorView = (OSIMapper)mapper->mapFromInternalState(SensorViewMessage);
		sensorView.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SensorViewConfigurationMessage:
		//sensorViewConfiguration = (OSIMapper)mapper->mapFromInternalState(SensorViewConfigurationMessage);
		sensorViewConfiguration.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case GroundTruthMessage:
		//groundTruth = (OSIMapper)mapper->mapFromInternalState(GroundTruthMessage);
		groundTruth.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SL45TrafficCommandMessage:
		//trafficCommand = (OSIMapper)mapper->mapFromInternalState(SL45TrafficCommandMessage);
		//trafficCommand.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SL45InVehicleSensorDataMessage:
		//inVehicleSensorData = (OSIMapper)mapper->mapFromInternalState(SL45InVehicleSensorDataMessage);
		//inVehicleSensorData.SerializeToArray((void*)address.addr.address, address.size);
		break;
	}

	return 0;
}
