#include "simulation_interfaces/OSIBridge.h"

int OSIBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int OSIBridge::connect(std::string) {
	//write osi configuration in writeAddressInformation and readAddressInformation
	return 0;
}

int OSIBridge::disconnect() {
	return 0;
}

int OSIBridge::writeToInternalState() {
	for (auto address : writeAddressInformation) {
		writeToInternalState(address.second, address.first);
	}
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
		std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(sensorView.SerializeAsString(), SensorViewMessage);
		break;
	case SensorViewConfigurationMessage:
		parseSuccess = sensorViewConfiguration.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(sensorViewConfiguration.SerializeAsString(), SensorViewConfigurationMessage);
		break;
	case SensorDataMessage:
		parseSuccess = sensorData.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(sensorData.SerializeAsString(), SensorDataMessage);
		break;
	case GroundTruthMessage:
		parseSuccess = groundTruth.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(groundTruth.SerializeAsString(), SensorViewConfigurationMessage);
		break;
	case SL45TrafficCommandMessage:
		//parseSuccess = trafficCommand.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(trafficCommand.SerializeAsString(), SL45TrafficCommandMessage);
		break;
	case SL45InVehicleSensorDataMessage:
		//parseSuccess = inVehicleSensorData.ParseFromArray((const void*)address.addr.address, address.size);
		if (parseSuccess == false) {
			return 1;
		}
		//std::static_pointer_cast<OSIMapper>(mapper)->mapOSIToInternalState(inVehicleSensorData.SerializeAsString(), SL45InVehicleSensorDataMessage);
		break;
	}
	return 0;
}


int OSIBridge::doStep(double stepSize) {
	return 0;
}

int OSIBridge::readFromInternalState(){
	for (auto address : readAddressInformation) {
		readFromInternalState(address.second, address.first);
	}
	return 0;
}

int OSIBridge::readFromInternalState(address& address, eOSIMessage messageType) {
	switch (messageType) {
	case SensorViewMessage:
		sensorView.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(SensorViewMessage));
		sensorView.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SensorViewConfigurationMessage:
		sensorViewConfiguration.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(SensorViewConfigurationMessage));
		sensorViewConfiguration.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SensorDataMessage:
		sensorData.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(SensorDataMessage));
		sensorData.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case GroundTruthMessage:
		groundTruth.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(GroundTruthMessage));
		groundTruth.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SL45TrafficCommandMessage:
		//trafficCommand.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(TrafficCommandMessage));
		//trafficCommand.SerializeToArray((void*)address.addr.address, address.size);
		break;
	case SL45InVehicleSensorDataMessage:
		//inVehicleSensorData.ParseFromString(std::static_pointer_cast<OSIMapper>(mapper)->mapOSIFromInternalState(InVehicleSensorDataMessage));
		//inVehicleSensorData.SerializeToArray((void*)address.addr.address, address.size);
		break;
	}

	return 0;
}
