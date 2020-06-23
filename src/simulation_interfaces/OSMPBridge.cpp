#include "simulation_interfaces/OSMPBridge.h"

int OSMPBridge::readOSMP(int hi, int lo, int size, std::string messageType) {
	address address;
	address.addr.base.hi = hi;
	address.addr.base.lo = lo;
	address.size = size;
	writeToInternalState(address, getMessageType(messageType));
	return 0;
}

int OSMPBridge::writeOSMP(int& hi, int& lo, int& size, std::string messageType) {
	address address;
	readFromInternalState(address, getMessageType(messageType));
	hi = address.addr.base.hi;
	lo = address.addr.base.lo;
	size = address.size;
	return 0;
}

eOSIMessage OSMPBridge::getMessageType(std::string messageType) {
	if (messageType == "SensorView") { return SensorViewMessage; }
	else if (messageType == "SensorViewConfiguration") { return SensorViewConfigurationMessage; }
	else if (messageType == "GroundTruth") { return GroundTruthMessage; }
	else if (messageType == "TrafficCommand") { return SL45TrafficCommandMessage; } //todo check if name is correct
	else if (messageType == "InVehicleSensorData") { return SL45InVehicleSensorDataMessage; } //todo check if name is correct
	else {
		std::cout << "Error: Can not find message " << messageType << std::endl;
	}
}
