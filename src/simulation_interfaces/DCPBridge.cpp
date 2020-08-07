#include "simulation_interfaces/DCPBridge.h"
#include "simulation_interfaces/iSimulationData.h"
#include <string>

int DCPBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int DCPBridge::connect(std::string) {
	return 0;
}

int DCPBridge::disconnect() {
	return 0;
}

int DCPBridge::writeToInternalState() {
	return 0;
}

int DCPBridge::doStep(double stepSize) {
	return 0;
}

int DCPBridge::readFromInternalState() {
	return 0;
}