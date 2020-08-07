#include <string>
#include "simulation_interfaces/ROSBridge.h"

int ROSBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int ROSBridge::connect(std::string) {
	return 0;
}

int ROSBridge::disconnect() {
	return 0;
}

int ROSBridge::writeToInternalState() {
	return 0;
}

int ROSBridge::doStep(double stepSize) {
	return 0;
}

int ROSBridge::readFromInternalState() {
	return 0;
}