#include <string>
#include "simulation_interfaces/UnrealBridge.h"

int UnrealBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int UnrealBridge::connect(std::string) {
	return 0;
}

int UnrealBridge::disconnect() {
	return 0;
}

int UnrealBridge::writeToInternalState() {
	return 0;
}

int UnrealBridge::doStep(double stepSize) {
	return 0;
}

int UnrealBridge::readFromInternalState() {
	return 0;
}