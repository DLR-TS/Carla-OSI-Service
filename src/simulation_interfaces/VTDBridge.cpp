#include <string>
#include "simulation_interfaces/VTDBridge.h"

int VTDBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int VTDBridge::connect(std::string) {
	return 0;
}

int VTDBridge::disconnect() {
	return 0;
}

int VTDBridge::writeToInternalState() {
	return 0;
}

int VTDBridge::doStep(double stepSize) {
	return 0;
}

int VTDBridge::readFromInternalState() {
	return 0;
}