#include "base_interfaces\DominionInterface.h"

int DominionInterface::readConfiguration(baseConfigVariants_t variant) {
	return 0;
}

int DominionInterface::connect() {
	return 0;
}

int DominionInterface::disconnect() {
	return 0;
}

int DominionInterface::getIntValue(std::string base_name) {
	return 0;
};

bool DominionInterface::getBoolValue(std::string base_name) {
	return true;
};

float DominionInterface::getFloatValue(std::string base_name) {
	return 0.0;
};

double DominionInterface::getDoubleValue(std::string base_name) {
	return 0.0;
};

std::string DominionInterface::getStringValue(std::string base_name) {
	return "";
};

int DominionInterface::setIntValue(std::string base_name, int value) {
	return 0;
};

int DominionInterface::setBoolValue(std::string base_name, bool value) {
	return 0;
};

int DominionInterface::setFloatValue(std::string base_name, float value) {
	return 0;
};

int DominionInterface::setDoubleValue(std::string base_name, double value) {
	return 0;
};

int DominionInterface::setStringValue(std::string base_name, std::string value) {
	return 0;
};
