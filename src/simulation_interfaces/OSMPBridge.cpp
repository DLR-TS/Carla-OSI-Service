#include "simulation_interfaces/OSMPBridge.h"

eOSIMessage OSMPBridge::getMessageType(std::string messageType) {
	if (messageType.find("SensorView") != std::string::npos
		&& messageType.find("Config") == std::string::npos) { return SensorViewMessage; }
	else if (messageType.find("SensorView") != std::string::npos 
		&& messageType.find("Config") != std::string::npos) { return SensorViewConfigurationMessage; }
	else if (messageType.find("SensorData") != std::string::npos) { return SensorDataMessage; }
	else if (messageType.find("GroundTruth") != std::string::npos) { return GroundTruthMessage; }
	else if (messageType.find("TrafficCommand") != std::string::npos) { return SL45TrafficCommandMessage; } //todo check if name is correct
	else if (messageType.find("InVehicleSensorData") != std::string::npos) { return SL45InVehicleSensorDataMessage; } //todo check if name is correct
	else {
		std::cout << "Error: Can not find message " << messageType << std::endl;
	}
}


int OSMPBridge::init(std::string scenario, float starttime, int mode) {
	//Instance name cannot be set with FMU4cpp. The model identifier is used automatically instead
	coSimSlave = coSimFMU->new_instance();

	coSimSlave->setup_experiment((fmi2Real)starttime);
	coSimSlave->enter_initialization_mode();
	coSimSlave->exit_initialization_mode();
	return 0;
}

int OSMPBridge::connect(std::string config) {
	return 0;
}

int OSMPBridge::disconnect() {
	return OSIBridge::disconnect();
}

int OSMPBridge::writeToInternalState() {
	addresses.clear();
	auto const model_description = coSimFMU->get_model_description();
	//iterate over unknowns declared as output
	for (auto const& unknown : model_description->model_structure->outputs) {
		// use index to translate unknown into scalar_variable. FMU ScalarVariable index begins at 1
		auto const& outputVar = (*model_description->model_variables.get())[unknown.index - 1];
		if (outputVar.is_integer()) {
			fmi2Integer integer;
			coSimSlave->read_integer(outputVar.value_reference, integer);
			saveToAddressMap(outputVar.name, integer);
		}
	}
	for (auto address : addresses) {
		OSIBridge::writeToInternalState(address.second, getMessageType(address.first));
	}
	return 0;
}
int OSMPBridge::readFromInternalState() {
	addresses.clear();
	auto const model_description = coSimFMU->get_model_description();
	//iterate over unknowns declared as output and create AddressMap
	for (auto const& inputVar : *(model_description->model_variables)) {
		if (inputVar.causality == fmi4cpp::fmi2::causality::output && inputVar.is_integer()) {
			fmi2Integer integer;
			coSimSlave->read_integer(inputVar.value_reference, integer);
			saveToAddressMap(inputVar.name, integer);
		}
	}
	//write message
	for (auto address : addresses) {
		OSIBridge::readFromInternalState(address.second, getMessageType(address.first));
	}
	//set pointers of messages
	for (auto const& inputVar : *(model_description->model_variables)) {
		if (inputVar.causality == fmi4cpp::fmi2::causality::output && inputVar.is_integer()) {
			for (auto address : addresses) {
				if (inputVar.name.find(address.first) != std::string::npos) {
					if (inputVar.name.find(".hi") != std::string::npos) {
						coSimSlave->write_integer(inputVar.value_reference, address.second.addr.base.hi);
					}else if (inputVar.name.find(".lo") != std::string::npos) {
						coSimSlave->write_integer(inputVar.value_reference, address.second.addr.base.lo);
					}else if (inputVar.name.find(".size") != std::string::npos) {
						coSimSlave->write_integer(inputVar.value_reference, address.second.size);
					}
				}
			}
		}
	}
	
	return 0;
}
int OSMPBridge::doStep(double stepSize) {
	//TODO
	//which parts of FMIBridge::doStep are needed?
	return OSIBridge::doStep(stepSize);
}

void OSMPBridge::saveToAddressMap(std::string name, int value) {
	if (0 == name.compare(name.length() - 8, 8, ".base.hi")) {
		std::string prefix = name.substr(0, name.length() - 8);
		if (addresses.find(prefix) == addresses.end()) {
			address a;
			a.addr.base.hi = value;
			addresses.insert({ prefix , a});
		}
		else {
			addresses.at(prefix).addr.base.hi = value;
		}
	}
	else if (0 == name.compare(name.length() - 8, 8, ".base.lo")) {
		std::string prefix = name.substr(0, name.length() - 8);
		if (addresses.find(prefix) == addresses.end()) {
			address a;
			a.addr.base.lo = value;
			addresses.insert({ prefix , a });
		}
		else {
			addresses.at(prefix).addr.base.lo = value;
		}
	}
	else if (0 == name.compare(name.length() - 5, 5, ".size")) {
		std::string prefix = name.substr(0, name.length() - 5);
		if (addresses.find(prefix) == addresses.end()) {
			address a;
			a.size = value;
			addresses.insert({ prefix , a });
		}
		else {
			addresses.at(prefix).size = value;
		}
	}
}
