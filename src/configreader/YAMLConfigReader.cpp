#include <string>
#include "configreader/YAMLConfigReader.h"

YAMLConfigReader::YAMLConfigReader(std::string path) {
	std::cout << "Load YAML file: " << path << std::endl;
	simulators = YAML::LoadFile(path);
}

const std::vector<SingleYAMLConfig> YAMLConfigReader::getSimulatorNames() {
	std::vector<SingleYAMLConfig> names;
	//counter for later tracking the amount of same interfaces with different configurations
	std::map<eSimulatorName, int> counter;
	for (int i = FMI; i != SIMULATORNAME_ERROR; i++)
	{
		eSimulatorName name = static_cast<eSimulatorName>(i);
		counter.emplace(name, 0);
	}

	for (std::size_t i = 0; i < simulators.size(); i++) {
		SimulatorName conf = simulators[i].as<SimulatorName>();
		const eSimulatorName simName = nameToEnum(conf.simulator);
		if (simName == SIMULATORNAME_ERROR) {
			std::cout << "Not supported simulator name in yaml configration file: " << conf.simulator << std::endl;
			exit(1);
		}
		std::map<eSimulatorName, int>::iterator countIter = counter.find(simName);
		names.push_back(SingleYAMLConfig(simName, countIter->second));
		countIter->second++;
	}
	return names;
}

int YAMLConfigReader::setConfig(std::shared_ptr<iSimulationData> simulator, SingleYAMLConfig simulatorname) {
	int index = 0;
	for (std::size_t i = 0; i < simulators.size(); i++) {
		SimulatorName name = simulators[i].as<SimulatorName>();
		if (nameToEnum(name.simulator) == simulatorname.simulator) {
			if (index == simulatorname.index) {
				switch (simulatorname.simulator) {
				case VTD:
				case SUMO:
				case UNREAL:
				case ROS:
					return simulator->getMapper()->readConfiguration(simulators[i].as<InterfaceYAMLConfig>());
				case FMI:
					return simulator->getMapper()->readConfiguration(simulators[i].as<FMIInterfaceConfig>());
					//case OSI:
						//TODO
				}
			}
			else {
				index++;
			}
		}
	}
	std::cout << "Error no node found with name: " << simulatorname.simulator << std::endl;
	return 1;
}

const eSimulatorName YAMLConfigReader::nameToEnum(std::string simulatorName) {
	std::transform(simulatorName.begin(), simulatorName.end(), simulatorName.begin(),
		[](unsigned char c) { return std::tolower(c); });
	if (simulatorName == "vtd") {
		return VTD;
	}
	else if (simulatorName == "fmi") {
		return FMI;
	}
	else if (simulatorName == "sumo") {
		return SUMO;
	}
	else if (simulatorName == "osi") {
		return OSI;
	}
	else if (simulatorName == "ue" || simulatorName == "unrealengine" || simulatorName == "unreal") {
		return UNREAL;
	}
	else if (simulatorName == "ros") {
		return ROS;
	}
	else {
		return SIMULATORNAME_ERROR;
	}
}
