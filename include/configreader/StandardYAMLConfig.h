#ifndef STANDARDYAMLCONGIF_H
#define STANDARDYAMLCONGIF_H
#include "yaml-cpp/yaml.h"

/**
 * \var std::string simulator
 * helper object for deciding which mapping schema an interface needs. Is used for iterating over all existing simulators in the configuration file.
 */
struct SimulatorName {
public:
	std::string simulator;
};

/**
* \var host
* host name or ip
* \var port
* port
*/
struct CARLAInterfaceConfig {
public:
	std::string host;
	uint16_t port;
};

/**
 *\paragraph Naming definitions
 * Within the CoSiMa we are using the following definitions for our host and the system we are interfacing with:

  @startuml
  node base_system
 
  node interface_system_1
  node interface_system_2

  base_system <-- interface_system_1
  base_system <-- interface_system_2
  @enduml
  
 *
 * \var std::string interface_name
 * holds name of the variable in the interface system
 * \var std::string base_name
 * holds name of the variable in the base system
 * \var std::string type
 * holds type name of the variable
 */
struct VariableDefinition {
public:
	std::string interface_name;
	std::string base_name;
	std::string type;
};

/**
* \var std::string interface_name
* holds name of the variable in the interface system
* \var std::string base_name
* holds name of the variable in the base system
* \var std::string type
* holds type name of the variable
*/

struct InterfaceYAMLConfig {
public:
	std::string simulator;
	std::string ip;
	int port;
	std::vector<VariableDefinition> inputs;
	std::vector<VariableDefinition> outputs;
};

/**
* \var std::string simulator
* name of simulator type
* \var std::string model
* path to FMU (file) //TODO should later point to Specification of System Structure and Parameterization (*.ssp file)

*/
struct FMIInterfaceConfig {
public:
	std::string simulator;
	std::string models;
};


/**
* \var std::string interface_name
* holds name of the variable in the interface system
* \var std::string base_name
* holds name of the variable in the base system
*/
struct OSIMessageConfig {
public:
	std::string interface_name;
	std::string base_name;
};

/**
* \var std::vector<OSIMessageConfig> inputs
* holds the input osi messages
* \var std::vector<OSIMessageConfig> outputs
* holds the output osi messages
*/
struct OSIInterfaceConfig {
public:
	std::string prefix;
	std::vector<OSIMessageConfig> inputs;
	std::vector<OSIMessageConfig> outputs;
};

/**
 * YAML-cpp converter for the above defined structs. Designed according to yaml-cpp tutorial: https://github.com/jbeder/yaml-cpp/wiki/Tutorial 

 * 
 */

namespace YAML {
	template<>
	struct convert<SimulatorName> {
		static Node encode(const SimulatorName& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, SimulatorName& config)
		{
			config.simulator = node["simulator"].as<std::string>();
			return true;
		}
	};

	template<>
	struct convert<InterfaceYAMLConfig> {
		static Node encode(const InterfaceYAMLConfig& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, InterfaceYAMLConfig& config)
		{
			config.simulator = node["simulator"].as<std::string>();
			config.ip = node["ip"].as<std::string>();
			config.port = node["port"].as<int>();
			config.inputs = node["input"].as<std::vector<VariableDefinition>>();
			config.outputs = node["output"].as<std::vector<VariableDefinition>>();
			return true;
		}
	};

	template<>
	struct convert<VariableDefinition> {
		static Node encode(const VariableDefinition& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, VariableDefinition& variableMap)
		{
			variableMap.interface_name = node["interface_name"].as<std::string>();
			variableMap.base_name = node["base_name"].as<std::string>();
			variableMap.type = node["type"].as<std::string>();
			return true;
		}
	};

	template<>
	struct convert<FMIInterfaceConfig> {
		static Node encode(const FMIInterfaceConfig& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, FMIInterfaceConfig& fmiInterface)
		{
			fmiInterface.simulator = node["simulator"].as<std::string>();
			fmiInterface.models = node["models"].as<std::string>();
			return true;
		}
	};

	template<>
	struct convert<OSIMessageConfig> {
		static Node encode(const OSIMessageConfig& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, OSIMessageConfig& osiMessage)
		{
			osiMessage.interface_name = node["interface_name"].as<std::string>();
			osiMessage.base_name = node["base_name"].as<std::string>();
			return true;
		}
	};

	template<>
	struct convert<OSIInterfaceConfig> {
		static Node encode(const OSIInterfaceConfig& config) {
			Node node;
			return node;
		}

		static bool decode(const Node& node, OSIInterfaceConfig& osiinterface)
		{
			osiinterface.inputs = node["input"].as<std::vector<OSIMessageConfig>>();
			osiinterface.outputs = node["output"].as<std::vector<OSIMessageConfig>>();
			return true;
		}
	};

}

#endif //!STANDARDYAMLCONGIF_H