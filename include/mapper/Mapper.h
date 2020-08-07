#ifndef MAPPER_H
#define MAPPER_H

#include <cctype>
#include <string>
#include <map>
#include <variant>
#include <iostream>
#include "base_interfaces/BaseSystemInterface.h"
#include "simulation_interfaces/iSimulationData.h"
#include "simulation_interfaces/internalState.h"
#include "configreader/StandardYAMLConfig.h"
#include "mapper/NamesAndIndex.h"

//forward declaration
class BaseSystemInterface;

/**
* Basic value types
*/
typedef std::variant<int, float, double, bool, std::string> values_t;
/**
* YAML configuration structs
*/
typedef std::variant<InterfaceYAMLConfig, FMIInterfaceConfig> configVariants_t;

/**
* Basic data types enum
*/
enum eDataType
{
	INTEGERCOSIMA,
	FLOATCOSIMA,
	DOUBLECOSIMA,
	BOOLCOSIMA,
	STRINGCOSIMA,
	DATATYPE_ERROR_COSIMA
};

/**
* Configuration of Mapper.
* Contains the relation between base name and (interface name and index in data structure) for each datatype as well as input and output.
*/
class MapperConfig {
public:
	//base_names to interface_names relation
	std::list<NamesAndIndex> intInputList{};
	std::list<NamesAndIndex> intOutputList{};
	std::list<NamesAndIndex> floatInputList{};
	std::list<NamesAndIndex> floatOutputList{};
	std::list<NamesAndIndex> doubleInputList{};
	std::list<NamesAndIndex> doubleOutputList{};
	std::list<NamesAndIndex> boolInputList{};
	std::list<NamesAndIndex> boolOutputList{};
	std::list<NamesAndIndex> stringInputList{};
	std::list<NamesAndIndex> stringOutputList{};
};

class iSimulationData;

/**
Abstract base class of all Mappers.
*/
class Mapper {

public:
	Mapper() {
		this->config = MapperConfig();
		this->state = std::make_shared<internalState>();
	}

protected:
	/**
	Configuration of Mapper.
	*/
	MapperConfig config;
	//most interfaces need these

	/**
	IP of interface. Used by almost all mapper implementations.
	*/
	std::string ip;
	/**
	Port of interface. Used by almost all mapper implementations.
	*/
	int port;
	/**
	Holds a copy of the simulator interface variables.
	*/
	std::shared_ptr<internalState> state;
	/**
	This interface has this mapper
	*/
	std::weak_ptr<iSimulationData> owner;
public:
	/**
	Constructor of Mapper
	\param owner the owner of this mapper
	*/
	virtual void setOwner(std::shared_ptr<iSimulationData> owner) final;
	/**
	search input of this mapper from base system interface
	\param simulationInterface base system simulation interfaces
	\return success status
	*/
	int searchInput(std::shared_ptr<BaseSystemInterface> simulationInterface);
	/**
	write output variables to base interface
	*/
	int writeOutput(std::shared_ptr<BaseSystemInterface> baseInterface);
	/**
	Read configuration and fill mapper configuration.
	\param config the decoding struct
	\return success status
	*/
	virtual int readConfiguration(configVariants_t config);
	/**
	Maps the given value, name and type to the internalState
	\param value value of the variable
	\param interfaceName name of variable in interface context
	\param type data type of variable
	*/
	void mapToInternalState(values_t value, std::string interfaceName, eDataType type);
	/**
	Retrieves the value mapped to the given name and type from the internalState
	\param interfaceName name of the variable in interface context
	\param type data type of variable
	\return mapped value in internalState
	*/
	values_t mapFromInternalState(std::string interfaceName, eDataType type);
	/**
	Get state variable buffer for this interface
	*/
	std::shared_ptr<internalState> getInternalState();

protected:
	/**
	Converts data type strings to enum values.
	\param type String representation of data type.
	\return Enum representation of data type.
	*/
	eDataType getType(std::string type);

};

#endif // !MAPPER_H
