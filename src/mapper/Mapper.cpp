#include "mapper/Mapper.h"


int Mapper::searchInput(std::shared_ptr<BaseSystemInterface> baseInterface) {
	//integer
	for (const NamesAndIndex &input : config.intInputList) {
		int value = baseInterface->getIntValue(input.baseName);
		state->integers.at(input.index) = value;
	}
	//float
	for (const NamesAndIndex &input : config.floatInputList) {
		float value = baseInterface->getFloatValue(input.baseName);
		state->floats.at(input.index) = value;
	}
	//double
	for (const NamesAndIndex &input : config.doubleInputList) {
		double value = baseInterface->getDoubleValue(input.baseName);
		state->doubles.at(input.index) = value;
	}
	//bool
	for (const NamesAndIndex &input : config.boolInputList) {
		bool value = baseInterface->getBoolValue(input.baseName);
		state->bools.at(input.index) = value;
	}
	//std::string
	for (const NamesAndIndex &input : config.stringInputList) {
		std::string value = baseInterface->getStringValue(input.baseName);
		//state->strings.at(input.index) = value;
		state->strings[input.index] = value;
	}
	return 0;
}

int Mapper::writeOutput(std::shared_ptr<BaseSystemInterface> baseInterface) {
	//integer
	for (const NamesAndIndex &output : config.intOutputList) {
		baseInterface->setIntValue(output.baseName, state->integers.at(output.index));
	}
	//float
	for (const NamesAndIndex &output : config.floatOutputList) {
		baseInterface->setFloatValue(output.baseName, state->floats.at(output.index));
	}
	//double
	for (const NamesAndIndex &output : config.doubleOutputList) {
		baseInterface->setDoubleValue(output.baseName, state->doubles.at(output.index));
	}
	//bool
	for (const NamesAndIndex &output : config.boolOutputList) {
		baseInterface->setBoolValue(output.baseName, state->bools.at(output.index));
	}
	//std::string
	for (const NamesAndIndex &output : config.stringOutputList) {
		baseInterface->setStringValue(output.baseName, state->strings.at(output.index));
	}
	return 0;
}

int Mapper::readConfiguration(configVariants_t configVariants) {

	if (std::get_if<InterfaceYAMLConfig>(&configVariants) == nullptr) {
		std::cout << "Wrong Configuration! Implement specific readConfiguration method for this interface." << std::endl;
		return 1;
	}
	InterfaceYAMLConfig yamlconfig = std::get<InterfaceYAMLConfig>(configVariants);

	port = yamlconfig.port;
	ip = yamlconfig.ip;

	//fill input vectors
	for (VariableDefinition definition : yamlconfig.inputs) {

		switch (getType(definition.type)) {
		case BOOLCOSIMA:
			config.boolInputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->bools.size()));
			state->bools.push_back(bool());
			break;
		case INTEGERCOSIMA:
			config.intInputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->integers.size()));
			state->integers.push_back(int());
			break;
		case FLOATCOSIMA:
			config.floatInputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->floats.size()));
			state->floats.push_back(float());
			break;
		case DOUBLECOSIMA:
			config.doubleInputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->doubles.size()));
			state->doubles.push_back(double());
			break;
		case STRINGCOSIMA:
			config.stringInputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->strings.size()));
			state->strings.push_back(std::string());
			break;
		case DATATYPE_ERROR_COSIMA:
			std::cout << "Wrong definition of input_map. Allowed are: string, int, (integer), float, double, bool, (boolean)" << std::endl;
			return 1;
		}
	}
	//fill output vectors and internalState for temporary storage
	for (VariableDefinition definition : yamlconfig.outputs) {
		switch (getType(definition.type)) {
		case BOOLCOSIMA:
			config.boolOutputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->bools.size()));
			state->bools.push_back(bool());
			break;
		case INTEGERCOSIMA:
			config.intOutputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->integers.size()));
			state->integers.push_back(int());
			break;
		case FLOATCOSIMA:
			config.floatOutputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->floats.size()));
			state->floats.push_back(float());
			break;
		case DOUBLECOSIMA:
			config.doubleOutputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->doubles.size()));
			state->doubles.push_back(double());
			break;
		case STRINGCOSIMA:
			config.stringOutputList.push_back(NamesAndIndex(definition.base_name, definition.interface_name, (int)state->strings.size()));
			state->strings.push_back(std::string());
			break;
		case DATATYPE_ERROR_COSIMA:
			std::cout << "Wrong definition of type in output_map. Allowed are: string, int, (integer), float, double, bool, (boolean)" << std::endl;
			return 1;
		}
	}
	return 0;
}

void Mapper::mapToInternalState(values_t value, std::string interfaceName, eDataType type) {
	switch (type) {
	case BOOLCOSIMA:
		for (NamesAndIndex const &entry : config.boolOutputList)
		{
			if (entry.interfaceName == interfaceName) {
				state->bools.at(entry.index) = std::get<bool>(value);
				return;
			}
		}
		break;
	case INTEGERCOSIMA:
		for (NamesAndIndex const &entry : config.intOutputList)
		{
			if (entry.interfaceName == interfaceName) {
				state->integers.at(entry.index) = std::get<int>(value);
				return;
			}
		}
		break;
	case FLOATCOSIMA:
		for (NamesAndIndex const &entry : config.floatOutputList)
		{
			if (entry.interfaceName == interfaceName) {
				state->floats.at(entry.index) = std::get<float>(value);
				return;
			}
		}
		break;
	case DOUBLECOSIMA:
		for (NamesAndIndex const &entry : config.doubleOutputList)
		{
			if (entry.interfaceName == interfaceName) {
				state->doubles.at(entry.index) = std::get<double>(value);
				return;
			}
		}
		break;
	case STRINGCOSIMA:
		for (NamesAndIndex const &entry : config.stringOutputList)
		{
			if (entry.interfaceName == interfaceName) {
				std::string a = std::get<std::string>(value);
				state->strings.at(entry.index) = std::get<std::string>(value);
				return;
			}
		}
		break;
	}
	std::cout << "Mapper.cpp(Mapper::mapToInternalState): Could not map variable " << interfaceName << " of type " << std::boolalpha
		<< type << " to internal state because there is no variable of such name and type to map to." << std::endl;
	//Not found
	//TODO really use throw?
	throw 404;
}

values_t Mapper::mapFromInternalState(std::string interfaceName, eDataType type)
{
	switch (type) {
	case BOOLCOSIMA:
		for (NamesAndIndex const &entry : config.boolInputList)
		{
			if (entry.interfaceName == interfaceName) {
				return state->bools.at(entry.index);
			}
		}
		break;
	case INTEGERCOSIMA:
		for (NamesAndIndex const &entry : config.intInputList)
		{
			if (entry.interfaceName == interfaceName) {
				return state->integers.at(entry.index);
			}
		}
		break;
	case FLOATCOSIMA:
		for (NamesAndIndex const &entry : config.floatInputList)
		{
			if (entry.interfaceName == interfaceName) {
				return state->floats.at(entry.index);
			}
		}
		break;
	case DOUBLECOSIMA:
		for (NamesAndIndex const &entry : config.doubleInputList)
		{
			if (entry.interfaceName == interfaceName) {
				return state->doubles.at(entry.index);
			}
		}
		break;
	case STRINGCOSIMA:
		for (NamesAndIndex const &entry : config.stringInputList)
		{
			if (entry.interfaceName == interfaceName) {
				return state->strings.at(entry.index);
			}
		}
		break;
	}
	std::cout << "Mapper.cpp(Mapper::mapFromInternalState): Could not map variable " << interfaceName << " of type " << std::boolalpha
		<< type << " from internal state because there is no variable of such name and type to map from." << std::endl;
	//Not found
	//TODO really use throw?
	throw 404;
}


eDataType Mapper::getType(std::string type) {
	std::transform(type.begin(), type.end(), type.begin(),
		[](unsigned char c) { return std::tolower(c); });
	if (type.compare("string") == 0) {
		return STRINGCOSIMA;
	}
	else if (type.compare("float") == 0) {
		return FLOATCOSIMA;
	}
	else if (type.compare("double") == 0) {
		return DOUBLECOSIMA;
	}
	else if (type.compare("bool") == 0 || type.compare("boolean") == 0) {
		return BOOLCOSIMA;
	}
	else if (type.compare("int") == 0 || type.compare("integer") == 0) {
		return INTEGERCOSIMA;
	}
	else {
		return DATATYPE_ERROR_COSIMA;
	}
}

void Mapper::setOwner(std::shared_ptr<iSimulationData> owner) {
	this->owner = owner;
}

std::shared_ptr<internalState> Mapper::getInternalState()
{
	return state;
}
