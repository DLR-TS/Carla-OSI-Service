#include "catch2/catch.hpp"

#include "mapper/Mapper.h"
#include "MockBaseSimulator.cpp"
#include "MockInterfaceSimulator.cpp"
#include "MockMapper.cpp"
#include "configreader/StandardYAMLConfig.h"

TEST_CASE("Request variable from base system") {

	MockBaseSimulator* basesimulator = new MockBaseSimulator();
	Mapper* mapper = (Mapper*)new MockMapper();
	std::shared_ptr<BaseSystemInterface> base_simulator_ptr = std::shared_ptr<BaseSystemInterface>((BaseSystemInterface*)basesimulator);
	std::shared_ptr<iSimulationData> interface_simulator = std::shared_ptr<iSimulationData>(new MockInterfaceSimulator(std::shared_ptr<Mapper>(mapper)));

	InterfaceYAMLConfig config;
	config.ip = "12.34.56.78";
	config.port = 1000;
	std::vector<VariableDefinition> inputs;
	std::vector<VariableDefinition> outputs;

	VariableDefinition definition;
	definition.base_name = "VERYBASENAME";
	definition.interface_name = "MUCHINTERFACE";
	definition.type = "string";
	inputs.push_back(definition);

	config.inputs = inputs;
	config.outputs = outputs;

	mapper->setOwner(interface_simulator);

	//test begin
	mapper->readConfiguration(config);
	mapper->searchInput(base_simulator_ptr);

	REQUIRE(basesimulator->requestedVariables.size() == 1);
	REQUIRE(basesimulator->requestedVariables.at(0) == "VERYBASENAME");
}

TEST_CASE("Mapping the outputs of interfaces to the internalstate") {
	MockBaseSimulator* basesimulator = new MockBaseSimulator();
	MockMapper* n = new MockMapper();
	Mapper* mapper = (Mapper*)n;
	std::shared_ptr<iSimulationData> interface_simulator = std::shared_ptr<iSimulationData>(new MockInterfaceSimulator(std::shared_ptr<Mapper>(mapper)));
	std::shared_ptr<BaseSystemInterface> base_simulator_ptr = std::shared_ptr<BaseSystemInterface>((BaseSystemInterface*)basesimulator);

	InterfaceYAMLConfig config;
	config.ip = "12.34.56.78";
	config.port = 1000;
	std::vector<VariableDefinition> inputs;
	std::vector<VariableDefinition> outputs;

	VariableDefinition definition;
	definition.base_name = "VERYBASENAME";
	definition.interface_name = "MUCHINTERFACE";
	definition.type = "string";
	outputs.push_back(definition);

	VariableDefinition definition2;
	definition2.base_name = "VERYBASENAME2";
	definition2.interface_name = "MUCHINTERFACE2";
	definition2.type = "int";
	outputs.push_back(definition2);

	config.inputs = inputs;
	config.outputs = outputs;

	interface_simulator->getMapper()->setOwner(interface_simulator);
	interface_simulator->getMapper()->readConfiguration(config);

	std::string value1 = "CRAZYVALUE";
	int value2 = 1289;

	//test begin
	interface_simulator->getMapper()->mapToInternalState(value1, "MUCHINTERFACE", eDataType::STRINGCOSIMA);
	interface_simulator->getMapper()->mapToInternalState(value2, "MUCHINTERFACE2", eDataType::INTEGERCOSIMA);

	REQUIRE(interface_simulator->getMapper()->getInternalState()->strings.at(0) == "CRAZYVALUE");
	REQUIRE(interface_simulator->getMapper()->getInternalState()->integers.at(0) == 1289);
}

TEST_CASE() {
	MockBaseSimulator* basesimulator = new MockBaseSimulator();
	MockMapper* n = new MockMapper();
	Mapper* mapper = (Mapper*)n;
	std::shared_ptr<iSimulationData> interface_simulator = std::shared_ptr<iSimulationData>(new MockInterfaceSimulator(std::shared_ptr<Mapper>(mapper)));
	std::shared_ptr<BaseSystemInterface> base_simulator_ptr = std::shared_ptr<BaseSystemInterface>((BaseSystemInterface*)basesimulator);

	InterfaceYAMLConfig config;
	config.ip = "12.34.56.78";
	config.port = 1000;
	std::vector<VariableDefinition> inputs;
	std::vector<VariableDefinition> outputs;

	VariableDefinition definition;
	definition.base_name = "VERYBASENAME";
	definition.interface_name = "MUCHINTERFACE";
	definition.type = "string";
	outputs.push_back(definition);

	config.inputs = inputs;
	config.outputs = outputs;

	interface_simulator->getMapper()->setOwner(interface_simulator);
	interface_simulator->getMapper()->readConfiguration(config);

	std::string value = "testvalue";
	mapper->mapToInternalState(value, "MUCHINTERFACE", STRINGCOSIMA);

	//test begin
	mapper->writeOutput(base_simulator_ptr);

	REQUIRE(basesimulator->stringvalue == "testvalue");
};

TEST_CASE("Read a YAML-config into internal state and back") {
	MockBaseSimulator* basesimulator = new MockBaseSimulator();
	MockMapper* n = new MockMapper();
	Mapper* mapper = (Mapper*)n;
	std::shared_ptr<iSimulationData> interface_simulator = std::shared_ptr<iSimulationData>(new MockInterfaceSimulator(std::shared_ptr<Mapper>(mapper)));
	std::shared_ptr<BaseSystemInterface> base_simulator_ptr = std::shared_ptr<BaseSystemInterface>((BaseSystemInterface*)basesimulator);

	InterfaceYAMLConfig config;
	config.ip = "12.34.56.78";
	config.port = 1000;
	std::vector<VariableDefinition> inputs;
	std::vector<VariableDefinition> outputs;

	//test values
	std::string stringValue = "A string for CoSiMa";
	bool boolValue = true;
	int integerValue = -1234;
	float floatValue = -1.234f;
	double doubleValue = 4.567;

	VariableDefinition definition;
	std::vector<std::string> inputVarNames;
	std::vector<std::string> outputVarNames;

	definition.type = "string";
	basesimulator->stringvalue = stringValue;
	definition.base_name = "base.string_in";
	definition.interface_name = "string_in";
	inputs.push_back(definition);
	inputVarNames.push_back(definition.interface_name);
	definition.base_name = "base.string_out";
	definition.interface_name = "string_out";
	outputs.push_back(definition);
	outputVarNames.push_back(definition.interface_name);

	definition.type = "bool";
	basesimulator->boolvalue = boolValue;
	definition.base_name = "base.bool_in";
	definition.interface_name = "bool_in";
	inputs.push_back(definition);
	inputVarNames.push_back(definition.interface_name);
	definition.base_name = "base.bool_out";
	definition.interface_name = "bool_out";
	outputs.push_back(definition);
	outputVarNames.push_back(definition.interface_name);

	definition.type = "integer";
	basesimulator->intvalue = integerValue;
	definition.base_name = "base.integer_in";
	definition.interface_name = "integer_in";
	inputs.push_back(definition);
	inputVarNames.push_back(definition.interface_name);
	definition.base_name = "base.integer_out";
	definition.interface_name = "integer_out";
	outputs.push_back(definition);
	outputVarNames.push_back(definition.interface_name);

	definition.type = "float";
	basesimulator->floatvalue = floatValue;
	definition.base_name = "base.float_in";
	definition.interface_name = "float_in";
	inputs.push_back(definition);
	inputVarNames.push_back(definition.interface_name);
	definition.base_name = "base.float_out";
	definition.interface_name = "float_out";
	outputs.push_back(definition);
	outputVarNames.push_back(definition.interface_name);

	definition.type = "double";
	basesimulator->doublevalue = doubleValue;
	definition.base_name = "base.double_in";
	definition.interface_name = "double_in";
	inputs.push_back(definition);
	inputVarNames.push_back(definition.interface_name);
	definition.base_name = "base.double_out";
	definition.interface_name = "double_out";
	outputs.push_back(definition);
	outputVarNames.push_back(definition.interface_name);

	config.inputs = inputs;
	config.outputs = outputs;

	interface_simulator->getMapper()->setOwner(interface_simulator);
	interface_simulator->getMapper()->readConfiguration(config);
	auto state = interface_simulator->getMapper()->getInternalState();

	// one input and output in each vector
	REQUIRE(2 == state->strings.size());
	REQUIRE(2 == state->bools.size());
	REQUIRE(2 == state->integers.size());
	REQUIRE(2 == state->floats.size());
	REQUIRE(2 == state->doubles.size());

	// variable space in vectors is initialized using the type's default
	REQUIRE(std::all_of(state->strings.begin(), state->strings.end(), [](std::string s) {return s.empty(); }));
	REQUIRE(std::all_of(state->bools.begin(), state->bools.end(), [](bool b) {return bool() == b; }));
	REQUIRE(std::all_of(state->integers.begin(), state->integers.end(), [](int i) {return int() == i; }));
	REQUIRE(std::all_of(state->floats.begin(), state->floats.end(), [](float f) {return float() == f; }));
	REQUIRE(std::all_of(state->doubles.begin(), state->doubles.end(), [](double d) {return double() == d; }));

	// Update input values in internal state
	mapper->searchInput(base_simulator_ptr);

	// still one input and output in each vector
	REQUIRE(2 == state->strings.size());
	REQUIRE(2 == state->bools.size());
	REQUIRE(2 == state->integers.size());
	REQUIRE(2 == state->floats.size());
	REQUIRE(2 == state->doubles.size());

	// All 5 inputs should have been requested from the base system
	REQUIRE(5 == basesimulator->requestedVariables.size());
	REQUIRE(std::all_of(basesimulator->requestedVariables.begin(), basesimulator->requestedVariables.end(), [&inputVarNames](std::string baseName) {
		return std::any_of(inputVarNames.begin(), inputVarNames.end(), [&baseName](std::string interfaceName) {
			// baseName ends with interfaceName?
			return baseName.size() >= interfaceName.size()
				&& 0 == baseName.substr(baseName.size() - interfaceName.size(), interfaceName.size()).compare(interfaceName);
		});
	}));

	// values of inputs are now those defined in the base system
	REQUIRE(std::get<std::string>(mapper->mapFromInternalState("string_in", eDataType::STRINGCOSIMA)) == stringValue);
	REQUIRE(std::get<bool>(mapper->mapFromInternalState("bool_in", eDataType::BOOLCOSIMA)) == boolValue);
	REQUIRE(std::get<int>(mapper->mapFromInternalState("integer_in", eDataType::INTEGERCOSIMA)) == integerValue);
	REQUIRE(std::get<float>(mapper->mapFromInternalState("float_in", eDataType::FLOATCOSIMA)) == floatValue);
	REQUIRE(std::get<double>(mapper->mapFromInternalState("double_in", eDataType::DOUBLECOSIMA)) == doubleValue);

	//write output values to base system
	mapper->writeOutput(base_simulator_ptr);

	// values of outputs didn't change
	REQUIRE(0 == basesimulator->stringvalue.compare(std::string()));
	REQUIRE(basesimulator->boolvalue == bool());
	REQUIRE(basesimulator->intvalue == int());
	REQUIRE(basesimulator->floatvalue == float());
	REQUIRE(basesimulator->doublevalue == double());

	//TODO write feedthrough mock interface simulator to be able to test for non default values in outputs
}