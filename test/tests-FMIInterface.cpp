#include "catch2/catch.hpp"

#include "mapper/Mapper.h"
#include "mapper/FMIMapper.h"
#include "simulation_interfaces/FMIBridge.h"
#include "configreader/StandardYAMLConfig.h"
#include "MockBaseSimulator.cpp"
#include "MockInterfaceSimulator.cpp"

TEST_CASE("FMIBridge: Read FMI simulator attributes from config and load FMU", "[FMIBridge][FMIMapper]") {

	// static conversion is impossible because of private inheritance
	//std::shared_ptr<Mapper> mapper = std::static_pointer_cast<Mapper>(std::make_shared<FMIMapper>());
	//std::shared_ptr<BaseSystemInterface> baseSystem = std::static_pointer_cast<BaseSystemInterface>(std::make_shared<MockBaseSimulator>(mapper));
	//auto fmiBridge = std::make_shared<FMIBridge>(mapper);
	//std::shared_ptr<iSimulationData> simulationInterface = std::static_pointer_cast<iSimulationData>(fmiBridge);
	const auto fmiMapper = new FMIMapper();
	auto mapper = std::shared_ptr<Mapper>((Mapper*)fmiMapper);
	auto baseInterface = new MockBaseSimulator();
	auto baseSystem = std::shared_ptr<BaseSystemInterface>((BaseSystemInterface*)baseInterface);
	const auto fmiBridge = new FMIBridge(mapper);
	std::shared_ptr<iSimulationData> simulationInterface = std::shared_ptr<iSimulationData>((iSimulationData*)fmiBridge);
	mapper->setOwner(simulationInterface);

	FMIInterfaceConfig config;
	config.simulator = "FMI";

	SECTION("CoSim-only FMU should load fine") {
		config.models = "../test/resources/Feedthrough_cs.fmu";

		REQUIRE(0 == mapper->readConfiguration(config));
		REQUIRE(0 == simulationInterface->init("A co-simulation fmu", 0, 0));

		auto state = simulationInterface->getMapper()->getInternalState();

		std::vector<std::string> inputNames = {
			"real_fixed_param",
			"real_tunable_param",
			"real_continuous_in",
			"real_discrete_in",
			"int_in",
			"bool_in",
			"string_param" };
		std::vector<std::string> outputNames = {
			"real_continuous_out",
			"real_discrete_out",
			"int_out",
			"bool_out" };


		SECTION("Use different values as fmu input: variant 0") {
			const bool boolvalue = true;
			const double doublevalue = 1.23;
			const float floatvalue = 3.21f;
			const int intvalue = 2;
			const std::string stringvalue = "A CAPITAL STRING";
			baseInterface->boolvalue = boolvalue;
			baseInterface->doublevalue = doublevalue;
			baseInterface->floatvalue = floatvalue;
			baseInterface->intvalue = intvalue;
			baseInterface->stringvalue = stringvalue;

			REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
			REQUIRE(0 == simulationInterface->readFromInternalState());

			REQUIRE(std::all_of(inputNames.begin(), inputNames.end(), [&baseInterface](const std::string n) {
				return std::any_of(baseInterface->requestedVariables.begin(), baseInterface->requestedVariables.end(),
					[&n](std::string b) { return 0 == ("Feedthrough." + n).compare(b); });
			}));
			//Inputs are not written to internal state => no need to test them
			//REQUIRE(1 == state->strings.size());
			//REQUIRE(0 == stringvalue.compare(state->strings.front()));
			//REQUIRE(std::all_of(state->bools.begin(), state->bools.end(), [boolvalue](const bool b) {return b == boolvalue; }));
			//REQUIRE(std::all_of(state->doubles.begin(), state->doubles.end(), [doublevalue](const double d) {return d == doublevalue; }));
			//REQUIRE(std::all_of(state->floats.begin(), state->floats.end(), [floatvalue](const float f) {return f == floatvalue; }));
			//REQUIRE(std::all_of(state->integers.begin(), state->integers.end(), [intvalue](const int i) {return i == intvalue; }));

			REQUIRE(0 == simulationInterface->doStep(1e-5));
			REQUIRE(0 == simulationInterface->writeToInternalState());
			REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));

			// Feedthrough shouldn't change the values, same for the FMU interface
			REQUIRE(doublevalue == baseInterface->doublevalue);
			REQUIRE(floatvalue == baseInterface->floatvalue);
			REQUIRE(intvalue == baseInterface->intvalue);
			// Feedthrough quirk: bool_out only returns bool_in if string_param is set to "FMI is awesome!"
			REQUIRE(false == baseInterface->boolvalue);
			baseInterface->stringvalue = "FMI is awesome!";
			baseInterface->boolvalue = boolvalue;
			REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
			REQUIRE(0 == simulationInterface->readFromInternalState());
			baseInterface->boolvalue = !boolvalue;
			REQUIRE(0 == simulationInterface->writeToInternalState());
			REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));
			REQUIRE(boolvalue == baseInterface->boolvalue);
		}
		SECTION("Use different values as fmu input: variant a") {
			const bool boolvalue = false;
			const double doublevalue = 1e23;
			const float floatvalue = 3.14f;
			const int intvalue = 1;
			const std::string stringvalue = "another string";
			baseInterface->boolvalue = boolvalue;
			baseInterface->doublevalue = doublevalue;
			baseInterface->floatvalue = floatvalue;
			baseInterface->intvalue = intvalue;
			baseInterface->stringvalue = stringvalue;

			REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
			REQUIRE(0 == simulationInterface->readFromInternalState());

			REQUIRE(std::all_of(inputNames.begin(), inputNames.end(), [&baseInterface](const std::string n) {
				return std::any_of(baseInterface->requestedVariables.begin(), baseInterface->requestedVariables.end(),
					[&n](std::string b) { return 0 == ("Feedthrough." + n).compare(b); });
			}));
			//Inputs are not written to internal state => no need to test them
			//REQUIRE(1 == state->strings.size());
			//REQUIRE(0 == stringvalue.compare(state->strings.front()));
			//REQUIRE(std::all_of(state->bools.begin(), state->bools.end(), [boolvalue](const bool b) {return b == boolvalue; }));
			//REQUIRE(std::all_of(state->doubles.begin(), state->doubles.end(), [doublevalue](const double d) {return d == doublevalue; }));
			//REQUIRE(std::all_of(state->floats.begin(), state->floats.end(), [floatvalue](const float f) {return f == floatvalue; }));
			//REQUIRE(std::all_of(state->integers.begin(), state->integers.end(), [intvalue](const int i) {return i == intvalue; }));

			REQUIRE(0 == simulationInterface->doStep(1e-3));
			REQUIRE(0 == simulationInterface->writeToInternalState());
			REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));

			// Feedthrough shouldn't change the values, same for the FMU interface
			REQUIRE(doublevalue == baseInterface->doublevalue);
			REQUIRE(floatvalue == baseInterface->floatvalue);
			REQUIRE(intvalue == baseInterface->intvalue);
			// Feedthrough quirk: bool_out only returns bool_in if string_param is set to "FMI is awesome!"
			REQUIRE(false == baseInterface->boolvalue);
			baseInterface->stringvalue = "FMI is awesome!";
			baseInterface->boolvalue = boolvalue;
			REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
			REQUIRE(0 == simulationInterface->readFromInternalState());
			REQUIRE(0 == simulationInterface->writeToInternalState());
			REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));
			REQUIRE(boolvalue == baseInterface->boolvalue);
		}
	}
	SECTION("ME-only FMUs are not supported and should fail to load") {
		config.models = "../test/resources/Feedthrough_me.fmu";
		REQUIRE(216373 == mapper->readConfiguration(config));
	}
	SECTION("Dual definition FMUs can also be imported for coSim") {
		config.models = "../test/resources/Stair.fmu";

		REQUIRE(0 == mapper->readConfiguration(config));
		//starting the experiment at timepoint 1s
		REQUIRE(0 == simulationInterface->init("A co-simulation fmu", 1, 0));

		SECTION("Negative stepSize should fail") {
			REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
			REQUIRE(0 != simulationInterface->doStep(-1e-3));
		}
		SECTION("Positive stepSize is OK") {
			// Stairs.fmu has a single scalar variable of causality output and type integer named 'counter' that increases with every progressed second from time t = 0
			SECTION("Two steps of 0.8s") {
				baseInterface->intvalue = -1;

				REQUIRE(0 == simulationInterface->mapToInterfaceSystem(baseSystem));
				REQUIRE(0 == simulationInterface->doStep(.8f));
				REQUIRE(0 == simulationInterface->writeToInternalState());
				REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));

				//output 'counter' depends on time t. At t=0: counter = 0, for t in (0,1]: counter=1, for t in (1,2]: counter=2,...
				REQUIRE(2 == baseInterface->intvalue);//t = 1.8

				//REQUIRE(0 == simulationInterface->mapInput(baseSystem));
				REQUIRE(0 == simulationInterface->doStep(.8f));
				REQUIRE(0 == simulationInterface->writeToInternalState());
				REQUIRE(0 == simulationInterface->mapFromInterfaceSystem(baseSystem));

				REQUIRE(3 == baseInterface->intvalue);//t = 2.6
			}
		}
	}
}