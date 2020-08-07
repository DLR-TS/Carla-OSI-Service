#include "catch2/catch.hpp"

#include "configreader/YAMLConfigReader.h"
#include "simulation_interfaces/VTDBridge.h"
#include "mapper/VTDMapper.h"

TEST_CASE("Read simulator names from config") {
	YAMLConfigReader reader("../test/resources/testconfig1.yaml");

	std::vector<SingleYAMLConfig> names = reader.getSimulatorNames();
	REQUIRE(names.size() == 3);
	REQUIRE(names.at(0).simulator == VTD);
	REQUIRE(names.at(0).index == 0);
	REQUIRE(names.at(1).simulator == FMI);
	REQUIRE(names.at(1).index == 0);
	REQUIRE(names.at(2).simulator == VTD);
	REQUIRE(names.at(2).index == 1);
}

TEST_CASE("set configuration of simulator") {
	YAMLConfigReader reader("../test/resources/testconfig1.yaml");

	SingleYAMLConfig conf;
	conf.index = 1;
	conf.simulator = eSimulatorName::VTD;

	std::shared_ptr<iSimulationData> vtdSimulator = std::shared_ptr<iSimulationData>((iSimulationData*) new VTDBridge(std::shared_ptr<Mapper>((Mapper*)new VTDMapper())));
	vtdSimulator->getMapper()->setOwner(vtdSimulator);

	REQUIRE(reader.setConfig(vtdSimulator, conf) == 0);
}

TEST_CASE("set invalid configuration of simulator") {
	YAMLConfigReader reader("../test/resources/testconfig1.yaml");

	SingleYAMLConfig conf;
	conf.index = 1;
	conf.simulator = eSimulatorName::SIMULATORNAME_ERROR;

	std::shared_ptr<iSimulationData> vtdSimulator = std::shared_ptr<iSimulationData>((iSimulationData*) new VTDBridge(std::shared_ptr<Mapper>((Mapper*)new VTDMapper())));

	REQUIRE(reader.setConfig(vtdSimulator, conf) == 1);
}
