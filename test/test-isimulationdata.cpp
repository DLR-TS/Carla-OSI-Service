#include "catch2/catch.hpp"

#include "simulation_interfaces/iSimulationData.h"
#include "simulation_interfaces/FMIBridge.h"
#include "mapper/FMIMapper.h"

TEST_CASE("internalstate is available") {
	std::shared_ptr<Mapper> mapper = std::shared_ptr<Mapper>((Mapper*)new FMIMapper());
	iSimulationData* simulationdata = (iSimulationData*) new FMIBridge(mapper);

	simulationdata->getMapper()->getInternalState()->bools.push_back(true);
	simulationdata->getMapper()->getInternalState()->bools.push_back(false);
	REQUIRE(simulationdata->getMapper()->getInternalState()->bools.at(0) == true);
	REQUIRE(simulationdata->getMapper()->getInternalState()->bools.at(1) == false);
	simulationdata->getMapper()->getInternalState()->doubles.push_back(0.0);
	simulationdata->getMapper()->getInternalState()->doubles.push_back(1000.5);
	REQUIRE(simulationdata->getMapper()->getInternalState()->doubles.at(0) == 0.0);
	REQUIRE(simulationdata->getMapper()->getInternalState()->doubles.at(1) == 1000.5);
	simulationdata->getMapper()->getInternalState()->floats.push_back(0.67f);
	simulationdata->getMapper()->getInternalState()->floats.push_back(-7.9f);
	REQUIRE(simulationdata->getMapper()->getInternalState()->floats.at(0) == 0.67f);
	REQUIRE(simulationdata->getMapper()->getInternalState()->floats.at(1) == -7.9f);
	simulationdata->getMapper()->getInternalState()->integers.push_back(10056);
	simulationdata->getMapper()->getInternalState()->integers.push_back(-59574);
	REQUIRE(simulationdata->getMapper()->getInternalState()->integers.at(0) == 10056);
	REQUIRE(simulationdata->getMapper()->getInternalState()->integers.at(1) == -59574);
	simulationdata->getMapper()->getInternalState()->strings.push_back("base_name");
	simulationdata->getMapper()->getInternalState()->strings.push_back("test1");
	REQUIRE(simulationdata->getMapper()->getInternalState()->strings.at(0) == "base_name");
	REQUIRE(simulationdata->getMapper()->getInternalState()->strings.at(1) == "test1");
}

TEST_CASE("set mapper") {
	std::shared_ptr<Mapper> mapper = std::shared_ptr<Mapper>((Mapper*)new FMIMapper());
	iSimulationData* simulationdata = (iSimulationData*) new FMIBridge(mapper);

	REQUIRE(simulationdata->getMapper() == mapper);
}
