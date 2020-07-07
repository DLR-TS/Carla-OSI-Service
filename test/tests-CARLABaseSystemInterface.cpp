#include "catch2/catch.hpp"

#include "base_interfaces/CARLAInterface.h"
#include "configreader/StandardYamlConfig.h"

TEST_CASE("CARLAInterface", "[CARLAInterface]") {
	CARLAInterfaceConfig config{ "localhost", 2000u };

	std::shared_ptr<CARLAInterface> carla = std::make_shared<CARLAInterface>();
	REQUIRE(0==carla->readConfiguration(config));

	// private members -> no access
	//REQUIRE(2000u == carla->port);
	//REQUIRE(0 == std::strcomp("localhost", carla->host));
}

TEST_CASE("CARLAInterface, wrong config", "[CARLAInterface]") {
	baseConfigVariants_t config;

	std::shared_ptr<CARLAInterface> carla = std::make_shared<CARLAInterface>();
	REQUIRE(1 == carla->readConfiguration(config));
}