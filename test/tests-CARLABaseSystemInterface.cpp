#include "catch2/catch.hpp"

#include "base_interfaces/CARLAInterface.h"
#include "configreader/StandardYamlConfig.h"

TEST_CASE("CARLAInterface", "[CARLAInterface][.][RequiresCarlaServer]") {
	// host, port, transaction timeout, delta seconds (1/framerate)
	CARLAInterfaceConfig config{ "localhost", 2000u, 5, (1.0 / 60) };

	std::shared_ptr<CARLAInterface> carla = std::make_shared<CARLAInterface>();
	REQUIRE(0 == carla->readConfiguration(config));

	// private members -> no access
	//REQUIRE(2000u == carla->port);
	//REQUIRE(0 == std::strcomp("localhost", carla->host));

	SECTION("Init") {
		carla->initialise();
	}
}

TEST_CASE("CARLAInterface, wrong config", "[CARLAInterface]") {
	baseConfigVariants_t config = DominionInterfaceConfig();

	std::shared_ptr<CARLAInterface> carla = std::make_shared<CARLAInterface>();
	REQUIRE(1 == carla->readConfiguration(config));
}

