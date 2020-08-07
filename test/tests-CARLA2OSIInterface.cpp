#include "catch2/catch.hpp"

#include "CARLA2OSIINterface.h"

TEST_CASE("CARLAInterface", "[CARLAInterface][.][RequiresCarlaServer]") {
	// host, port, transaction timeout, delta seconds (1/framerate)
	CARLA2OSIInterfaceConfig config{ "localhost", 2000u, 5, (1.0 / 60) };

	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	REQUIRE(0 == carla->readConfiguration(config));

	// private members -> no access
	//REQUIRE(2000u == carla->port);
	//REQUIRE(0 == std::strcomp("localhost", carla->host));

	SECTION("Init") {
		carla->initialise();
	}
}

