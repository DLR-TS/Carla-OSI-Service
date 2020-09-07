#include "catch2/catch.hpp"

#include "CARLA2OSIInterface.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

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

	SECTION("Init with generated static props") {
		auto client = std::make_unique<carla::client::Client>(config.host, config.port);
		client->SetTimeout(std::chrono::duration<double>(config.transactionTimeout));
		auto world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

		//find a prop and spawn it in the current world to assert it contains an actor of type prop
		auto blueprintLibrary = world->GetBlueprintLibrary();
		auto prop = blueprintLibrary->Find("static.prop.barbeque");
		auto randomLocation = world->GetRandomLocationFromNavigation();
		auto fallbackLocation = carla::geom::Location(0, 0, 1);
		world->SpawnActor(*prop, randomLocation.value_or(fallbackLocation));

		carla->initialise();

	}
}

