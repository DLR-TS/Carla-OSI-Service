#include "catch2/catch.hpp"

#include "CARLA2OSIInterface.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

TEST_CASE("CARLA2OSIInterface", "[CARLAInterface][.][RequiresCarlaServer]") {
	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	
	// carla server
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 5;
	// delta seconds (1/framerate)
	double deltaSeconds = (1.0 / 60);

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto timeout = std::chrono::duration<double>(transactionTimeout);
	auto client = std::make_unique<carla::client::Client>(host, port);
	client->SetTimeout(timeout);
	auto world = client->GetWorld();
	if (world.GetMap()->GetName().rfind("Town", 0) == std::string::npos) {
		std::cout << "Destroying current world '" << world.GetMap()->GetName() << "' to load world 'Town10HD'" << std::endl;
		world = client->LoadWorld("Town10HD");
		world.WaitForTick(std::chrono::seconds(45));
	}

	SECTION("Init") {
		carla->initialise(host, port, transactionTimeout, deltaSeconds);
	}

	SECTION("Init with generated static props") {
		auto client = std::make_unique<carla::client::Client>(host, port);
		client->SetTimeout(std::chrono::duration<double>(transactionTimeout));
		auto world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

		//find a prop and spawn it in the current world to assert it contains an actor of type prop
		auto blueprintLibrary = world->GetBlueprintLibrary();
		auto prop = blueprintLibrary->Find("static.prop.barbeque");
		auto randomLocation = world->GetRandomLocationFromNavigation();
		auto fallbackLocation = carla::geom::Location(0, 0, 1);
		world->SpawnActor(*prop, randomLocation.value_or(fallbackLocation));

		carla->initialise(host, port, transactionTimeout, deltaSeconds);

	}
}

