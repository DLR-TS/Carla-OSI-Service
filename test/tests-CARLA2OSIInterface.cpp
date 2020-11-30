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
		world = client->ReloadWorld();
		world.WaitForTick(std::chrono::seconds(45));

		//find a prop and spawn it in the current world to assert it contains an actor of type prop
		auto blueprintLibrary = world.GetBlueprintLibrary();
		auto prop = blueprintLibrary->Find("static.prop.barbeque");
		auto randomLocation = world.GetRandomLocationFromNavigation();
		auto fallbackLocation = carla::geom::Location(0, 0, 1);
		world.SpawnActor(*prop, randomLocation.value_or(fallbackLocation));

		carla->initialise(host, port, transactionTimeout, deltaSeconds);

	}
}

TEST_CASE("Parsing of added vehicle attributes for osi3::MovingObject", "[.][RequiresCarlaServer][MovingObject]") {

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
	else {
		//clear world
		world = client->ReloadWorld();
	}

	//spawn vehicles
	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto vehicleBlueprints = blueprintLibrary->Filter("vehicle.*");
	auto recommendedSpawnPoints = world.GetMap()->GetRecommendedSpawnPoints();
	for (size_t i = 0; i < vehicleBlueprints->size() && i < recommendedSpawnPoints.size(); i++) {
		auto vehicleBlueprint = vehicleBlueprints->at(i);
		auto actor = world.SpawnActor(vehicleBlueprint, recommendedSpawnPoints.at(i));
	}

	// compare ground truth to vehicles
	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	carla->initialise(host, port, transactionTimeout, deltaSeconds);
	auto groundTruth = carla->getLatestGroundTruth();
	REQUIRE(std::min(vehicleBlueprints->size(), recommendedSpawnPoints.size()) == groundTruth->moving_object_size());
	for (auto& movingObject : groundTruth->moving_object()) {
		auto actor = world.GetActor(movingObject.id().value());
		auto vehicle = boost::static_pointer_cast<carla::client::Vehicle>(actor);
		auto bbox = vehicle->GetBoundingBox();
		float wheel_radius = -1;
		carla::geom::Location bbcenter_to_front, bbcenter_to_rear;
		// loop over attributes and reconstruct bbcenter_to_X
		for (auto attribute : actor->GetAttributes()) {
			auto id = attribute.GetId();
			if (0 == id.rfind("bbcenter_to", 0)) {
				if ('f' == id.at(12)) {
					if ('x' == id.back()) {
						CHECK(Approx(0) != attribute.As<float>());
						bbcenter_to_front.x = attribute.As<float>();
					}
					else if ('y' == id.back()) {
						CHECK(1 > attribute.As<float>());
						bbcenter_to_front.y = attribute.As<float>();
					}
					else if ('z' == id.back()) {
						CHECK(Approx(0) != attribute.As<float>());
						bbcenter_to_front.z = attribute.As<float>();
					}
				}
				else if ('r' == id.at(12)) {
					if ('x' == id.back()) {
						CHECK(Approx(0) != attribute.As<float>());
						bbcenter_to_rear.x = attribute.As<float>();
					}
					else if ('y' == id.back()) {
						CHECK(1 > attribute.As<float>());
						bbcenter_to_rear.y = attribute.As<float>();
					}
					else if ('z' == id.back()) {
						CHECK(Approx(0) != attribute.As<float>());
						bbcenter_to_rear.z = attribute.As<float>();
					}
				}
			}
			else if (0 == id.rfind("wheel_radius", 0)) {
				wheel_radius = attribute.As<float>();
			}
		}
		CHECK(0 < wheel_radius);
	
		REQUIRE(0 < groundTruth->moving_object_size());
		CarlaUtility::IDUnion expectedOSIId{ 1ULL << (/*16 + */32) | actor->GetId() };
		REQUIRE(actor->GetId() == expectedOSIId.id);
		REQUIRE(CarlaUtility::CarlaUniqueID_e::ActorID == expectedOSIId.type);
		REQUIRE(0 == expectedOSIId.special);
		REQUIRE(movingObject.has_base());
		auto base = movingObject.base();
		REQUIRE(base.has_position());
		REQUIRE(CarlaUtility::toCarla(&base.position()) == (vehicle->GetLocation() + bbox.location));
		REQUIRE(base.has_dimension());
		REQUIRE(CarlaUtility::toCarla(&base.dimension(), &base.position()).extent == bbox.extent);
		REQUIRE(base.has_orientation());
		auto osiRotation = CarlaUtility::toCarla(&base.orientation());
		auto transform = vehicle->GetTransform();
		REQUIRE(osiRotation.pitch == transform.rotation.pitch);
		REQUIRE(osiRotation.yaw == transform.rotation.yaw);
		REQUIRE(osiRotation.roll == transform.rotation.roll);
		REQUIRE(movingObject.has_vehicle_classification());
		auto classification = movingObject.vehicle_classification();
		REQUIRE(classification.has_type());
		REQUIRE(osi3::MovingObject_VehicleClassification_Type_TYPE_UNKNOWN != classification.type());
		REQUIRE(movingObject.has_vehicle_attributes());
		auto attributes = movingObject.vehicle_attributes();
		REQUIRE(CarlaUtility::toCarla(&attributes.bbcenter_to_front()) == bbcenter_to_front);
		REQUIRE(CarlaUtility::toCarla(&attributes.bbcenter_to_rear()) == bbcenter_to_rear);
		REQUIRE(attributes.has_number_wheels());
		REQUIRE(((4 == attributes.number_wheels()) || (2 == attributes.number_wheels())));
		REQUIRE(attributes.has_radius_wheel());
		REQUIRE(Approx(wheel_radius) == attributes.radius_wheel());
	}
}

