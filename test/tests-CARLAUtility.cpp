#include "catch2/catch.hpp"

#include "Utility.h"
#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"
#include "carla_osi/TrafficSignals.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

#include <carla/Debug.h>

TEST_CASE("Two way difference", "[TwoWayDifference][Utility]") {
	SECTION("All empty") {
		std::vector<int> old, updated, added, removed;

		auto[added_end, removed_end] = CarlaUtility::twoWayDifference(old.begin(), old.end(), updated.begin(), updated.end(), std::inserter(added, added.begin()), std::inserter(removed, removed.begin()));
		//REQUIRE(added.end() == added_end);
		//REQUIRE(removed.end() == removed_end);
		REQUIRE(std::empty(old));
		REQUIRE(std::empty(updated));
		REQUIRE(std::empty(added));
		REQUIRE(std::empty(removed));
	}
	SECTION("different range") {
		std::vector<int> old{ 1,2,3 }, updated{ 2,3,4 }, added, removed;

		auto[added_end, removed_end] = CarlaUtility::twoWayDifference(old.begin(), old.end(), updated.begin(), updated.end(), std::inserter(added, added.begin()), std::inserter(removed, removed.begin()));
		//REQUIRE(added.end() == added_end);
		//REQUIRE(removed.end() == removed_end);
		REQUIRE(3 == std::size(old));
		REQUIRE(3 == std::size(updated));
		REQUIRE(1 == std::size(added));
		REQUIRE(4 == *added.begin());
		REQUIRE(1 == std::size(removed));
		REQUIRE(1 == *removed.begin());
	}
	SECTION("difference not at border") {
		std::vector<int> old{ 1,3,4 }, updated{ 1,2,4 }, added, removed;

		auto[added_end, removed_end] = CarlaUtility::twoWayDifference(old.begin(), old.end(), updated.begin(), updated.end(), std::inserter(added, added.begin()), std::inserter(removed, removed.begin()));
		//REQUIRE(added.end() == added_end);
		//REQUIRE(removed.end() == removed_end);
		REQUIRE(3 == std::size(old));
		REQUIRE(3 == std::size(updated));
		REQUIRE(1 == std::size(added));
		REQUIRE(2 == *added.begin());
		REQUIRE(1 == std::size(removed));
		REQUIRE(3 == *removed.begin());
	}
}

TEST_CASE("Coordinate system conversion Carla <=> OSI", "[Carla][Utility][Coords]") {
	SECTION("toOSI") {
		SECTION("Rotation") {
			carla::geom::Rotation rotation(45, -90, 180);
			std::unique_ptr<osi3::Orientation3d> orientation = Geometry::getInstance()->toOSI(rotation);
			REQUIRE(M_PI_4 == orientation->pitch());
			REQUIRE(M_PI_2 == orientation->yaw());
			REQUIRE(M_PI == orientation->roll());
		}

		SECTION("Location") {
			carla::geom::Location location(1.2f, 3.4f, 5.6f);
			std::unique_ptr<osi3::Vector3d> position = Geometry::getInstance()->toOSI(location);
			REQUIRE(1.2f == position->x());
			REQUIRE(-3.4f == position->y());
			REQUIRE(5.6f == position->z());
		}

		SECTION("MountingPosition") {
			carla::geom::Location location(1.2f, 3.4f, 5.6f);
			carla::geom::Rotation rotation(45, -90, 180);
			carla::geom::Transform transform(location, rotation);
			std::unique_ptr<osi3::MountingPosition> mountingPosition = Geometry::getInstance()->toOSI(transform);
			auto position = mountingPosition->position();
			auto orientation = mountingPosition->orientation();
			REQUIRE(1.2f == position.x());
			REQUIRE(-3.4f == position.y());
			REQUIRE(5.6f == position.z());
			REQUIRE(M_PI_4 == orientation.pitch());
			REQUIRE(M_PI_2 == orientation.yaw());
			REQUIRE(M_PI == orientation.roll());
		}

		SECTION("BoundingBox") {
			carla::geom::Location location(1.2f, 3.4f, 5.6f);
			carla::geom::BoundingBox boundingBox(location, carla::geom::Vector3D(1.f, 2.f, 4.f));
			auto osiBB = Geometry::getInstance()->toOSI(boundingBox);
			REQUIRE(1.2f == osiBB.second->x());
			REQUIRE(-3.4f == osiBB.second->y());
			REQUIRE(5.6f == osiBB.second->z());
			REQUIRE(2.f == osiBB.first->length());
			REQUIRE(4.f == osiBB.first->width());
			REQUIRE(8.f == osiBB.first->height());
		}

		SECTION("ActorID") {
			//actorID is unsigned
			carla::ActorId actorID(-2);
			std::unique_ptr<osi3::Identifier> identifier = carla_osi::id_mapping::toOSI(actorID, carla_osi::id_mapping::ActorID);
			carla_osi::id_mapping::IDUnion value;
			value.value = identifier->value();
			REQUIRE(8589934590ull == value.value);
			REQUIRE(-2 == value.id);
			REQUIRE(0 == value.special);
			REQUIRE(carla_osi::id_mapping::ActorID == value.type);
		}
		SECTION("ActorID (implicit type)") {
			//actorID is unsigned
			carla::ActorId actorID(1234567890u);
			std::unique_ptr<osi3::Identifier> identifier = carla_osi::id_mapping::toOSI(actorID);
			carla_osi::id_mapping::IDUnion value;
			value.value = identifier->value();
			REQUIRE(5529535186ull == value.value);
			REQUIRE(1234567890u == value.id);
			REQUIRE(0 == value.special);
			REQUIRE(carla_osi::id_mapping::ActorID == value.type);
		}

		SECTION("Roads and Junctions") {
			//LaneID is signed
			SECTION("RoadID and LaneID") {
				carla::road::RoadId roadID(-1234567890);
				carla::road::LaneId laneID(-7);
				std::unique_ptr<osi3::Identifier> identifier = carla_osi::id_mapping::toOSI(roadID, laneID);
				carla_osi::id_mapping::IDUnion value;
				value.value = identifier->value();
				REQUIRE(0xF902B669FD2Eull == value.value);
				REQUIRE(-1234567890 == value.id);
				REQUIRE(-7 == value.special);
				REQUIRE(carla_osi::id_mapping::RoadIDLaneID == value.type);
			}
			SECTION("JuncID") {
				carla::road::JuncId juncID(-1234567890);
				std::unique_ptr<osi3::Identifier> identifier = carla_osi::id_mapping::toOSI(juncID, carla_osi::id_mapping::JuncID);
				carla_osi::id_mapping::IDUnion value;
				value.value = identifier->value();
				REQUIRE(0x3B669FD2Eull == value.value);
				REQUIRE(-1234567890 == value.id);
				REQUIRE(0 == value.special);
				REQUIRE(carla_osi::id_mapping::JuncID == value.type);
			}
		}
	}
	SECTION("toCarla") {
		SECTION("Rotation") {
			osi3::Orientation3d orientation;
			orientation.set_pitch(M_PI_4);
			orientation.set_yaw(M_PI_2);
			orientation.set_roll(M_PI - M_PI_4);
			carla::geom::Rotation rotation = Geometry::getInstance()->toCarla(orientation);
			REQUIRE(45.f == rotation.pitch);
			REQUIRE(-90.f == rotation.yaw);
			REQUIRE(135.f == rotation.roll);
		}

		SECTION("Location") {
			osi3::Vector3d position;
			position.set_x(0.1);
			position.set_y(2.3);
			position.set_z(4.5);
			carla::geom::Location location = Geometry::getInstance()->toCarla(position);
			REQUIRE(0.1f == location.x);
			REQUIRE(-2.3f == location.y);
			REQUIRE(4.5f == location.z);
		}

		SECTION("BoundingBox") {
			osi3::Vector3d position;
			position.set_x(0.1);
			position.set_y(2.3);
			position.set_z(4.5);
			osi3::Dimension3d dimension;
			dimension.set_length(8.);
			dimension.set_width(4.);
			dimension.set_height(10.);
			carla::geom::BoundingBox boundingBox = Geometry::getInstance()->toCarla(dimension, position);
			REQUIRE(0.1f == boundingBox.location.x);
			REQUIRE(-2.3f == boundingBox.location.y);
			REQUIRE(4.5f == boundingBox.location.z);
			REQUIRE(4.f == boundingBox.extent.x);
			REQUIRE(2.f == boundingBox.extent.y);
			REQUIRE(5.f == boundingBox.extent.z);
		}

		SECTION("IndicatorState") {
			osi3::MovingObject_VehicleClassification_LightState lightState;
			carla::rpc::VehicleLightState::LightState carlaLightState = CarlaUtility::toCarla(&lightState);
			//none
			REQUIRE(carlaLightState == carla::rpc::VehicleLightState::LightState::None);

			//one
			lightState.set_high_beam(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
			carlaLightState = CarlaUtility::toCarla(&lightState);
			REQUIRE(carlaLightState == carla::rpc::VehicleLightState::LightState::HighBeam);

			//two
			lightState.set_brake_light_state(osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL);
			carlaLightState = CarlaUtility::toCarla(&lightState);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::HighBeam);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Brake);

			//set all vehicle intersecting lights between OSI and CARLA
			lightState.set_reversing_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
			lightState.set_front_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
			lightState.set_head_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
			lightState.set_emergency_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
			carlaLightState = CarlaUtility::toCarla(&lightState);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::HighBeam);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Brake);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Reverse);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Fog);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::LowBeam);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Special1);
			REQUIRE((uint32_t)carlaLightState & (uint32_t)carla::rpc::VehicleLightState::LightState::Special2);

		}
	}
}

TEST_CASE("Carla Prop to StationaryObject", "[Carla][Utility][!hide][RequiresCarlaServer]") {
	//TODO find a way to use the ActorFactory without a carla server

	auto client = std::make_unique<carla::client::Client>("localhost", 2000u);
	client->SetTimeout(std::chrono::duration<double>(10));
	auto world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

	//find a prop and spawn it in the current world to assert it contains an actor of type prop
	auto blueprintLibrary = world->GetBlueprintLibrary();
	auto propBlueprint = blueprintLibrary->Find("static.prop.barbeque");
	auto randomLocation = world->GetRandomLocationFromNavigation();
	auto fallbackLocation = carla::geom::Location(0, 0, 1);
	/*carla::SharedPtr<carla::client::Actor> propActor = world->SpawnActor(*propBlueprint, randomLocation.value_or(fallbackLocation));
	auto environmentObject = boost::static_pointer_cast<const carla::rpc::EnvironmentObject>(propActor);
	auto bbox = environmentObject->bounding_box;
	std::string barbeque = "Barbeque";
	auto stationaryObject = CarlaUtility::toOSI(environmentObject, barbeque);

	REQUIRE(stationaryObject->has_classification());
	REQUIRE(stationaryObject->has_base());
	REQUIRE(stationaryObject->has_id());
	REQUIRE(stationaryObject->base().has_dimension());
	REQUIRE(stationaryObject->base().has_orientation());
	REQUIRE(stationaryObject->base().has_position());
	REQUIRE(stationaryObject->classification().has_type());*/
}

TEST_CASE("Carla GetStationaryObject", "[Carla][Utility][!hide][RequiresCarlaServer][GetStationaryObject]") {
	//TODO find a way to use the ActorFactory without a carla server

	auto client = std::make_unique<carla::client::Client>("localhost", 2000u);
	client->SetTimeout(std::chrono::duration<double>(120));
	auto world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

	auto stationaryMapObjects = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Any);

	for (auto stationaryMapObject : stationaryMapObjects) {
		////if (std::string::npos != stationaryMapObject.name.rfind("Jeep"))
		//if (9u == stationaryMapObject.semantic_tag) {
		//	std::cout << stationaryMapObject.name << "  (" << (unsigned int) stationaryMapObject.semantic_tag << ") ";
		//	std::cout << std::endl;
		//	std::cout << "Extent: " << stationaryMapObject.bounding_box.extent.x << ", " << stationaryMapObject.bounding_box.extent.y << ", " << stationaryMapObject.bounding_box.extent.z << std::endl;
		//	std::cout << "Position: " << stationaryMapObject.transform.location.x << ", " << stationaryMapObject.transform.location.y << ", " << stationaryMapObject.transform.location.z << std::endl;
		//}

		REQUIRE(0 < stationaryMapObject.id);
	}
	std::cout << stationaryMapObjects.size() << std::endl;
}

//This test draws bounding boxes around the traffic light bulbs and requires visual verification using the CARLA spectator. It is therefore hidden by default
//Also, the map '04_atCity_AF_DLR_Braunschweig_Prio1_ROD_offset' has to be present on the server. (Can be changed to another OpenDRIVE-based map that uses the default traffic lights)
TEST_CASE("TrafficLight Debug box", "[.][DrawDebugStuff][VisualizationRequiresCarlaServer]") {
	// carla server
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 30;
	auto timeout = std::chrono::duration<double>(transactionTimeout);
	std::string targetMapName = "2020-05-04_atCity_AF_DLR_Braunschweig_Prio1_ROD_offset";
	//std::string targetMapName = "Town10HD";

	auto client = std::make_unique<carla::client::Client>(host, port);
	client->SetTimeout(timeout);
	auto world = client->GetWorld();
	if (world.GetMap()->GetName() != targetMapName) {
		std::cout << "Destroying current world '" << world.GetMap()->GetName() << "' to load world '" + targetMapName + "'" << std::endl;
		world = client->LoadWorld(targetMapName);
	}
	/*else
	{
		world = client->ReloadWorld();
	}*/
	world.Tick(timeout);
	//pugi::xml_document xodr;
	//auto result = xodr.load_string(world.GetMap()->GetOpenDrive().c_str());

	//Get current list of actors
	auto actors = world.GetSnapshot();
	//std::cout << "Actor count: " << actors.size() << std::endl;
	CHECK(0 < actors.size());

	//filter for traffic lights
	std::vector<carla::client::ActorSnapshot> trafficLightActorSnapshots;
	std::copy_if(actors.begin(), actors.end(), std::back_inserter(trafficLightActorSnapshots),
		[&world](auto actor) {
		return "traffic.traffic_light" == world.GetActor(actor.id)->GetTypeId();
	});

	//define bounding box colors
	carla::client::DebugHelper::Color red(255, 0, 0);
	carla::client::DebugHelper::Color yellow(255, 255, 0);
	carla::client::DebugHelper::Color green(0, 255, 0);
	auto debug = world.MakeDebugHelper();

	//Will not draw traffic light bulb bounding boxes without any traffic lights in the map
	CHECK(0 < trafficLightActorSnapshots.size());

	for (auto& trafficLightActor : trafficLightActorSnapshots) {
		auto trafficLight = boost::dynamic_pointer_cast<const carla::client::TrafficLight>(world.GetActor(trafficLightActor.id));
		//auto osiTrafficLight = carla_osi::traffic_signals::getOSITrafficLight(trafficLight/*, xodr*/);
		/*auto heads = world.GetTrafficLightHeads(trafficLight);
		auto osiTrafficLight = carla_osi::traffic_signals::getOSITrafficLight(trafficLight, heads);

		std::cout << trafficLight->GetDisplayId() << std::endl;
		auto transform = trafficLight->GetTransform();
		//std::cout << "Actor rotation: " << transform.rotation.pitch << "," << transform.rotation.yaw << "," << transform.rotation.roll << std::endl;
		//std::cout << "Bulb group" << std::endl;
		CHECK(1 < osiTrafficLight.size());

		for (auto& bulb : osiTrafficLight) {
			auto bbox = carla_osi::geometry::toCarla(bulb->base().dimension(), bulb->base().position());
			auto orientation = carla_osi::geometry::toCarla(bulb->base().orientation());

			// draw box in the respective color
			switch (bulb->classification().color())
			{
			case osi3::TrafficLight_Classification_Color_COLOR_GREEN:
				debug.DrawBox(bbox, orientation, 0.1f, green, 60.f);
				//std::cout << "Green bulb bbox: " << bbox.location.x << "," << bbox.location.y << "," << bbox.location.z << " " << bbox.extent.x << "," << bbox.extent.y << "," << bbox.extent.z << std::endl;
				break;
			case osi3::TrafficLight_Classification_Color_COLOR_YELLOW:
				debug.DrawBox(bbox, orientation, 0.1f, yellow, 60.f);
				//std::cout << "Yellow bulb bbox: " << bbox.location.x << "," << bbox.location.y << "," << bbox.location.z << " " << bbox.extent.x << "," << bbox.extent.y << "," << bbox.extent.z << std::endl;
				break;
			default:
				debug.DrawBox(bbox, orientation, 0.1f, red, 60.f);
				//std::cout << "Red bulb bbox: " << bbox.location.x << "," << bbox.location.y << "," << bbox.location.z << " " << bbox.extent.x << "," << bbox.extent.y << "," << bbox.extent.z << std::endl;
				break;
			}
			//std::cout << "rotation: " << orientation.pitch << "," << orientation.yaw << "," << orientation.roll << std::endl;
		}*/
	}

	//carla->initialise(host, port, transactionTimeout, deltaSeconds, false);
}

TEST_CASE("bbcenter_to_X raw attribute", "[DEBUG][.][TestsCarlaOsiServer][DrawDebugStuff]") {
	//NOT a test of a utility function, but of additional data needed from the carla server
	auto client = std::make_unique<carla::client::Client>("localhost", 2000u);
	auto transactionTimeout = std::chrono::duration<double>(60);
	client->SetTimeout(transactionTimeout);
	auto world = std::make_unique<carla::client::World>(std::move(client->ReloadWorld()));

	// retrieve vehicle blueprints and spawn them
	auto blueprintLibrary = world->GetBlueprintLibrary();
	auto vehicleBlueprints = blueprintLibrary->Filter("vehicle.*");
	auto recommendedSpawnPoints = world->GetMap()->GetRecommendedSpawnPoints();
	for (size_t i = 0, j = 0; i < vehicleBlueprints->size() && j < recommendedSpawnPoints.size(); i++, j++) {
		auto vehicleBlueprint = vehicleBlueprints->at(i);
		carla::SharedPtr<carla::client::Actor> actor;
		do {
			try {
				actor = world->SpawnActor(vehicleBlueprint, recommendedSpawnPoints.at(j));
			}
			catch (std::exception e) {
				std::cout << "Spawn failed: " << e.what() << std::endl;
				j++;
			}
		} while (!actor && j < recommendedSpawnPoints.size());
		std::cout << __FUNCTION__ << ": Spawning " << actor->GetTypeId() << std::endl;
		auto vehicle = boost::static_pointer_cast<carla::client::Vehicle>(actor);
		CHECK(0 < actor->GetId());

		carla::geom::Vector3D bbcenter_to_front;
		carla::geom::Vector3D bbcenter_to_rear;
		float wheel_radius = -1;
		auto attributes = actor->GetAttributes();
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_x"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_y"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_z"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_x"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_y"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_z"; }));
		// build bbcenter_to_front/back vectors and query wheel radius
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
			else if ("wheel_radius" == id) {
				wheel_radius = attribute.As<float>();
			}
		}
		// variable is initialized as -1; expression should be true if attribute was defined
		CHECK(0 <= wheel_radius);

		// draw line for axis using debug helpers
		auto time = world->WaitForTick(transactionTimeout).GetTimestamp();
		carla::geom::Transform vehicleTransform = vehicle->GetTransform();
		carla::geom::BoundingBox vehicleBBox = vehicle->GetBoundingBox();
		auto spectator = world->GetSpectator();
		auto spectatorTransform = spectator->GetTransform();
		spectatorTransform.location = vehicleTransform.location;
		spectatorTransform.location += vehicleTransform.GetRightVector() * std::max(2.f, vehicleBBox.extent.y * 4.f);
		// points down
		spectatorTransform.location -= vehicleTransform.GetUpVector() * 1.72f;
		spectatorTransform.rotation = vehicleTransform.rotation;
		spectatorTransform.rotation.yaw -= 90;
		spectatorTransform.rotation.pitch -= 22.5f;
		spectator->SetTransform(spectatorTransform);
		auto debug = world->MakeDebugHelper();
		carla::client::DebugHelper::Color red(255U, 0, 0);
		carla::client::DebugHelper::Color yellow(255U, 255U, 0);
		carla::client::DebugHelper::Color red_2(255U / 2U, 0, 0);
		carla::client::DebugHelper::Color yellow_2(255U / 2U, 255U / 2U, 0);
		carla::client::DebugHelper::Color orange(255U, 80U, 0);
		carla::client::DebugHelper::Color lila(255U, 0, 255U);
		carla::client::DebugHelper::Color blue(0, 0, 255U);
		carla::client::DebugHelper::Color turkis(0, 255U, 255U);
		carla::client::DebugHelper::Color green(0, 255U, 0);
		auto vehicleBBoxWorld(vehicleBBox);
		CHECK(vehicleBBoxWorld.location == vehicleBBox.location);
		vehicleBBoxWorld.location += vehicleTransform.location;
		CHECK(vehicleBBoxWorld.location != vehicleBBox.location);
		// draw bounding box at spawn location
		debug.DrawBox(vehicleBBoxWorld, vehicleTransform.rotation, 0.1f, red, 6);
		// wait for vehicle to settle after spawn
		time = world->WaitForTick(transactionTimeout).GetTimestamp();
		double timestamp = time.elapsed_seconds;
		std::cout << "Time at box 1 " << time.elapsed_seconds << std::endl;
		while (timestamp + 3 > time.elapsed_seconds) {
			time = world->WaitForTick(transactionTimeout).GetTimestamp();
		}
		std::cout << "Time at box 2 " << time.elapsed_seconds << std::endl;
		vehicleTransform = vehicle->GetTransform();
		auto vehicleLocation = vehicle->GetLocation();
		CHECK(vehicleLocation == vehicleTransform.location);
		CHECK(Approx(vehicleLocation.SquaredLength()) == vehicleTransform.location.SquaredLength());
		vehicleBBoxWorld.location = vehicleBBox.location + vehicleTransform.location;
		//draw bounding box at settled location
		debug.DrawBox(vehicleBBoxWorld, vehicleTransform.rotation, 0.1f, red, 60);
		// transform from bbox to vehicle coordinates
		auto frontAxleLocation = bbcenter_to_front - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		auto rearAxleLocation = bbcenter_to_rear - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		// transform from vehicle to world coordinates
		vehicleTransform.TransformPoint(frontAxleLocation);
		vehicleTransform.TransformPoint(rearAxleLocation);
		carla::geom::Vector3D extent = vehicleTransform.GetRightVector() * vehicleBBox.extent.y * 1.2f;
		debug.DrawPoint(frontAxleLocation, 0.2f, red, 300);
		debug.DrawPoint(rearAxleLocation, 0.2f, yellow, 300);
		debug.DrawPoint(vehicleBBoxWorld.location, 0.4f, green, 300);
		debug.DrawPoint(vehicleTransform.location, 0.2f, blue, 300);
		debug.DrawPoint(vehicleLocation, 0.2f, turkis, 300);
		debug.DrawLine(frontAxleLocation - extent, frontAxleLocation + extent, .05f, red, 300);
		debug.DrawLine(rearAxleLocation - extent, rearAxleLocation + extent, .05f, yellow, 300);
		debug.DrawArrow(vehicleBBoxWorld.location, frontAxleLocation, 0.02f, 0.1f, blue, 300);
		debug.DrawArrow(vehicleBBoxWorld.location, rearAxleLocation, 0.02f, 0.1f, turkis, 300);

		// Get current axle positions in vehicle coordinates
		/*auto&[currentFrontAxleOffset, currentRearAxleOffset] = world->GetAxlePositions(actor->GetId());

		CHECK(Approx(currentFrontAxleOffset.x).margin(0.001f) == bbcenter_to_front.x);
		CHECK(Approx(currentFrontAxleOffset.y).margin(0.001f) == bbcenter_to_front.y);
		CHECK(Approx(currentFrontAxleOffset.z).margin(0.001f) == bbcenter_to_front.z);
		CHECK(Approx(currentRearAxleOffset.x).margin(0.001f) == bbcenter_to_rear.x);
		CHECK(Approx(currentRearAxleOffset.y).margin(0.001f) == bbcenter_to_rear.y);
		CHECK(Approx(currentRearAxleOffset.z).margin(0.001f) == bbcenter_to_rear.z);

		// Transform axle offsets from bbox coordinates to vehicle coordinates
		auto currentFrontAxleLocation = currentFrontAxleOffset - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		auto currentRearAxleLocation = currentRearAxleOffset - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		// Transform positions from vehicle coordinates to world coordinates
		vehicleTransform.TransformPoint(currentFrontAxleLocation);
		vehicleTransform.TransformPoint(currentRearAxleLocation);
		// Compare current world axle positions with attribute values
		CHECK(Approx(currentFrontAxleLocation.x).margin(0.001f) == frontAxleLocation.x);
		CHECK(Approx(currentFrontAxleLocation.y).margin(0.001f) == frontAxleLocation.y);
		CHECK(Approx(currentFrontAxleLocation.z).margin(0.001f) == frontAxleLocation.z);
		CHECK(Approx(currentRearAxleLocation.x).margin(0.001f) == rearAxleLocation.x);
		CHECK(Approx(currentRearAxleLocation.y).margin(0.001f) == rearAxleLocation.y);
		CHECK(Approx(currentRearAxleLocation.z).margin(0.001f) == rearAxleLocation.z);
		debug.DrawLine(currentFrontAxleLocation - extent, currentFrontAxleLocation + extent, .05f, yellow_2, 300);
		debug.DrawLine(currentRearAxleLocation - extent, currentRearAxleLocation + extent, .05f, red_2, 300);*/

		while (timestamp + 9 > time.elapsed_seconds) {
			time = world->WaitForTick(transactionTimeout).GetTimestamp();
		}
	}
}

TEST_CASE("bbcenter_to_X raw attribute 2", "[DEBUG][.][TestsCarlaOsiServer][DontDraw]") {
	//NOT a test of a utility function, but of additional data needed from the carla server
	//TODO refactor this to improve run time - spawn all vehicles at once and test attributes after they have settled, waiting only once
	auto client = std::make_unique<carla::client::Client>("localhost", 2000u);
	auto transactionTimeout = std::chrono::duration<double>(60);
	client->SetTimeout(transactionTimeout);
	auto world = std::make_unique<carla::client::World>(std::move(client->ReloadWorld()));

	// retrieve vehicle blueprints and spawn them
	auto blueprintLibrary = world->GetBlueprintLibrary();
	auto vehicleBlueprints = blueprintLibrary->Filter("vehicle.*");
	auto recommendedSpawnPoints = world->GetMap()->GetRecommendedSpawnPoints();
	for (size_t i = 0; i < vehicleBlueprints->size() && i < recommendedSpawnPoints.size(); i++) {
		auto vehicleBlueprint = vehicleBlueprints->at(i);
		auto actor = world->SpawnActor(vehicleBlueprint, recommendedSpawnPoints.at(i));
		auto vehicle = boost::static_pointer_cast<carla::client::Vehicle>(actor);
		CHECK(0 < actor->GetId());

		carla::geom::Vector3D bbcenter_to_front;
		carla::geom::Vector3D bbcenter_to_rear;
		float wheel_radius = -1;
		auto attributes = actor->GetAttributes();
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_x"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_y"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_front_z"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_x"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_y"; }));
		CHECK(std::any_of(attributes.begin(), attributes.end(),
			[](carla::client::ActorAttributeValue& attribute) {return attribute.GetId() == "bbcenter_to_rear_z"; }));
		// build bbcenter_to_front/back vectors and query wheel radius
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
			else if ("wheel_radius" == id) {
				wheel_radius = attribute.As<float>();
			}
		}
		CHECK(0 <= wheel_radius);

		// draw line for axis using debug helpers
		auto time = world->WaitForTick(transactionTimeout).GetTimestamp();
		carla::geom::Transform vehicleTransform = vehicle->GetTransform();
		carla::geom::BoundingBox vehicleBBox = vehicle->GetBoundingBox();

		// 
		auto spectator = world->GetSpectator();
		auto spectatorTransform = spectator->GetTransform();
		spectatorTransform.location = vehicleTransform.location;
		spectatorTransform.location += vehicleTransform.GetRightVector() * vehicleBBox.extent.y * 4;
		// points down
		spectatorTransform.location -= vehicleTransform.GetUpVector() * 1.72f;
		spectatorTransform.rotation = vehicleTransform.rotation;
		spectatorTransform.rotation.yaw -= 90;
		spectatorTransform.rotation.pitch -= 22.5f;
		spectator->SetTransform(spectatorTransform);

		auto vehicleBBoxWorld(vehicleBBox);
		CHECK(vehicleBBoxWorld.location == vehicleBBox.location);
		vehicleBBoxWorld.location += vehicleTransform.location;
		CHECK(vehicleBBoxWorld.location != vehicleBBox.location);

		// wait for vehicle to settle after spawn
		time = world->WaitForTick(transactionTimeout).GetTimestamp();
		double timestamp = time.elapsed_seconds;
		while (timestamp + 3 > time.elapsed_seconds) {
			time = world->WaitForTick(transactionTimeout).GetTimestamp();
		}

		vehicleTransform = vehicle->GetTransform();
		auto vehicleLocation = vehicle->GetLocation();
		CHECK(vehicleLocation == vehicleTransform.location);
		CHECK(Approx(vehicleLocation.SquaredLength()) == vehicleTransform.location.SquaredLength());
		vehicleBBoxWorld.location = vehicleBBox.location + vehicleTransform.location;
		// transform from bbox to vehicle coordinates
		auto frontAxleLocation = bbcenter_to_front - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		auto rearAxleLocation = bbcenter_to_rear - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		// transform from vehicle to world coordinates
		vehicleTransform.TransformPoint(frontAxleLocation);
		vehicleTransform.TransformPoint(rearAxleLocation);

		// Get current axle positions in vehicle coordinates
		/*auto&[currentFrontAxleOffset, currentRearAxleOffset] = world->GetAxlePositions(actor->GetId());

		// Transform axle offsets from bbox coordinates to vehicle coordinates
		auto currentFrontAxleLocation = currentFrontAxleOffset - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		auto currentRearAxleLocation = currentRearAxleOffset - static_cast<carla::geom::Vector3D>(vehicleBBox.location);
		// Transform positions from vehicle coordinates to world coordinates
		vehicleTransform.TransformPoint(currentFrontAxleLocation);
		vehicleTransform.TransformPoint(currentRearAxleLocation);
		// Compare current world axle positions with attribute values
		CHECK(Approx(currentFrontAxleLocation.x).margin(0.001f) == frontAxleLocation.x);
		CHECK(Approx(currentFrontAxleLocation.y).margin(0.001f) == frontAxleLocation.y);
		CHECK(Approx(currentFrontAxleLocation.z).margin(0.001f) == frontAxleLocation.z);
		CHECK(Approx(currentRearAxleLocation.x).margin(0.001f) == rearAxleLocation.x);
		CHECK(Approx(currentRearAxleLocation.y).margin(0.001f) == rearAxleLocation.y);
		CHECK(Approx(currentRearAxleLocation.z).margin(0.001f) == rearAxleLocation.z);*/
	}
}