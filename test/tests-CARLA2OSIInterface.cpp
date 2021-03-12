#include "catch2/catch.hpp"

#include "testhelpers.h"

#include "CARLA2OSIInterface.h"
#include "Utility.h"
#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector2D.h>
#include <carla/geom/Vector3D.h>

TEST_CASE("CARLA2OSIInterface", "[CARLAInterface][.][RequiresCarlaServer]") {
	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();

	// carla server
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 25;
	// delta seconds (1/framerate)
	double deltaSeconds = (1.0 / 60);

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(host, port, transactionTimeout);

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

TEST_CASE("Parsing of added vehicle attributes for osi3::MovingObject", "[CARLAInterface][.][RequiresCarlaServer][MovingObject]") {

	// carla server
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 25;
	// delta seconds (1/framerate)
	double deltaSeconds = (1.0 / 60);

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(host, port, transactionTimeout);
	//world = client->LoadWorld("2020-05-04_atCity_AF_DLR_Braunschweig_Prio1_ROD_offset");
	//world.WaitForTick(std::chrono::seconds(45));

	//spawn vehicles
	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto vehicleBlueprints = blueprintLibrary->Filter("vehicle.*");
	auto recommendedSpawnPoints = world.GetMap()->GetRecommendedSpawnPoints();
	uint32_t spawns = 0, fails = 0;
	for (size_t i = 0; i < vehicleBlueprints->size() && i < recommendedSpawnPoints.size(); i++) {
		auto vehicleBlueprint = vehicleBlueprints->at(i);
		try {
			auto actor = world.SpawnActor(vehicleBlueprint, recommendedSpawnPoints.at(i));
			spawns++;
		}
		catch (std::exception e) {
			std::cout << "Spawn failed: " << e.what() << std::endl;
			fails++;
		}
	}
	world.WaitForTick(std::chrono::seconds(45));

	// compare ground truth to vehicles
	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	carla->initialise(host, port, transactionTimeout, deltaSeconds);
	auto groundTruth = carla->getLatestGroundTruth();
	CHECK(std::min(vehicleBlueprints->size(), recommendedSpawnPoints.size()) - fails == spawns);
	CHECK(std::min(vehicleBlueprints->size(), recommendedSpawnPoints.size()) == groundTruth->moving_object_size() + fails);
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
		carla_osi::id_mapping::IDUnion expectedOSIId{
			((uint64_t)carla_osi::id_mapping::CarlaUniqueID_e::ActorID) << (/*16ULL + 8ULL +*/ 32ULL) | actor->GetId() };
		CHECK(actor->GetId() == expectedOSIId.id);
		CHECK(carla_osi::id_mapping::CarlaUniqueID_e::ActorID == expectedOSIId.type);
		CHECK(0 == expectedOSIId.special);
		CHECK(0 == expectedOSIId.special2);
		REQUIRE(movingObject.has_base());
		auto base = movingObject.base();
		testBaseMoving(base, vehicle, bbox);
		REQUIRE(movingObject.has_vehicle_classification());
		auto classification = movingObject.vehicle_classification();
		REQUIRE(classification.has_type());
		REQUIRE(osi3::MovingObject_VehicleClassification_Type_TYPE_UNKNOWN != classification.type());
		REQUIRE(movingObject.has_vehicle_attributes());
		auto attributes = movingObject.vehicle_attributes();
		REQUIRE(carla_osi::geometry::toCarla(&attributes.bbcenter_to_front()) == bbcenter_to_front);
		REQUIRE(carla_osi::geometry::toCarla(&attributes.bbcenter_to_rear()) == bbcenter_to_rear);
		REQUIRE(attributes.has_number_wheels());
		REQUIRE(((4 == attributes.number_wheels()) || (2 == attributes.number_wheels())));
		REQUIRE(attributes.has_radius_wheel());
		REQUIRE(Approx(wheel_radius) == attributes.radius_wheel());
		switch (classification.type()) {
			// types with a defined default value
		case osi3::MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_COMPACT_CAR:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_BUS:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_DELIVERY_VAN:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_HEAVY_TRUCK:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_SEMITRAILER:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_TRAILER:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE:
		case osi3::MovingObject_VehicleClassification_Type_TYPE_BICYCLE:
			REQUIRE(0 <= attributes.ground_clearance());
			break;
			// everything else
		default:
			break;
		}

		auto&[frontAxle, rearAxle] = world.GetAxlePositions(actor->GetId());
		auto current_front_offset = /*static_cast<carla::geom::Vector3D>(bbox.location) -*/ frontAxle;
		auto current_rear_offset = /*static_cast<carla::geom::Vector3D>(bbox.location) -*/ rearAxle;
		CHECK(Approx(current_front_offset.x).margin(0.001f) == bbcenter_to_front.x);
		CHECK(Approx(current_front_offset.y).margin(0.001f) == bbcenter_to_front.y);
		CHECK(Approx(current_front_offset.z).margin(0.001f) == bbcenter_to_front.z);
		CHECK(Approx(current_rear_offset.x).margin(0.001f) == bbcenter_to_rear.x);
		CHECK(Approx(current_rear_offset.y).margin(0.001f) == bbcenter_to_rear.y);
		CHECK(Approx(current_rear_offset.z).margin(0.001f) == bbcenter_to_rear.z);
	}
}

TEST_CASE("Parse CARLA Walker into OSI MovinObject", "[CARLAInterface][.][RequiresCarlaServer][Pedestrian]") {
	// carla server
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 25;
	// delta seconds (1/framerate)
	double deltaSeconds = (1.0 / 60);

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(host, port, transactionTimeout);

	auto map = world.GetMap();

	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto walkerBp = blueprintLibrary->Filter("walker.pedestrian.*")->at(0);
	carla::geom::Transform transform{ world.GetRandomLocationFromNavigation().get_value_or(carla::geom::Location()) };
	auto walker = boost::static_pointer_cast<carla::client::Walker>(world.SpawnActor(walkerBp, transform));

	// get waypoint next to a suggested spawn location
	auto waypoint = map->GetWaypoint(transform.location,
		true, (uint32_t)carla::road::Lane::LaneType::Any);
	std::unique_ptr<osi3::Identifier>  expectedWaypointIdentity{ waypoint->IsJunction() ?
		carla_osi::id_mapping::toOSI(waypoint->GetJunctionId(), carla_osi::id_mapping::JuncID)
		: carla_osi::id_mapping::toOSI(waypoint->GetRoadId(), waypoint->GetLaneId(), waypoint->GetSectionId()) };

	// draw debug point to visualize the waypoints transform
	auto debug = world.MakeDebugHelper();
	debug.DrawArrow(waypoint->GetTransform().location, transform.location, 0.04f);

	auto spectator = world.GetSpectator();
	auto spectatorTransform = spectator->GetTransform();
	spectatorTransform.location = transform.location;
	spectatorTransform.location += transform.GetRightVector() * 4.f;
	// UpVector points down?
	spectatorTransform.location -= transform.GetUpVector() * 1.72f;
	spectatorTransform.rotation = transform.rotation;
	spectatorTransform.rotation.yaw -= 90;
	spectatorTransform.rotation.pitch -= 22.5f;
	spectator->SetTransform(spectatorTransform);

	auto& bbox = walker->GetBoundingBox();

	std::unique_ptr<osi3::Identifier> expectedIdentity{ carla_osi::id_mapping::toOSI(walker->GetId()) };

	std::cout << __FUNCTION__ << ": Distance from walker to waypoint: " << walker->GetLocation().Distance(waypoint->GetTransform().location) << std::endl;

	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	carla->initialise(host, port, transactionTimeout, deltaSeconds);

	auto groundTruth = carla->getLatestGroundTruth();
	// search for moving object of type PEDESTRIAN
	auto iter = std::find_if(groundTruth->moving_object().begin(), groundTruth->moving_object().end(), [](osi3::MovingObject m) {
		return osi3::MovingObject_Type_TYPE_PEDESTRIAN == m.type(); });

	if (iter != groundTruth->moving_object().end()) {
		auto& movingObject = *iter;
		REQUIRE(movingObject.id().value() == expectedIdentity->value());
		REQUIRE(movingObject.has_base());
		testBaseMoving(movingObject.base(), walker, bbox);
	}
}

TEST_CASE("Parse some camera sensor frames", "[CARLAInterface][.][RequiresCarlaServer][CameraSensor]") {
	std::string host = "localhost";
	uint16_t port = 2000u;
	double transactionTimeout = 25;
	// delta seconds (1/framerate)
	double deltaSeconds = (1.0 / 60);

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(host, port, transactionTimeout);

	auto map = world.GetMap();

	std::string role = "sensor.camera.rgb";
	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto sensorBp = blueprintLibrary->at("sensor.camera.rgb");
	sensorBp.SetAttribute("role_name", role);
	sensorBp.SetAttribute("image_size_x", "800");
	sensorBp.SetAttribute("image_size_y", "600");
	carla::geom::Transform sensorTransform({ -172.19658203f, 183.85912109f, 27.63805176f }, { 0, -45, 0 });
	auto sensor = world.SpawnActor(sensorBp, sensorTransform);

	std::shared_ptr<CARLA2OSIInterface> carla = std::make_shared<CARLA2OSIInterface>();
	carla->initialise(host, port, transactionTimeout, deltaSeconds);

	auto sensorView = carla->getSensorView(role);
	REQUIRE(sensorView->camera_sensor_view_size());
	auto cameraSensorView = sensorView->camera_sensor_view().at(0);
	REQUIRE(cameraSensorView.has_view_configuration());
	auto config = cameraSensorView.view_configuration();
	REQUIRE(config.has_sensor_id());
	carla_osi::id_mapping::IDUnion expectedId{ 0 };
	expectedId.id = sensor->GetId();
	expectedId.type = carla_osi::id_mapping::CarlaUniqueID_e::ActorID;
	CHECK(config.sensor_id().value() == expectedId.value);
	REQUIRE(config.sensor_id().value() == carla_osi::id_mapping::getOSIActorId(sensor)->value());
	REQUIRE(config.has_number_of_pixels_horizontal());
	REQUIRE(800 == config.number_of_pixels_horizontal
	());
	REQUIRE(config.has_number_of_pixels_vertical());
	REQUIRE(600 == config.number_of_pixels_vertical());
	REQUIRE(config.has_mounting_position());
	auto expectedMountingPosition = carla_osi::geometry::toOSI(sensorTransform);
	REQUIRE(Approx(config.mounting_position().position().x()) == expectedMountingPosition->position().x());
	REQUIRE(Approx(config.mounting_position().position().y()) == expectedMountingPosition->position().y());
	REQUIRE(Approx(config.mounting_position().position().z()) == expectedMountingPosition->position().z());
	REQUIRE(Approx(config.mounting_position().orientation().pitch()) == expectedMountingPosition->orientation().pitch());
	REQUIRE(Approx(config.mounting_position().orientation().yaw()) == expectedMountingPosition->orientation().yaw());
	REQUIRE(Approx(config.mounting_position().orientation().roll()) == expectedMountingPosition->orientation().roll());
	// Not defined in CARLA
	//REQUIRE(config.has_mounting_position_rmse());
	REQUIRE(config.channel_format_size());
	REQUIRE(osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_RGB_U8_LIN == config.channel_format()
		.Get(0));
	REQUIRE(cameraSensorView.has_image_data());
	auto& imageData = cameraSensorView.image_data();
	REQUIRE(imageData.size() == 800 * 600 * 3);


	boost::gil::rgb8c_pixel_t* pixels = reinterpret_cast<const boost::gil::rgb8_pixel_t*>(imageData.data());
	boost::gil::rgb8c_view_t view = boost::gil::interleaved_view((std::ptrdiff_t)config.number_of_pixels_horizontal(),
		(std::ptrdiff_t)config.number_of_pixels_vertical(), pixels,
		sizeof(uint8_t) * 3 * config.number_of_pixels_horizontal());
	boost::gil::write_view("CARLA_camera_sensor_image.png", view, boost::gil::png_tag());


}