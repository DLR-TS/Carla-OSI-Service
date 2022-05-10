#include "carla_osi/TrafficSignals.h"

// enable math definitons (M_PI) using MSVC
#define _USE_MATH_DEFINES
#include <math.h>

#include "carla_osi/Identifiers.h"
#include "carla_osi/Geometry.h"


std::unique_ptr<osi3::TrafficSign> carla_osi::traffic_signals::getOSITrafficSign(
	const carla::SharedPtr<const carla::client::TrafficSign> actor,
	const carla::geom::BoundingBox& bbox/*, const pugi::xml_document& xodr*/)
{

	std::cout << "parsing a traffic sign" << std::endl;

	std::unique_ptr<osi3::TrafficSign> sign = std::make_unique<osi3::TrafficSign>();
	sign->set_allocated_id(carla_osi::id_mapping::getOSIActorId(actor).release());

	//TODO use OpenDRIVE for better traffic sign description. Also use it to differentiate between traffic signs as road marking and 'normal' traffic signs

	auto main = sign->mutable_main_sign();
	auto base = main->mutable_base();
	// bounding boxes of traffic signs declare hit boxes where their restrictions should apply and don't identify the bounds of the sign
	// --> use world::GetActorBoundingBox and pass as argument
	auto[dimension, position] = carla_osi::geometry::toOSI(bbox);
	base->set_allocated_dimension(dimension.release());
	auto transform = actor->GetTransform();
	base->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	// OSI traffic signs point along x, while Carla traffic signs point along y => rotate yaw by 90°
	//TODO assure rotation is applied local
	auto rotation = carla::geom::Rotation(transform.rotation.pitch, 90 + transform.rotation.yaw, transform.rotation.roll);
	base->set_allocated_orientation(carla_osi::geometry::toOSI(rotation).release());
	//TODO How to get base_polygon from actor? (https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1BaseStationary.html#aa1db348acaac2d5a2ba0883903d962cd)

	auto classification = main->mutable_classification();
	//TODO find LaneID for traffic sign
	//classification->add_assigned_lane_id

	classification->set_direction_scope(osi3::TrafficSign::MainSign::Classification::DirectionScope::TrafficSign_MainSign_Classification_DirectionScope_DIRECTION_SCOPE_NO_DIRECTION);
	// Carla doesn't differentiate the variability of traffic signs
	classification->set_variability(osi3::TrafficSign_Variability::TrafficSign_Variability_VARIABILITY_FIXED);

	// Traffic sign IDs as defined in <carla 0.9.9>/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEpisode.cpp
	if (actor->GetTypeId() == "traffic.traffic_light") {
		std::cerr << "Traffic lights should not be parsed as traffic signs" << std::endl;
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.30") {
		//TODO also set unit
		classification->mutable_value()->set_value(30);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.40") {
		//TODO also set unit
		classification->mutable_value()->set_value(40);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.50") {
		//TODO also set unit
		classification->mutable_value()->set_value(50);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.60") {
		//TODO also set unit
		classification->mutable_value()->set_value(60);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.90") {
		//TODO also set unit
		classification->mutable_value()->set_value(90);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.100") {
		//TODO also set unit
		classification->mutable_value()->set_value(100);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.120") {
		//TODO also set unit
		classification->mutable_value()->set_value(120);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.speed_limit.130") {
		//TODO also set unit
		classification->mutable_value()->set_value(130);
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_SPEED_LIMIT_BEGIN);
	}
	else if (actor->GetTypeId() == "traffic.stop") {
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_STOP);
	}
	else if (actor->GetTypeId() == "traffic.yield") {
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_GIVE_WAY);
	}
	else if (actor->GetTypeId() == "traffic.unknown") {
		//Unknown as part of OSI ground truth is forbidden
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_OTHER);
	}
	else {
		std::cerr << __FUNCTION__ << ": Encountered traffic sign with unknown mapping (" << actor->GetTypeId() << ")" << std::endl;
	}


	return sign;
}

std::vector<std::unique_ptr<osi3::TrafficLight>> carla_osi::traffic_signals::getOSITrafficLight(
	const carla::SharedPtr<const carla::client::TrafficLight> actor/*,/*, const  pugi::xml_document& xodr*/)
{

	std::cout << "parsing a traffic light with the old function" << std::endl;

	std::vector<std::unique_ptr<osi3::TrafficLight>> osiTrafficLights;

	//OSI defines a traffic light as an actual bulb. Therefore, red, yellow and green are three separate traffic lights
	auto baseTransform = actor->GetTransform();
	float yawDegree = baseTransform.rotation.yaw;
	double yaw = yawDegree * M_PI / 180;

	//Values extracted from Carla model OpenDrive Traffic Light
	std::map<int, carla::geom::Location> bulbInfos;
	//Difference between lightulbs is about 35 cm
	carla::geom::Location greenLightLocationDiff;
	greenLightLocationDiff.x = -5.99f;
	greenLightLocationDiff.y = 0.50f;
	greenLightLocationDiff.z = 5.22f;
	carla::geom::Location yellowLightLocationDiff;
	yellowLightLocationDiff.x = -5.99f;
	yellowLightLocationDiff.y = 0.50f;
	yellowLightLocationDiff.z = 5.57f;
	carla::geom::Location redLightLocationDiff;
	redLightLocationDiff.x = -5.99f;
	redLightLocationDiff.y = 0.50f;
	redLightLocationDiff.z = 5.92f;

	bulbInfos.insert({ 0, greenLightLocationDiff });
	bulbInfos.insert({ 1, yellowLightLocationDiff });
	bulbInfos.insert({ 2, redLightLocationDiff });

	// create three traffic lights using information available in Carla
	for (auto info : bulbInfos)
	{
		//apply yaw to location vector
		float x = info.second.x * std::cos(yaw) - info.second.y * std::sin(yaw);
		float y = info.second.x * std::sin(yaw) + info.second.y * std::cos(yaw);
		//combine base vector and added vector from base to lightbulb
		carla::geom::Location bulbLocation;
		bulbLocation.x = x + baseTransform.location.x;
		bulbLocation.y = y + baseTransform.location.y;
		bulbLocation.z = info.second.z + baseTransform.location.z;

		std::unique_ptr<osi3::TrafficLight> trafficLightBulb = std::make_unique<osi3::TrafficLight>();
		trafficLightBulb->set_allocated_id(carla_osi::id_mapping::getOSITrafficLightId(actor, info.first).release());

		auto base = trafficLightBulb->mutable_base();
		base->set_allocated_position(carla_osi::geometry::toOSI(bulbLocation).release());
		// OSI traffic lights point along x, while Carla traffic lights point along y => rotate yaw by 90°
		//TODO assure rotation is applied local
		auto rotation = carla::geom::Rotation(baseTransform.rotation.pitch, 90 + baseTransform.rotation.yaw, baseTransform.rotation.roll);
		base->set_allocated_orientation(carla_osi::geometry::toOSI(rotation).release());
		osi3::Dimension3d* dimension = new osi3::Dimension3d();
		//bulbs have circa 30 centimeter diameter
		dimension->set_height(0.30f);
		dimension->set_length(0.30f);
		dimension->set_width(0.30f);
		base->set_allocated_dimension(dimension);

		auto classification = trafficLightBulb->mutable_classification();
		switch (info.first) {
		case 0:
			classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_GREEN);
			classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
			break;
		case 1:
			classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_YELLOW);
			classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
			break;
		case 2:
			classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_RED);
			classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
			break;
		default:
			classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_OTHER);
			classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_OFF);
		}
		classification->set_icon(osi3::TrafficLight_Classification_Icon_ICON_NONE);
		osiTrafficLights.push_back(std::move(trafficLightBulb));
	}
	return osiTrafficLights;
}

/*std::vector<std::unique_ptr<osi3::TrafficLight>> carla_osi::traffic_signals::getOSITrafficLight(
	const carla::SharedPtr<const carla::client::TrafficLight> actor,
	const std::vector<carla::rpc::TrafficLightHeads> heads) {

	//fallback - use old behaviour
	if (!heads.size()) {
		return carla_osi::traffic_signals::getOSITrafficLight(actor);
	}

	std::cout << "parsing a traffic light with the new function" << std::endl;

	std::vector<std::unique_ptr<osi3::TrafficLight>> osiTrafficLights;
	int counter = 0;
	auto origin = actor->GetTransform();

	for (const auto& head : heads) {
		for (const auto& light : head.lights) {
			auto bulb = std::make_unique<osi3::TrafficLight>();

			//set id
			bulb->set_allocated_id(carla_osi::id_mapping::getOSITrafficLightId(actor, counter++).release());

			//fill base stationary
			auto base = bulb->mutable_base();
			//carla::geom::BoundingBox bbox(origin.TransformPoint(light.bbox.location),light.bbox.extent, light.bbox.rotation * origin.rotation);
			auto[dimension, position] = carla_osi::geometry::toOSI(light.bbox);
			base->set_allocated_position(position.release());
			base->set_allocated_dimension(dimension.release());
			base->set_allocated_orientation(carla_osi::geometry::toOSI(light.bbox.rotation).release());

			//fill classification
			auto classification = bulb->mutable_classification();
			//TODO flashing is not described by CARLA's state
			carla::rpc::TrafficLightState state = actor->GetState();
			switch (light.color)
			{
			case carla::rpc::TrafficLightColor::Red:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_RED);
				if (carla::rpc::TrafficLightState::Red == state)
					classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
				break;
			case carla::rpc::TrafficLightColor::Yellow:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_YELLOW);
				if (carla::rpc::TrafficLightState::Yellow == state)
					classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
				break;
			case carla::rpc::TrafficLightColor::Green:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_GREEN);
				if (carla::rpc::TrafficLightState::Green == state)
					classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
				break;
			case carla::rpc::TrafficLightColor::Blue:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_BLUE);
				classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_FLASHING);
				break;
			case carla::rpc::TrafficLightColor::White:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_WHITE);
				classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
				break;
			default:
			case carla::rpc::TrafficLightColor::Other:
				classification->set_color(osi3::TrafficLight_Classification_Color_COLOR_OTHER);
				classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_CONSTANT);
				break;
			}
			if (carla::rpc::TrafficLightState::Off == state) {
				classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_OFF);
			}
			else if (carla::rpc::TrafficLightState::Unknown == state) {
				classification->set_mode(osi3::TrafficLight_Classification_Mode_MODE_OTHER);
			}
			//TODO CARLA traffic lights have no knowledge about displayed icons (only defined as texture)
			classification->set_icon(osi3::TrafficLight_Classification_Icon_ICON_NONE);

			//TODO fill missing attributes (assigned_lane_id, is_out_of_service)

			osiTrafficLights.push_back(std::move(bulb));
		}
	}

	return osiTrafficLights;
}*/
