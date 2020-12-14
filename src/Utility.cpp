#include "Utility.h"

using ID = CarlaUtility::CarlaUniqueID_t;

carla::geom::Vector3D CarlaUtility::mul(const carla::geom::Vector3D & vector, const float f)
{
	return carla::geom::Vector3D(vector.x * f, vector.y * f, vector.z * f);
}

osi3::Orientation3d* CarlaUtility::toOSI(const carla::geom::Rotation& rotation)
{
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	osi3::Orientation3d* orient = new osi3::Orientation3d();
	//TODO OSI prefers values in angular range [pi,pi]
	orient->set_yaw(-rotation.yaw * M_PI / 180.0);
	orient->set_pitch(rotation.pitch * M_PI / 180.0);
	orient->set_roll(rotation.roll * M_PI / 180.0);
	return orient;
}

std::pair<std::unique_ptr<osi3::Dimension3d>, std::unique_ptr<osi3::Vector3d>> CarlaUtility::toOSI(const carla::geom::BoundingBox& boundingBox) {
	std::unique_ptr<osi3::Dimension3d> dim = std::make_unique<osi3::Dimension3d>();
	// dimensions are unsigned
	dim->set_length(boundingBox.extent.x * 2);
	dim->set_width(boundingBox.extent.y * 2);
	dim->set_height(boundingBox.extent.z * 2);
	std::unique_ptr<osi3::Vector3d> vec;
	vec.reset(CarlaUtility::toOSI(boundingBox.location));
	return std::pair(std::move(dim), std::move(vec));
}

osi3::Vector3d* CarlaUtility::toOSI(const carla::geom::Vector3D& location) {
	//flip y
	osi3::Vector3d* vec = new osi3::Vector3d();
	vec->set_x(location.x);
	vec->set_y(-location.y);
	vec->set_z(location.z);
	return vec;
}

osi3::Vector2d* CarlaUtility::toOSI(const carla::geom::Vector2D& vector) {
	//flip y
	osi3::Vector2d* vec = new osi3::Vector2d();
	vec->set_x(vector.x);
	vec->set_y(vector.y);
	return vec;
}

carla::geom::Rotation CarlaUtility::toCarla(const osi3::Orientation3d* orientation) {
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	return carla::geom::Rotation(
		(float)(orientation->pitch() * 180 * M_1_PI),
		(float)(orientation->yaw() * -180 * M_1_PI),
		(float)(orientation->roll() * 180 * M_1_PI));
}

carla::geom::BoundingBox CarlaUtility::toCarla(const osi3::Dimension3d* dimension, const osi3::Vector3d* position) {
	carla::geom::Location pos = CarlaUtility::toCarla(position);
	carla::geom::Vector3D extent((float)(dimension->length() / 2.0), (float)(dimension->width() / 2.0), (float)(dimension->height() / 2.0));
	return carla::geom::BoundingBox(pos, extent);
}

carla::geom::Location CarlaUtility::toCarla(const osi3::Vector3d* position) {
	//flip y
	return carla::geom::Location((float)position->x(), (float)-position->y(), (float)position->z());
}

carla::geom::Vector2D CarlaUtility::toCarla(const osi3::Vector2d* position) {
	return carla::geom::Vector2D((float)position->x(), (float)position->y());
}

CarlaUtility::CarlaUniqueID_t CarlaUtility::toCarla(const osi3::Identifier* identifier) {
	//carlaID as lower 32 bits, CarlaUniqueID_t type index as upper 32 bits, as stored in osi3::Identifier * CarlaUtility::toOSI(CarlaUtility::CarlaUniqueID_t carlaID) {
	IDUnion idUnion{ identifier->value() };

	switch ((CarlaUniqueID_e)idUnion.type) {
	default:
	case ActorID:
		return (carla::ActorId) idUnion.id;
	case RoadIDLaneID:
		return std::make_tuple((carla::road::RoadId)idUnion.id, (carla::road::LaneId)idUnion.special, idUnion.special2);
	case JuncID:
		return (carla::road::JuncId) idUnion.id;
	}

}

osi3::Identifier * CarlaUtility::toOSI(const uint32_t id, CarlaUtility::CarlaUniqueID_e type)
{
	////id has to match index of type in variant CarlaUniqueID_t
	//return CarlaUtility::toOSI(CarlaUtility::getTyped(type, id));
	return toOSI(id, 0, type);
}

osi3::Identifier * CarlaUtility::toOSI(const uint32_t roadId, const int8_t laneId, CarlaUniqueID_e type) {
	return toOSI(roadId, laneId, 0u, type);
}

osi3::Identifier * CarlaUtility::toOSI(const uint32_t roadId, const int8_t laneId, const uint16_t sectionId, CarlaUniqueID_e type)
{
	CarlaUtility::IDUnion idUnion;
	idUnion.type = type;
	idUnion.special2 = sectionId;
	idUnion.special = (int16_t)laneId;
	idUnion.id = roadId;

	osi3::Identifier* identifier = new osi3::Identifier();
	identifier->set_value(idUnion.value);
	return identifier;
}

osi3::Identifier * CarlaUtility::toOSI(const carla::road::RoadId roadId, const carla::road::LaneId laneId, const uint16_t sectionId, const RoadIDType_e roadMarkType, CarlaUniqueID_e type)
{
	if (-3u < sectionId) {
		throw std::out_of_range("Section id is too large and cannot be differentiated from road mark id");
	}

	uint16_t sectionIdWithRoadMarkType = sectionId | ((uint16_t)roadMarkType) << 8u;
	return toOSI(roadId, laneId, sectionIdWithRoadMarkType, type);
}

osi3::StationaryObject* CarlaUtility::toOSI(const carla::SharedPtr< const carla::client::Actor> actor, carla::geom::BoundingBox& bbox)
{
	osi3::StationaryObject* prop = new osi3::StationaryObject();
	prop->set_allocated_id(CarlaUtility::toOSI(actor->GetId(), CarlaUniqueID_e::ActorID));

	osi3::BaseStationary* base = prop->mutable_base();
	// bounding boxes are only available for Junction, Vehicle and Walker, not for Actor as generalization (though there is a protected GetBoundingBox() member in ActorState)
	// also mentioned in https://github.com/carla-simulator/carla/issues/3186, https://github.com/carla-simulator/carla/issues/3025 and https://github.com/carla-simulator/carla/issues/1766
	// The attribute behind the protected field is misused for traffic signs and traffic lights and holds their active area instead
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	// The bounding box has to be given as argument. To circumvent the described limitation, World::GetActorBoundingBox(ActorId) is added
	auto[dimension, position] = CarlaUtility::toOSI(bbox);
	base->set_allocated_dimension(dimension.release());
	auto transform = actor->GetTransform();
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(CarlaUtility::toOSI(transform.rotation));

	//TODO How to get base_polygon from actor? (https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1BaseStationary.html#aa1db348acaac2d5a2ba0883903d962cd)

	//TODO Carla doesn't seem to offer information needed for osi3::StationaryObject::Classification. Using default instance
	auto classification = prop->mutable_classification();//creates default instance as side-effect
	classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_OTHER);
	//TODO fill with information from OpenDRIVE file, if available

	prop->set_model_reference(actor->GetTypeId());

	return prop;
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	// but bounding boxes only exist for Vehicle and Walker specializations
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(CarlaUtility::toOSI(transform.rotation));

	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Walker> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	//TODO verify relation of bouding box origin and actor.GetLocation() for Walkers
	auto bbox = actor->GetBoundingBox();
	// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in CarlaUtility::toOSI
	auto[dimension, location] = CarlaUtility::toOSI(actor->GetBoundingBox());
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations - assume bbox is not rotated
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(CarlaUtility::toOSI(transform.rotation));

	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	auto bbox = actor->GetBoundingBox();
	// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in CarlaUtility::toOSI
	auto[dimension, location] = CarlaUtility::toOSI(bbox);
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations, but
	// vehicle bounding boxes shouldn't be rotated
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(CarlaUtility::toOSI(transform.rotation));


	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving_common(const carla::SharedPtr<const carla::client::Actor> actor, std::unique_ptr<osi3::BaseMoving> base)
{
	auto transform = actor->GetTransform();

	//TODO determine contour on z-plane
	//auto contour = base->mutable_base_polygon();
	//contour->Add

	// velocity and acceleration as part of ground truth are given in global coordinate system
	//TODO reference frame of actor velocity is not documented might be local and has to be transformed
	base->set_allocated_velocity(CarlaUtility::toOSI(actor->GetVelocity()));
	base->set_allocated_acceleration(CarlaUtility::toOSI(actor->GetAcceleration()));
	auto angularVelocity = actor->GetAngularVelocity();//Carla uses Vector3d instead of Rotation as type
	base->set_allocated_orientation_rate(CarlaUtility::toOSI(carla::geom::Rotation(angularVelocity.y, angularVelocity.z, angularVelocity.x)));

	//TODO Carla has no rotational acceleration
	//base->set_allocated_orientation_acceleration

	return base;
}

osi3::TrafficSign* CarlaUtility::toOSI(const carla::SharedPtr<const carla::client::TrafficSign> actor/*, const pugi::xml_document& xodr*/)
{
	osi3::TrafficSign* sign = new osi3::TrafficSign();

	//TODO use OpenDRIVE for better traffic sign description. Also use it to differentiate between traffic signs as road marking and 'normal' traffic signs

	auto main = sign->mutable_main_sign();
	auto base = main->mutable_base();
	//TODO defined bounding box of traffic signs declare hit boxes where their restrictions should apply and don't identify the bounds of the sign
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	//base->set_allocated_dimension(dimension);
	auto transform = actor->GetTransform();
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	// OSI traffic signs point along x, while Carla traffic signs point along y => rotate yaw by 90�
	//TODO assure rotation is applied local
	auto rotation = carla::geom::Rotation(transform.rotation.pitch, 90 + transform.rotation.yaw, transform.rotation.roll);
	base->set_allocated_orientation(CarlaUtility::toOSI(rotation));
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

std::vector<osi3::TrafficLight*> CarlaUtility::toOSI(const carla::SharedPtr<const carla::client::TrafficLight> actor/*, const  pugi::xml_document& xodr*/)
{
	std::vector<osi3::TrafficLight*> osiTrafficLights;

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

		osi3::TrafficLight* trafficLightBulb = new osi3::TrafficLight();
		trafficLightBulb->set_allocated_id(CarlaUtility::toOSI(actor->GetId(), info.first, CarlaUniqueID_e::ActorID));

		auto base = trafficLightBulb->mutable_base();
		base->set_allocated_position(CarlaUtility::toOSI(bulbLocation));
		// OSI traffic lights point along x, while Carla traffic lights point along y => rotate yaw by 90�
		//TODO assure rotation is applied local
		auto rotation = carla::geom::Rotation(baseTransform.rotation.pitch, 90 + baseTransform.rotation.yaw, baseTransform.rotation.roll);
		base->set_allocated_orientation(CarlaUtility::toOSI(rotation));
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
		osiTrafficLights.push_back(trafficLightBulb);
	}
	return osiTrafficLights;
}

std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> CarlaUtility::toOSI(carla::client::Vehicle::LightState vehicleLights)
{
	auto lightState = std::make_unique<osi3::MovingObject_VehicleClassification_LightState>();

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Position) {
		// has no mapping
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::LowBeam) {
		lightState->set_head_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_head_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::HighBeam) {
		lightState->set_high_beam(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_high_beam(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Brake) {
		lightState->set_brake_light_state(osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL);
	}
	else {
		lightState->set_brake_light_state(osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights &
		((uint32_t)carla::client::Vehicle::LightState::RightBlinker & (uint32_t)carla::client::Vehicle::LightState::LeftBlinker)) {
		// Both indicator lights
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_WARNING);
	}
	else if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::RightBlinker) {
		// Only right indicator light
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_RIGHT);
	}
	else if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::LeftBlinker) {
		// Only left indicator light
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_LEFT);
	}
	else {
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Reverse) {
		lightState->set_reversing_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_reversing_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	// Setting both, front and rear fog lights because Carla does not differentiate
	//TODO do Carla vehicles have both front and rear fog lights or only front or rear?
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Fog) {
		lightState->set_front_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
		lightState->set_rear_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_front_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
		lightState->set_rear_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Interior) {
		// has no mapping
	}

	//TODO how to determine type of special illumination in Carla? OSI field can only be set if supported by the vehicle, which cannot be determined from Carla's light state
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Special1) {
		lightState->set_emergency_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE);
	}
	//else {
	//	lightState->set_emergency_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	//}
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Special2) {
		lightState->set_service_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER);
	}
	//else {
	//	lightState->set_service_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	//}

	return lightState;
}

osi3::CameraSensorView* CarlaUtility::toOSICamera(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	//Contains RGBA uint8 values
	auto image = boost::dynamic_pointer_cast<const carla::sensor::data::Image>(sensorData);
	if (!image) return nullptr;
	auto height = image->GetHeight();
	auto width = image->GetWidth();
	double aspect = ((double)width) / ((double)height);
	auto pixelCount = height * width;

	//Buffer for RGB OSI raw image
	auto rgb = new char[pixelCount * 3];

	//TODO use faster pixel format conversion or use osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_OTHER
	for (size_t i = 0; i < pixelCount; i++) {
		auto pixel = image->at(i);
		rgb[3 * i] = pixel.r;
		rgb[3 * i + 1] = pixel.g;
		rgb[3 * i + 2] = pixel.b;
	}

	osi3::CameraSensorView* cameraSensorView = new osi3::CameraSensorView();
	cameraSensorView->set_image_data(rgb);

	auto config = cameraSensorView->mutable_view_configuration();
	config->add_channel_format(osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_RGB_U8_LIN);
	// Carla only reports horizontal field of view
	auto fov = image->GetFOVAngle() * M_PI / 180.0;
	config->set_field_of_view_horizontal(fov);
	// guess vertical fov based on aspect ratio
	config->set_field_of_view_horizontal(fov / aspect);
	config->set_number_of_pixels_horizontal(width);
	config->set_number_of_pixels_vertical(height);
	config->set_allocated_sensor_id(CarlaUtility::toOSI(sensor->GetId(), CarlaUniqueID_e::ActorID));

	//TODO calculate sensor position in vehicle coordinates
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)


	return cameraSensorView;
}

osi3::LidarSensorView* CarlaUtility::toOSILidar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::LidarMeasurement>(sensorData);
	std::optional<double> rotationFrequency;
	std::optional<double> upperFov;
	std::optional<double> lowerFov;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("rotation_frequency" == attribute.GetId()) {
			rotationFrequency = std::stod(attribute.GetValue());
		}
		else if ("upper_fov" == attribute.GetId()) {
			upperFov = std::stod(attribute.GetValue());
		}
		else if ("lower_fov" == attribute.GetId()) {
			upperFov = std::stod(attribute.GetValue());
		}
	}
	std::optional<double> vFov;
	if (upperFov && lowerFov) {
		//upper and lower field of view are given in degree
		vFov = (upperFov.value() - lowerFov.value()) * M_PI / 180.0;
	}
	uint32_t numPixels;//OSI field uses uint32_t
	for (size_t i = 0; i < measurement->GetChannelCount(); i++) {
		numPixels += measurement->GetPointCount(i);
	}

	//TODO find translation from Carla point cloud to OSI Reflections. OSI uses signal strength, time of flight, doppler shift and normal to surface as measurements instead of simple hit point positions.
	//TODO Maybe use the osi3::FeatureData-based osi3::LidarDetection instead of a osi3::SensorView, which is similar to Carla's Lidar output

	osi3::LidarSensorView* lidarSensorView = new osi3::LidarSensorView();

	auto config = lidarSensorView->mutable_view_configuration();
	//TODO get lidar directions
	//config->add_directions()
	if (rotationFrequency) {
		config->set_emitter_frequency(rotationFrequency.value());
	}
	if (vFov) {
		config->set_field_of_view_vertical(vFov.value());
	}
	config->set_field_of_view_horizontal(M_PI * 2);
	config->set_max_number_of_interactions(1);

	//TODO calculate sensor position in vehicle coordinates, that is relative to the vehicles rear
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)

	//TODO OSI expects a constant number of pixels per message, but Carla only reports new values of the angle sweeped during the last frame
	config->set_num_of_pixels(numPixels);
	//TODO number of rays (horizontal/vertical) of lidar
	config->set_allocated_sensor_id(CarlaUtility::toOSI(sensor->GetId(), CarlaUniqueID_e::ActorID));

	return lidarSensorView;
}

osi3::RadarSensorView* CarlaUtility::toOSIRadar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::RadarMeasurement>(sensorData);
	std::optional<double> hFov;
	std::optional<double> vFov;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("horizontal_fov" == attribute.GetId()) {
			hFov = std::stod(attribute.GetValue());
		}
		else if ("vertical_fov" == attribute.GetId()) {
			vFov = std::stod(attribute.GetValue());
		}
	}

	auto radarSensorview = new osi3::RadarSensorView();

	//TODO find translation from Carla point cloud to OSI Reflections. OSI uses signal strength, time of flight, doppler shift and source vertical and horizontal angle as measurements instead of simple hit point positions.
	//TODO Maybe use the osi3::FeatureData-based osi3::RadarDetection instead of a osi3::SensorView, which is similar to Carla's Radar output

	auto config = radarSensorview->mutable_view_configuration();
	config->set_allocated_sensor_id(CarlaUtility::toOSI(sensor->GetId(), CarlaUniqueID_e::ActorID));
	if (hFov) {
		config->set_field_of_view_horizontal(hFov.value());
	}
	if (vFov) {
		config->set_field_of_view_vertical(vFov.value());
	}
	config->set_max_number_of_interactions(1);
	//TODO number of rays (horizontal/vertical)
	//TODO rx and tx antenna diagrams
	//TODO calculate sensor position in vehicle coordinates, that is relative to the vehicles rear
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)

	return nullptr;
}

carla::SharedPtr<carla::client::Vehicle> CarlaUtility::getParentVehicle(const carla::SharedPtr<const carla::client::Actor> actor)
{
	auto current = actor->GetParent();
	while (current && 0 != current->GetTypeId().rfind("vehicle", 0)) {
		current = current->GetParent();
	}
	auto vehicle = boost::dynamic_pointer_cast<carla::client::Vehicle>(current);
	return vehicle;
}

carla::rpc::VehicleLightState::LightState CarlaUtility::toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState) {
	//aggregate all received light states
	std::set<carla::rpc::VehicleLightState::LightState> receivedStates;

	if (indicatorState->has_indicator_state()) {
		switch (indicatorState->indicator_state()) {
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_LEFT:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LeftBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_RIGHT:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::RightBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_WARNING:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LeftBlinker);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::RightBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OTHER:
			break;
		default:
			break;
		}
	}
	//same effect as rear_fog_light
	if (indicatorState->has_front_fog_light()) {
		switch (indicatorState->front_fog_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Fog);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as front_fog_light
	if (indicatorState->has_rear_fog_light()) {
		switch (indicatorState->rear_fog_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Fog);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_head_light()) {
		switch (indicatorState->head_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LowBeam);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_high_beam()) {
		switch (indicatorState->high_beam()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::HighBeam);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_reversing_light()) {
		switch (indicatorState->reversing_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Reverse);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_brake_light_state()) {
		switch (indicatorState->brake_light_state()) {
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_STRONG:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Brake);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OTHER:
			break;
		default:
			break;
		}
	}
	//not part of carla
	if (indicatorState->has_license_plate_illumination_rear()) {
		switch (indicatorState->license_plate_illumination_rear()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as service_vehicle_illumination
	//(Special1 and Special2: This is reserved for certain vehicles that can have special lights, like a siren.)
	if (indicatorState->has_emergency_vehicle_illumination()) {
		switch (indicatorState->emergency_vehicle_illumination()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special1);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special2);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as emergency_vehicle_illumination
	//(Special1 and Special2: This is reserved for certain vehicles that can have special lights, like a siren.)
	if (indicatorState->has_service_vehicle_illumination()) {
		switch (indicatorState->service_vehicle_illumination()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special1);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special2);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}

	//aggregate all received light states into one state
	carla::rpc::VehicleLightState::LightState state = carla::rpc::VehicleLightState::LightState::None;
	for (carla::rpc::VehicleLightState::LightState receivedState : receivedStates) {
		state = carla::rpc::VehicleLightState::LightState(
			(carla::rpc::VehicleLightState::flag_type)state | (carla::rpc::VehicleLightState::flag_type)receivedState);
	}
	return state;
}

std::pair<std::unique_ptr<osi3::LaneBoundary::Classification>, std::unique_ptr<osi3::LaneBoundary::Classification>>
CarlaUtility::parseLaneBoundary(const carla::road::element::LaneMarking& laneMarking) {

	auto laneBoundaryClassification = std::make_unique<osi3::LaneBoundary::Classification>();
	// holds second classification in case of double lines. Is null otherwise
	std::unique_ptr<osi3::LaneBoundary::Classification> laneBoundaryClassification2;
	switch (laneMarking.color) {
	case carla::road::element::LaneMarking::Color::White:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_WHITE);
		break;
	case carla::road::element::LaneMarking::Color::Yellow:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_YELLOW);
		break;
	case carla::road::element::LaneMarking::Color::Blue:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_BLUE);
		break;
	case carla::road::element::LaneMarking::Color::Green:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_GREEN);
		break;
	case carla::road::element::LaneMarking::Color::Red:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_RED);
		break;
	case carla::road::element::LaneMarking::Color::Other:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_OTHER);
		break;
	}
	switch (laneMarking.type) {
	case carla::road::element::LaneMarking::Type::BottsDots:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_BOTTS_DOTS);
	case carla::road::element::LaneMarking::Type::BrokenBroken:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		break;
	case carla::road::element::LaneMarking::Type::BrokenSolid:
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		laneBoundaryClassification2->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Broken:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Curb:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_CURB);
		break;
	case carla::road::element::LaneMarking::Type::Grass:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_GRASS_EDGE);
		break;
	case carla::road::element::LaneMarking::Type::Other:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_OTHER);
		break;
	case carla::road::element::LaneMarking::Type::SolidSolid:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		break;
	case carla::road::element::LaneMarking::Type::SolidBroken:
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		laneBoundaryClassification2->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Solid:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		break;
	case carla::road::element::LaneMarking::Type::None:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN);
		break;
	}

	return { std::move(laneBoundaryClassification), std::move(laneBoundaryClassification2) };
}

std::tuple<google::protobuf::RepeatedPtrField<osi3::LaneBoundary>, uint64_t, uint64_t> CarlaUtility::parseLaneBoundary(carla::client::Map::TopologyList::value_type laneSection) {
	auto&[begin, end] = laneSection;
	// From OSI documentationon of osi3::LaneBoundary::Classification::Type:
	// There is no special representation for double lines, e.g. solid / solid or dashed / solid. In such 
	// cases, each lane will define its own side of the lane boundary.

	// => Always parse the outer lane boundary and only add the inner boundary if it consists of double lines to prevent duplicate entries. 
	//    Center lanes (laneId == 0) only have outer boundaries
	bool leftLaneIsOuter = 0 >= begin->GetLaneId();
	bool rightLaneIsOuter = 0 <= begin->GetLaneId();
	google::protobuf::RepeatedPtrField<osi3::LaneBoundary> laneBoundaries;
	uint64_t left_lane_boundary_id, right_lane_boundary_id;

	auto leftLaneMarking = begin->GetLeftLaneMarking();
	if (leftLaneMarking) {
		auto leftClassifications = parseLaneBoundary(leftLaneMarking.value());
		if (leftLaneIsOuter) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(leftClassifications.first.release());
			laneBoundary->set_allocated_id(
				CarlaUtility::toOSI(begin->GetRoadId(), begin->GetLaneId(), begin->GetSectionId(),
					CarlaUtility::RoadIDType_e::OuterBoundaryLine));
			left_lane_boundary_id = laneBoundary->id().value();
		}
		else if (leftClassifications.second) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(leftClassifications.second.release());
			laneBoundary->set_allocated_id(
				CarlaUtility::toOSI(begin->GetRoadId(), begin->GetLaneId(), begin->GetSectionId(),
					CarlaUtility::RoadIDType_e::InnerBoundaryLine));
			left_lane_boundary_id = laneBoundary->id().value();
		}
	}

	auto rightLaneMarking = begin->GetRightLaneMarking();
	if (rightLaneMarking) {
		auto rightClassifications = parseLaneBoundary(rightLaneMarking.value());
		if (rightLaneIsOuter) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(rightClassifications.first.release());
			laneBoundary->set_allocated_id(
				CarlaUtility::toOSI(begin->GetRoadId(), begin->GetLaneId(), begin->GetSectionId(),
					CarlaUtility::RoadIDType_e::OuterBoundaryLine));
			right_lane_boundary_id = laneBoundary->id().value();
		}
		else if (rightClassifications.second) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(rightClassifications.second.release());
			laneBoundary->set_allocated_id(
				CarlaUtility::toOSI(begin->GetRoadId(), begin->GetLaneId(), begin->GetSectionId(),
					CarlaUtility::RoadIDType_e::InnerBoundaryLine));
			right_lane_boundary_id = laneBoundary->id().value();
		}
	}

	//TODO Add BoundaryPoints - maybe parse from OpenDRIVE file?

	return { laneBoundaries, left_lane_boundary_id, right_lane_boundary_id };
}

osi3::MovingObject_VehicleClassification_Type CarlaUtility::ParseVehicleType(const std::string & value)
{
	if (std::string::npos != value.find("OTHER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_OTHER;
	}
	if (std::string::npos != value.find("SMALL_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR;
	}
	if (std::string::npos != value.find("COMPACT_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_COMPACT_CAR;
	}
	if (std::string::npos != value.find("MEDIUM_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR;
	}
	if (std::string::npos != value.find("LUXURY_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR;
	}
	if (std::string::npos != value.find("DELIVERY_VAN")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_DELIVERY_VAN;
	}
	if (std::string::npos != value.find("HEAVY_TRUCK")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_HEAVY_TRUCK;
	}
	if (std::string::npos != value.find("SEMITRAILER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_SEMITRAILER;
	}
	if (std::string::npos != value.find("TRAILER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAILER;
	}
	if (std::string::npos != value.find("MOTORBIKE")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE;
	}
	if (std::string::npos != value.find("BICYCLE")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_BICYCLE;
	}
	if (std::string::npos != value.find("BUS")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_BUS;
	}
	if (std::string::npos != value.find("TRAM")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAM;
	}
	if (std::string::npos != value.find("TRAIN")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAIN;
	}
	if (std::string::npos != value.find("WHEELCHAIR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_WHEELCHAIR;
	}
	return osi3::MovingObject_VehicleClassification_Type_TYPE_UNKNOWN;
}

