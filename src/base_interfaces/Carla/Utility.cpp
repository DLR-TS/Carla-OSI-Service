#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "base_interfaces/Carla/Utility.h"

osi3::Orientation3d* CarlaUtility::toOSI(carla::geom::Rotation& rotation)
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

std::pair<osi3::Dimension3d*, osi3::Vector3d*> CarlaUtility::toOSI(carla::geom::BoundingBox& boundingBox) {
	osi3::Dimension3d* dim = new osi3::Dimension3d();
	// dimensions are unsigned
	dim->set_length(boundingBox.extent.x * 2);
	dim->set_width(boundingBox.extent.y * 2);
	dim->set_height(boundingBox.extent.z * 2);
	osi3::Vector3d* vec = CarlaUtility::toOSI(boundingBox.location);
	return std::pair(dim, vec);
}

osi3::Vector3d* CarlaUtility::toOSI(carla::geom::Vector3D& location) {
	//flip y
	osi3::Vector3d* vec = new osi3::Vector3d();
	vec->set_x(location.x);
	vec->set_y(-location.y);
	vec->set_z(location.z);
	return vec;
}

osi3::Vector2d* CarlaUtility::toOSI(carla::geom::Vector2D& vector) {
	//flip y
	osi3::Vector2d* vec = new osi3::Vector2d();
	vec->set_x(vector.x);
	vec->set_y(vector.y);
	return vec;
}

carla::geom::Rotation CarlaUtility::toCarla(osi3::Orientation3d* orientation) {
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	return carla::geom::Rotation(
		(float)(orientation->pitch() * 180 * M_1_PI),
		(float)(orientation->yaw() * -180 * M_1_PI),
		(float)(orientation->roll() * 180 * M_1_PI));
}

carla::geom::BoundingBox CarlaUtility::toCarla(osi3::Dimension3d* dimension, osi3::Vector3d* position) {
	carla::geom::Location pos = CarlaUtility::toCarla(position);
	carla::geom::Vector3D extent((float)(dimension->length() / 2.0), (float)(dimension->width() / 2.0), (float)(dimension->height() / 2.0));
	return carla::geom::BoundingBox(pos, extent);
}

carla::geom::Location CarlaUtility::toCarla(osi3::Vector3d* position) {
	//flip y
	return carla::geom::Location((float)position->x(), (float)-position->y(), (float)position->z());
}

carla::geom::Vector2D CarlaUtility::toCarla(osi3::Vector2d* position) {
	return carla::geom::Vector2D((float)position->x(), (float)position->y());
}

osi3::Identifier* CarlaUtility::toOSI(carla::ActorId actorID)
{
	osi3::Identifier* id = new osi3::Identifier();
	id->set_value(actorID);
	return id;
}

carla::ActorId CarlaUtility::toCarla(osi3::Identifier* id)
{
	return (carla::ActorId)(id->value());
}

osi3::StationaryObject* CarlaUtility::toOSIStationaryObject(carla::SharedPtr< carla::client::Actor> actor)
{
	osi3::StationaryObject* prop = new osi3::StationaryObject();
	prop->set_allocated_id(CarlaUtility::toOSI(actor->GetId()));

	osi3::BaseStationary* base = prop->mutable_base();
	//TODO bounding boxes are only available for Junction, Vehicle and Walker, not for Actor as generalization (though there is a protected GetBoundingBox() member in ActorState)
	// also mentioned in https://github.com/carla-simulator/carla/issues/3025
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	//base->set_allocated_dimension(dimension);
	auto transform = actor->GetTransform();
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(CarlaUtility::toOSI(transform.rotation));

	//TODO How to get base_polygon from actor? (https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1BaseStationary.html#aa1db348acaac2d5a2ba0883903d962cd)

	//TODO Carla doesn't seem to offer information needed for osi3::StationaryObject::Classification. Using default instance
	prop->mutable_classification();//creates default instance as side-effect

	prop->set_model_reference(actor->GetTypeId());

	return prop;
}

osi3::TrafficSign* CarlaUtility::toOSITrafficSign(carla::SharedPtr<carla::client::Actor> actor)
{
	osi3::TrafficSign* sign = new osi3::TrafficSign();

	auto main = sign->mutable_main_sign();
	auto base = main->mutable_base();
	//TODO defined bounding box of traffic signs declare hit boxes where their restrictions should apply and don't identify the bounds of the sign
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	//base->set_allocated_dimension(dimension);
	auto transform = actor->GetTransform();
	base->set_allocated_position(CarlaUtility::toOSI(transform.location));
	// OSI traffic signs point along x, while Carla traffic signs point along y => rotate yaw by 90°
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
		//Unknown as part of ground truth is forbidden
		classification->set_type(osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_OTHER);
	}


	return sign;
}
