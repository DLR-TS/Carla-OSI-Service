#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "..\..\..\include\base_interfaces\Carla\Utility.h"
#include "base_interfaces/Carla/Utility.h"

osi3::Orientation3d CarlaUtility::toOSI(carla::geom::Rotation& rotation)
{
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	osi3::Orientation3d orient;
	//TODO OSI prefers values in angular range [pi,pi]
	orient.set_yaw(-rotation.yaw * M_PI / 180.0);
	orient.set_pitch(rotation.pitch * M_PI / 180.0);
	orient.set_roll(rotation.roll * M_PI / 180.0);
	return orient;
}

std::pair<osi3::Dimension3d, osi3::Vector3d> CarlaUtility::toOSI(carla::geom::BoundingBox& boundingBox) {
	osi3::Dimension3d dim;
	// dimensions are unsigned
	dim.set_length(boundingBox.extent.x * 2);
	dim.set_width(boundingBox.extent.y * 2);
	dim.set_height(boundingBox.extent.z * 2);
	osi3::Vector3d vec = CarlaUtility::toOSI(boundingBox.location);
	return std::pair(dim, vec);
}

osi3::Vector3d CarlaUtility::toOSI(carla::geom::Vector3D& location) {
	//flip y
	osi3::Vector3d vec;
	vec.set_x(location.x);
	vec.set_y(-location.y);
	vec.set_z(location.z);
	return vec;
}

osi3::Vector2d CarlaUtility::toOSI(carla::geom::Vector2D& vector) {
	//flip y
	osi3::Vector2d vec;
	vec.set_x(vector.x);
	vec.set_y(vector.y);
	return vec;
}

carla::geom::Rotation CarlaUtility::toCarla(osi3::Orientation3d& orientation) {
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	return carla::geom::Rotation(
		(float)(orientation.pitch() * 180 * M_1_PI),
		(float)(orientation.yaw() * -180 * M_1_PI),
		(float)(orientation.roll() * 180 * M_1_PI));
}

carla::geom::BoundingBox CarlaUtility::toCarla(osi3::Dimension3d& dimension, osi3::Vector3d& position) {
	carla::geom::Location pos = CarlaUtility::toCarla(position);
	carla::geom::Vector3D extent((float)(dimension.length() / 2.0), (float)(dimension.width() / 2.0), (float)(dimension.height() / 2.0));
	return carla::geom::BoundingBox(pos, extent);
}

carla::geom::Location CarlaUtility::toCarla(osi3::Vector3d& position) {
	//flip y
	return carla::geom::Location((float)position.x(), (float)-position.y(), (float)position.z());
}

carla::geom::Vector2D CarlaUtility::toCarla(osi3::Vector2d& position) {
	return carla::geom::Vector2D((float)position.x(), (float)position.y());
}

osi3::Identifier CarlaUtility::toOSI(carla::ActorId actorID)
{
	osi3::Identifier id;
	id.set_value(actorID);
	return id;
}

carla::ActorId CarlaUtility::toCarla(osi3::Identifier id)
{
	return static_cast<carla::ActorId>(id.value);
}

osi3::StationaryObject CarlaUtility::toOSIStationaryObject(carla::SharedPtr< carla::client::Actor> actor)
{
	osi3::StationaryObject prop;
	prop.set_allocated_id(&CarlaUtility::toOSI(actor->GetId()));
	
	osi3::BaseStationary* base = prop.mutable_base();
	//TODO bounding boxes are only available for Junction, Vehicle and Walker, not for Actor as generalization (though there is a protected GetBoundingBox() member in ActorState)
	// also mentioned in https://github.com/carla-simulator/carla/issues/3025
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	//base->set_allocated_dimension(dimension);
	auto transform = actor->GetTransform();
	base->set_allocated_position(&CarlaUtility::toOSI(transform.location));
	base->set_allocated_orientation(&CarlaUtility::toOSI(transform.rotation));
	
	//TODO How to get base_polygon from actor? (https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1BaseStationary.html#aa1db348acaac2d5a2ba0883903d962cd)

	//TODO Carla doesn't seem to offer information needed for osi3::StationaryObject::Classification. Using default instance
	prop.mutable_classification();//creates default instance as side-effect

	prop.set_model_reference(actor->GetTypeId());

	return prop;
}
