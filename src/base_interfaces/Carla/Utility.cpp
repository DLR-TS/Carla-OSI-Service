#include "base_interfaces/Carla/Utility.h"

osi3::Orientation3d CarlaUtility::toOSI(carla::geom::Rotation& rotation)
{
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	osi3::Orientation3d orient;
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

carla::geom::Rotation CarlaUtility::toCarla(osi3::Orientation3d& orientation) {
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	return carla::geom::Rotation((float)(orientation.pitch() * 180 / M_PI), (float)(orientation.yaw() * 180 / M_PI), (float)(orientation.roll() * 180 / M_PI));
}

carla::geom::BoundingBox CarlaUtility::toCarla(osi3::Dimension3d& dimension, osi3::Vector3d& position) {
	//TODO
	carla::geom::Location pos = CarlaUtility::toCarla(position);
	carla::geom::Vector3D extent((float)(dimension.length() / 2.0), (float)(dimension.width() / 2.0), (float)(dimension.height() / 2.0));
	return carla::geom::BoundingBox();
}

carla::geom::Location CarlaUtility::toCarla(osi3::Vector3d& position) {
	//flip y
	return carla::geom::Location((float)position.x(), (float)-position.y(), (float)position.z());
}