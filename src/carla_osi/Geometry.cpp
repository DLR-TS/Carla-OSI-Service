#include "carla_osi/Geometry.h"

carla::geom::Vector3D carla_osi::geometry::mul(const carla::geom::Vector3D & vector, const float f)
{
	return carla::geom::Vector3D(vector.x * f, vector.y * f, vector.z * f);
}

std::unique_ptr<osi3::Orientation3d> carla_osi::geometry::toOSI(const carla::geom::Rotation& rotation)
{
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	std::unique_ptr<osi3::Orientation3d> orient = std::make_unique<osi3::Orientation3d>();
	//TODO OSI prefers values in angular range [pi,pi]
	orient->set_yaw(-rotation.yaw * M_PI / 180.0);
	orient->set_pitch(rotation.pitch * M_PI / 180.0);
	orient->set_roll(rotation.roll * M_PI / 180.0);
	return orient;
}

std::pair<std::unique_ptr<osi3::Dimension3d>, std::unique_ptr<osi3::Vector3d>> carla_osi::geometry::toOSI(const carla::geom::BoundingBox& boundingBox) {
	std::unique_ptr<osi3::Dimension3d> dim = std::make_unique<osi3::Dimension3d>();
	// dimensions are unsigned
	dim->set_length(boundingBox.extent.x * 2);
	dim->set_width(boundingBox.extent.y * 2);
	dim->set_height(boundingBox.extent.z * 2);
	std::unique_ptr<osi3::Vector3d> vec = carla_osi::geometry::toOSI(boundingBox.location);
	return std::pair(std::move(dim), std::move(vec));
}

std::unique_ptr<osi3::Vector3d> carla_osi::geometry::toOSI(const carla::geom::Vector3D& location) {
	//flip y
	std::unique_ptr<osi3::Vector3d> vec = std::make_unique<osi3::Vector3d>();
	vec->set_x(location.x);
	vec->set_y(-location.y);
	vec->set_z(location.z);
	return vec;
}

std::unique_ptr<osi3::Vector2d> carla_osi::geometry::toOSI(const carla::geom::Vector2D& vector) {
	//flip y
	std::unique_ptr<osi3::Vector2d> vec = std::make_unique<osi3::Vector2d>();
	vec->set_x(vector.x);
	vec->set_y(vector.y);
	return vec;
}

std::unique_ptr<osi3::MountingPosition> carla_osi::geometry::toOSI(const carla::geom::Transform & transform)
{
	auto mountingPosition = std::make_unique<osi3::MountingPosition>();
	mountingPosition->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	mountingPosition->set_allocated_orientation(carla_osi::geometry::toOSI(transform.rotation).release());
	return mountingPosition;
}

carla::geom::Rotation carla_osi::geometry::toCarla(const osi3::Orientation3d* orientation) {
	// According to https://carla.readthedocs.io/en/0.9.9/python_api/#carlarotation, Carla/UE4 uses right-hand rotations except for yaw, even though the coordinate system is defined as left-handed.
	// Iff the rotations are performed in the same order (//TODO could not find any information on this in UE4 documentation), only change of signage of yaw and conversion from radians to degree is needed.
	return carla::geom::Rotation(
		(float)(orientation->pitch() * 180 * M_1_PI),
		(float)(orientation->yaw() * -180 * M_1_PI),
		(float)(orientation->roll() * 180 * M_1_PI));
}

carla::geom::BoundingBox carla_osi::geometry::toCarla(const osi3::Dimension3d* dimension, const osi3::Vector3d* position) {
	carla::geom::Location pos = carla_osi::geometry::toCarla(position);
	carla::geom::Vector3D extent((float)(dimension->length() / 2.0), (float)(dimension->width() / 2.0), (float)(dimension->height() / 2.0));
	return carla::geom::BoundingBox(pos, extent);
}

carla::geom::Location carla_osi::geometry::toCarla(const osi3::Vector3d* position) {
	//flip y
	return carla::geom::Location((float)position->x(), (float)-position->y(), (float)position->z());
}

carla::geom::Vector2D carla_osi::geometry::toCarla(const osi3::Vector2d* position) {
	return carla::geom::Vector2D((float)position->x(), (float)position->y());
}