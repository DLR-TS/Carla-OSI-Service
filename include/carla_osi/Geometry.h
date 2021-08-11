/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef GEOMETRY_H
#define GEOMETRY_H

#define _USE_MATH_DEFINES

#include <carla/geom/BoundingBox.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector2D.h>
#include <carla/geom/Vector3D.h>

#include "osi_common.pb.h"

namespace carla_osi {
	namespace geometry {

		carla::geom::Vector3D mul(const carla::geom::Vector3D& vector, const float f);

		// Geometric translations
		// Coordinate system in Carla/UE4:	left-handed,	X->forward, rot-, Y->right, rot-,	Z->up, rot+
		// OSI/DIN ISO 8855:				right-handed,	X->forward, rot+, Y->left, rot+,	Z->up, rot+		(see also https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames#R6)

		std::unique_ptr<osi3::Orientation3d> toOSI(const carla::geom::Rotation& rotation);
		std::pair<std::unique_ptr<osi3::Dimension3d>, std::unique_ptr<osi3::Vector3d>> toOSI(const carla::geom::BoundingBox& boundingBox);
		std::unique_ptr<osi3::Vector3d> toOSI(const carla::geom::Vector3D& location);
		//carla::geom::Vector3D is a generalization of carla::geom::Location
		//osi3::Vector3d toOSI(const carla::geom::Location& location);
		std::unique_ptr<osi3::Vector2d> toOSI(const carla::geom::Vector2D& vector);
		std::unique_ptr<osi3::MountingPosition> toOSI(const carla::geom::Transform& transform);

		carla::geom::Rotation toCarla(const osi3::Orientation3d* orientation);
		carla::geom::BoundingBox toCarla(const osi3::Dimension3d* dimension, const osi3::Vector3d* position);
		carla::geom::Location toCarla(const osi3::Vector3d* position);
		carla::geom::Vector2D toCarla(const osi3::Vector2d* vector);
	}
}

#endif //!GEOMETRY_H
