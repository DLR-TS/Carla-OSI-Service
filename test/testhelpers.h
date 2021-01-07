#pragma once

#include "carla_osi/Geometry.h"

#include "osi_common.pb.h"

#include "carla/client/Actor.h"
#include "carla/geom/BoundingBox.h"

static inline void testBaseMoving(const osi3::BaseMoving& base, carla::SharedPtr<const carla::client::Actor> actor, const carla::geom::BoundingBox& bbox) {
	REQUIRE(base.has_position());
	REQUIRE(carla_osi::geometry::toCarla(&base.position()) == (actor->GetLocation() + bbox.location));
	REQUIRE(base.has_dimension());
	REQUIRE(carla_osi::geometry::toCarla(&base.dimension(), &base.position()).extent == bbox.extent);
	REQUIRE(base.has_orientation());
	auto osiRotation = carla_osi::geometry::toCarla(&base.orientation());
	auto transform = actor->GetTransform();
	REQUIRE(osiRotation.pitch == transform.rotation.pitch);
	REQUIRE(osiRotation.yaw == transform.rotation.yaw);
	REQUIRE(osiRotation.roll == transform.rotation.roll);
}