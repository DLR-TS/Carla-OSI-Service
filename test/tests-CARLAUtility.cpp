#include "catch2/catch.hpp"

#include "base_interfaces/Carla/Utility.h"

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

TEST_CASE("Coordinate system conversion Carla <=> OSI", "[Carla][Utility]") {
	SECTION("toOSI") {
		SECTION("Rotation") {
			carla::geom::Rotation rotation(45, -90, 180);
			osi3::Orientation3d* orientation = CarlaUtility::toOSI(rotation);
			REQUIRE(M_PI_4 == orientation->pitch());
			REQUIRE(M_PI_2 == orientation->yaw());
			REQUIRE(M_PI == orientation->roll());
		}

		SECTION("Location") {
			carla::geom::Location location(1.2f, 3.4f, 5.6f);
			osi3::Vector3d* position = CarlaUtility::toOSI(location);
			REQUIRE(1.2f == position->x());
			REQUIRE(-3.4f == position->y());
			REQUIRE(5.6f == position->z());
		}

		SECTION("2D Vector") {
			carla::geom::Vector2D vector(0.1f, -0.1f);
			osi3::Vector2d* vector2d = CarlaUtility::toOSI(vector);
			REQUIRE(0.1f == vector2d->x());
			REQUIRE(-0.1f == vector2d->y());
		}

		SECTION("BoundingBox") {
			carla::geom::Location location(1.2f, 3.4f, 5.6f);
			carla::geom::BoundingBox boundingBox(location, carla::geom::Vector3D(1.f, 2.f, 4.f));
			std::pair<osi3::Dimension3d*, osi3::Vector3d*> osiBB = CarlaUtility::toOSI(boundingBox);
			REQUIRE(1.2f == osiBB.second->x());
			REQUIRE(-3.4f == osiBB.second->y());
			REQUIRE(5.6f == osiBB.second->z());
			REQUIRE(2.f == osiBB.first->length());
			REQUIRE(4.f == osiBB.first->width());
			REQUIRE(8.f == osiBB.first->height());
		}
	}
	SECTION("toCarla") {
		SECTION("Rotation") {
			osi3::Orientation3d orientation;
			orientation.set_pitch(M_PI_4);
			orientation.set_yaw(M_PI_2);
			orientation.set_roll(M_PI - M_PI_4);
			carla::geom::Rotation rotation = CarlaUtility::toCarla(&orientation);
			REQUIRE(45.f == rotation.pitch);
			REQUIRE(-90.f == rotation.yaw);
			REQUIRE(135.f == rotation.roll);
		}

		SECTION("Location") {
			osi3::Vector3d position;
			position.set_x(0.1);
			position.set_y(2.3);
			position.set_z(4.5);
			carla::geom::Location location = CarlaUtility::toCarla(&position);
			REQUIRE(0.1f == location.x);
			REQUIRE(-2.3f == location.y);
			REQUIRE(4.5f == location.z);
		}

		SECTION("2D Vector") {
			osi3::Vector2d vector2d;
			vector2d.set_x(0.1);
			vector2d.set_y(-0.1);
			carla::geom::Vector2D vector = CarlaUtility::toCarla(&vector2d);
			REQUIRE(0.1f == vector.x);
			REQUIRE(-0.1f == vector.y);
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
			carla::geom::BoundingBox boundingBox = CarlaUtility::toCarla(&dimension, &position);
			REQUIRE(0.1f == boundingBox.location.x);
			REQUIRE(-2.3f == boundingBox.location.y);
			REQUIRE(4.5f == boundingBox.location.z);
			REQUIRE(4.f == boundingBox.extent.x);
			REQUIRE(2.f == boundingBox.extent.y);
			REQUIRE(5.f == boundingBox.extent.z);
		}

	}
}

TEST_CASE("Carla Prop to StationaryObject", "[Carla][Utility][!hide][RequiresCarlaServer]") {
	//TODO find a way to use the ActorFactory without a carla server

}