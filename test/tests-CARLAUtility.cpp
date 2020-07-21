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