#pragma once

#include <string>

#include "carla_osi/Geometry.h"

#include "osi_common.pb.h"

#include "carla/client/Client.h"
#include "carla/client/Actor.h"
#include "carla/client/Map.h"
#include "carla/client/World.h"
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

template<class T>
static inline std::tuple<std::unique_ptr<carla::client::Client>, carla::client::World> getCarlaDefaultWorld(const std::string carlaHost, const uint16_t port, const std::string map, const std::chrono::duration<T> transactionTimeout, const bool reload = true) {
	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto client = std::make_unique<carla::client::Client>(carlaHost, port);
	client->SetTimeout(transactionTimeout);
	auto maps = client->GetAvailableMaps();
	auto world = client->GetWorld();
	std::string carlaMap = map;
	size_t length = map.size();
	if (!std::any_of(maps.begin(), maps.end(), [&maps, &carlaMap, &length](const std::string map) {
		return map.length() >= length && carlaMap.compare(map.substr(map.length() - length, length)) == 0;
	})) {
		// no map with name given in carlaMap - use Town10HD as fallback
		carlaMap = "Town10HD";
		std::cout << "CARLA Server has no map '" << map << "'! Using a default map instead" << std::endl;
		// default maps all begin with 'Town' - don't load another world if already using a default map
		length = 4;
	}
	const std::string currentMap = world.GetMap()->GetName();
	if (currentMap.rfind(carlaMap.substr(0, length), 0) == std::string::npos) {
		std::cout << "Destroying current world '" << currentMap << "' to load map '" << carlaMap << "'" << std::endl;
		world = client->LoadWorld(carlaMap);
		world.WaitForTick(std::chrono::seconds(45));
	}
	else if (reload) {
		std::cout << "Reloading current map '" << currentMap << "' to clear world" << std::endl;
		//clear world
		world = client->ReloadWorld();
		world.WaitForTick(std::chrono::seconds(45));
	}
	return { std::move(client), world };
}

static inline auto getCarlaDefaultWorld(const std::string carlaHost, const uint16_t port, const double transactionTimeout_seconds = 5, const std::string map = "Town10HD", const bool reload = true) {
	return getCarlaDefaultWorld<double>(carlaHost, port, map, std::chrono::duration<double>(transactionTimeout_seconds), reload);
}