#pragma once

#include <carla/client/Junction.h>
#include <carla/client/Map.h>
#include <carla/road/RoadTypes.h>

#include <osi_common.pb.h>
#include <osi_lane.pb.h>
#include <osi_roadmarking.pb.h>

namespace carla_osi::lanes {

	google::protobuf::RepeatedPtrField<osi3::Lane::Classification::LanePairing> GetOSILanePairings(
		const carla::road::Map& roadMap, const carla::road::element::Waypoint& roadStart, const carla::road::element::Waypoint& roadEnd);

	/*
	parsing from carla roadmarking to osi laneboundary

	some lane marking types define double lines, resulting in two lane boundaries
	*/
	std::pair<std::unique_ptr<osi3::LaneBoundary::Classification>, std::unique_ptr<osi3::LaneBoundary::Classification>>
		parseLaneBoundary(const carla::road::element::LaneMarking&);
	// \return tuple consisting of osi boundaries, left_lane_boundary_id, right_lane_boundary_id
	std::tuple<google::protobuf::RepeatedPtrField<osi3::LaneBoundary>, uint64_t, uint64_t> parseLaneBoundary(
		carla::client::Map::TopologyList::value_type laneSection);
}