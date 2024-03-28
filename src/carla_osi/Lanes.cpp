#include "carla_osi/Lanes.h"

google::protobuf::RepeatedPtrField<osi3::Lane::Classification::LanePairing> carla_osi::lanes::GetOSILanePairings(
	const carla::road::Map& roadMap, const carla::road::element::Waypoint& roadStart, const carla::road::element::Waypoint& roadEnd)
{
	google::protobuf::RepeatedPtrField<osi3::Lane::Classification::LanePairing> lanePairings;

	//add antecesseor/successor pairs
	for (const auto& inbound : roadMap.GetPredecessors(roadStart)) {
		for (const auto& outbound : roadMap.GetSuccessors(roadEnd)) {
			auto pair = lanePairings.Add();

			pair->set_allocated_antecessor_lane_id(carla_osi::id_mapping::getOSIWaypointId(inbound, roadMap).release());

			pair->set_allocated_successor_lane_id(carla_osi::id_mapping::getOSIWaypointId(outbound, roadMap).release());
		}
	}
	return lanePairings;
}

google::protobuf::RepeatedPtrField<osi3::Lane::Classification::LanePairing> carla_osi::lanes::GetOSILanePairings(
	const carla::road::Map& roadMap, const carla::traffic_manager::WaypointPtr& roadStart, const  carla::traffic_manager::WaypointPtr& roadEnd)
{
	google::protobuf::RepeatedPtrField<osi3::Lane::Classification::LanePairing> lanePairings;

	//add antecesseor/successor pairs
	for (const auto& inbound : roadStart->GetPrevious(10)) {
		for (const auto& outbound : roadEnd->GetNext(10)) {
			auto pair = lanePairings.Add();

			pair->set_allocated_antecessor_lane_id(carla_osi::id_mapping::getOSIWaypointId(inbound).release());

			pair->set_allocated_successor_lane_id(carla_osi::id_mapping::getOSIWaypointId(outbound).release());
		}
	}
	return lanePairings;
}

std::pair<std::unique_ptr<osi3::LaneBoundary::Classification>, std::unique_ptr<osi3::LaneBoundary::Classification>>
carla_osi::lanes::parseLaneBoundary(const carla::road::element::LaneMarking& laneMarking) {

	auto laneBoundaryClassification = std::make_unique<osi3::LaneBoundary::Classification>();
	// holds second classification in case of double lines. Is null otherwise
	std::unique_ptr<osi3::LaneBoundary::Classification> laneBoundaryClassification2;
	switch (laneMarking.color) {
	case carla::road::element::LaneMarking::Color::White:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_WHITE);
		break;
	case carla::road::element::LaneMarking::Color::Yellow:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_YELLOW);
		break;
	case carla::road::element::LaneMarking::Color::Blue:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_BLUE);
		break;
	case carla::road::element::LaneMarking::Color::Green:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_GREEN);
		break;
	case carla::road::element::LaneMarking::Color::Red:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_RED);
		break;
	case carla::road::element::LaneMarking::Color::Other:
		laneBoundaryClassification->set_color(osi3::LaneBoundary_Classification_Color_COLOR_OTHER);
		break;
	}
	switch (laneMarking.type) {
	case carla::road::element::LaneMarking::Type::BottsDots:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_BOTTS_DOTS);
	case carla::road::element::LaneMarking::Type::BrokenBroken:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		break;
	case carla::road::element::LaneMarking::Type::BrokenSolid:
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		laneBoundaryClassification2->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Broken:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Curb:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_CURB);
		break;
	case carla::road::element::LaneMarking::Type::Grass:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_GRASS_EDGE);
		break;
	case carla::road::element::LaneMarking::Type::Other:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_OTHER);
		break;
	case carla::road::element::LaneMarking::Type::SolidSolid:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		break;
	case carla::road::element::LaneMarking::Type::SolidBroken:
		laneBoundaryClassification2 = std::make_unique<osi3::LaneBoundary::Classification>();
		laneBoundaryClassification2->CopyFrom(*laneBoundaryClassification.get());
		laneBoundaryClassification2->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE);
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		break;
	case carla::road::element::LaneMarking::Type::Solid:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE);
		break;
	case carla::road::element::LaneMarking::Type::None:
		laneBoundaryClassification->set_type(osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN);
		break;
	}

	return { std::move(laneBoundaryClassification), std::move(laneBoundaryClassification2) };
}

std::tuple<google::protobuf::RepeatedPtrField<osi3::LaneBoundary>, uint64_t, uint64_t>
carla_osi::lanes::parseLaneBoundary(carla::client::Map::TopologyList::value_type laneSection) {
	auto &begin = std::get<0>(laneSection);
	auto &end = std::get<1>(laneSection);
	// From OSI documentationon of osi3::LaneBoundary::Classification::Type:
	// There is no special representation for double lines, e.g. solid / solid or dashed / solid. In such 
	// cases, each lane will define its own side of the lane boundary.

	// => Always parse the outer lane boundary and only add the inner boundary if it consists of double lines to prevent duplicate entries. 
	//    Center lanes (laneId == 0) only have outer boundaries
	bool leftLaneIsOuter = 0 >= begin->GetLaneId();
	bool rightLaneIsOuter = 0 <= begin->GetLaneId();
	google::protobuf::RepeatedPtrField<osi3::LaneBoundary> laneBoundaries;
	uint64_t left_lane_boundary_id, right_lane_boundary_id;

	auto leftLaneMarking = begin->GetLeftLaneMarking();
	if (leftLaneMarking) {
		auto leftClassifications = parseLaneBoundary(leftLaneMarking.value());
		if (leftLaneIsOuter) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(leftClassifications.first.release());
			laneBoundary->set_allocated_id(
				carla_osi::id_mapping::getOSIWaypointBoundaryId(begin,
					carla_osi::id_mapping::RoadIDType_e::OuterBoundaryLine).release());
			left_lane_boundary_id = laneBoundary->id().value();
		}
		else if (leftClassifications.second) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(leftClassifications.second.release());
			laneBoundary->set_allocated_id(
				carla_osi::id_mapping::getOSIWaypointBoundaryId(begin,
					carla_osi::id_mapping::RoadIDType_e::InnerBoundaryLine).release());
			left_lane_boundary_id = laneBoundary->id().value();
		}
	}

	auto rightLaneMarking = begin->GetRightLaneMarking();
	if (rightLaneMarking) {
		auto rightClassifications = parseLaneBoundary(rightLaneMarking.value());
		if (rightLaneIsOuter) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(rightClassifications.first.release());
			laneBoundary->set_allocated_id(
				carla_osi::id_mapping::getOSIWaypointBoundaryId(begin,
					carla_osi::id_mapping::RoadIDType_e::OuterBoundaryLine).release());
			right_lane_boundary_id = laneBoundary->id().value();
		}
		else if (rightClassifications.second) {
			auto laneBoundary = laneBoundaries.Add();
			laneBoundary->set_allocated_classification(rightClassifications.second.release());
			laneBoundary->set_allocated_id(
				carla_osi::id_mapping::getOSIWaypointBoundaryId(begin,
					carla_osi::id_mapping::RoadIDType_e::InnerBoundaryLine).release());
			right_lane_boundary_id = laneBoundary->id().value();
		}
	}

	//TODO Add BoundaryPoints - maybe parse from OpenDRIVE file?

	return { laneBoundaries, left_lane_boundary_id, right_lane_boundary_id };
}
