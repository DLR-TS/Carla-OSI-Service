#include "carla_osi/Identifiers.h"

using ID = carla_osi::id_mapping::CarlaUniqueID_t;

carla_osi::id_mapping::CarlaUniqueID_t carla_osi::id_mapping::toCarla(const osi3::Identifier* identifier) {
	IDUnion idUnion{ identifier->value() };

	switch ((CarlaUniqueID_e)idUnion.type) {
	default:
	case ActorID:
		return (carla::ActorId) idUnion.id;
	case RoadIDLaneID:
		return std::make_tuple((carla::road::RoadId)idUnion.id, (carla::road::LaneId)idUnion.special, idUnion.special2);
	case JuncID:
		return (carla::road::JuncId) idUnion.id;
	}

}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIActorId(carla::SharedPtr<const carla::client::Actor> actor)
{
	return toOSI(actor->GetId(), carla_osi::id_mapping::CarlaUniqueID_e::ActorID);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSITrafficLightId(carla::SharedPtr<const carla::client::TrafficLight> trafficLight, const int index)
{
	return toOSI(trafficLight->GetId(), index, carla_osi::id_mapping::CarlaUniqueID_e::ActorID);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIJunctionId(carla::SharedPtr<const carla::client::Junction> junction)
{
	return toOSI(junction->GetId(), carla_osi::id_mapping::CarlaUniqueID_e::JuncID);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIEnvironmentObjectId(const carla::rpc::EnvironmentObject& object)
{
	return toOSI(object.id, carla_osi::id_mapping::CarlaUniqueID_e::EnvironmentObject);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIWaypointId(carla::SharedPtr<const carla::client::Waypoint> waypoint)
{
	if (waypoint->IsJunction()) {
		return toOSI(waypoint->GetJunctionId(), carla_osi::id_mapping::CarlaUniqueID_e::JuncID);
	}
	else {
		return toOSI(waypoint->GetRoadId(), waypoint->GetLaneId(), waypoint->GetSectionId(), RoadIDLaneID);
	}
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIWaypointBoundaryId(carla::SharedPtr<const carla::client::Waypoint> waypoint, const carla_osi::id_mapping::RoadIDType_e type)
{
	if (carla_osi::id_mapping::RoadIDType_e::LaneIDSectionID == type) {
		// not a boundary id, so return road/lane/section id
		return getOSIWaypointId(waypoint);
	}
	else {
		return toOSI(waypoint->GetRoadId(), waypoint->GetLaneId(), waypoint->GetSectionId(), type, RoadIDLaneID);
	}
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::getOSIWaypointId(const carla::road::element::Waypoint& waypoint, const carla::road::Map& map)
{
	if (map.IsJunction(waypoint.road_id)) {
		return carla_osi::id_mapping::toOSI(map.GetJunctionId(waypoint.road_id), carla_osi::id_mapping::JuncID);
	}
	else {
		return carla_osi::id_mapping::toOSI(waypoint.road_id,
			waypoint.lane_id, waypoint.section_id, carla_osi::id_mapping::RoadIDLaneID);
	}
}


std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::toOSI(const uint32_t id, carla_osi::id_mapping::CarlaUniqueID_e type)
{
	//type has to match index in variant CarlaUniqueID_t
	return toOSI(id, 0, type);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::toOSI(const uint32_t roadId, const int8_t laneId, CarlaUniqueID_e type) {
	return toOSI(roadId, laneId, 0u, type);
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::toOSI(const uint32_t roadId, const int8_t laneId, const uint16_t sectionId, CarlaUniqueID_e type)
{
	carla_osi::id_mapping::IDUnion idUnion;
	idUnion.type = type;
	idUnion.special2 = sectionId;
	idUnion.special = (int16_t)laneId;
	idUnion.id = roadId;

	std::unique_ptr<osi3::Identifier> identifier = std::make_unique<osi3::Identifier>();
	identifier->set_value(idUnion.value);
	return identifier;
}

std::unique_ptr<osi3::Identifier> carla_osi::id_mapping::toOSI(const carla::road::RoadId roadId, const carla::road::LaneId laneId, const uint16_t sectionId, const RoadIDType_e roadMarkType, CarlaUniqueID_e type)
{
	if (-3u < sectionId) {
		throw std::out_of_range("Section id is too large and cannot be differentiated from road mark id");
	}

	uint16_t sectionIdWithRoadMarkType = sectionId | ((uint16_t)roadMarkType) << 8u;
	return toOSI(roadId, laneId, sectionIdWithRoadMarkType, type);
}