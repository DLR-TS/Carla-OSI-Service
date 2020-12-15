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

osi3::Identifier * carla_osi::id_mapping::toOSI(const uint32_t id, carla_osi::id_mapping::CarlaUniqueID_e type)
{
	//type has to match index in variant CarlaUniqueID_t
	return toOSI(id, 0, type);
}

osi3::Identifier * carla_osi::id_mapping::toOSI(const uint32_t roadId, const int8_t laneId, CarlaUniqueID_e type) {
	return toOSI(roadId, laneId, 0u, type);
}

osi3::Identifier * carla_osi::id_mapping::toOSI(const uint32_t roadId, const int8_t laneId, const uint16_t sectionId, CarlaUniqueID_e type)
{
	carla_osi::id_mapping::IDUnion idUnion;
	idUnion.type = type;
	idUnion.special2 = sectionId;
	idUnion.special = (int16_t)laneId;
	idUnion.id = roadId;

	osi3::Identifier* identifier = new osi3::Identifier();
	identifier->set_value(idUnion.value);
	return identifier;
}

osi3::Identifier * carla_osi::id_mapping::toOSI(const carla::road::RoadId roadId, const carla::road::LaneId laneId, const uint16_t sectionId, const RoadIDType_e roadMarkType, CarlaUniqueID_e type)
{
	if (-3u < sectionId) {
		throw std::out_of_range("Section id is too large and cannot be differentiated from road mark id");
	}

	uint16_t sectionIdWithRoadMarkType = sectionId | ((uint16_t)roadMarkType) << 8u;
	return toOSI(roadId, laneId, sectionIdWithRoadMarkType, type);
}