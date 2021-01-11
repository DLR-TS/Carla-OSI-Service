#pragma once

#include <variant>

#include <carla/road/RoadTypes.h>
#include <carla/rpc/ActorId.h>

#include "osi_common.pb.h"

namespace carla_osi {
	namespace id_mapping {
		//OSI uses a single identifier database
		//We need to avoid using ids of possibly existing in another type,
		//as Carla uses different counters for ids of actors, roads/lanes and junctions. (As does OpenDRIVE)
		//
		//LaneIDs are not globally unique, but relative to their road. Also, they are close to 0, as they have to be defined continuously from 0, up and/or down. Thus, they have to be combined with their RoadID
		typedef std::variant<carla::ActorId, std::tuple<carla::road::RoadId, carla::road::LaneId, uint16_t>, carla::road::JuncId> CarlaUniqueID_t;
		//
		//Carla ids have only 32 bits, which will be copied into the lower 32 bits of the OSI identifier. (RoadID in case of Lanes)
		//The type will be marked in the next 8 bits of the osi3::Identifier's 64bit value, the remaining 24 upper bits are used for special sub-ids, e.g. the LaneID and road mark id. This limits the implementation to only 256 lanes per road.
		union IDUnion {
			// OSI id value
			uint64_t value;
			struct {
				// Carla/OpenDRIVE id (both uint32_t)
				uint32_t id;

				// Carla/OpenDRIVE id type
				//uint16_t type;
				uint8_t type;

				// Carla/OpenDRIVE sub-id, for example, LaneID as part of a road id
				int8_t special;

				// sub-sub-id, e. g. road section and road markings of a lane
				uint16_t special2;
			};
		};

		//should be the same index as used in CarlaUniqueID_t
		enum CarlaUniqueID_e : uint8_t {
			NoCarlaID = 0,
			StationaryMapObject = 0,
			ActorID = 1,
			RoadIDLaneID = 2,//also used for road marks, which are defined per lane in OpenDRIVE
			JuncID = 3,
		};//Unmapped globally unique OpenDRIVE ids: object, outline, tunnel, bridge, signal, controller, junctionGroup, (some for railroads: switch, mainTrack, sideTrack, station, platform),

		enum RoadIDType_e : uint8_t {
			// identifies a lane/section element
			LaneIDSectionID = 0,
			// identifies a road mark on the lane's outer side. 
			OuterBoundaryLine = 1 << 7,
			// Used when there is a double line and thus the inner line also has to be stored. Otherwise, the inner boundary is the inner lane's outer boundary
			InnerBoundaryLine = 1 << 6,
		};

		//Since some Carla ids are typedefs of the same primitive type, the relevant type for CarlaUniqueID can not be deduced because of the ambiguity
		osi3::Identifier* toOSI(const uint32_t id, CarlaUniqueID_e type = ActorID);
		osi3::Identifier* toOSI(const uint32_t id, int8_t special, const CarlaUniqueID_e type = RoadIDLaneID);
		osi3::Identifier* toOSI(const uint32_t id, const int8_t special, const uint16_t special2, const CarlaUniqueID_e type = RoadIDLaneID);
		// stores roadMark  in the 2 upper bits of special 2
		osi3::Identifier* toOSI(const carla::road::RoadId id, const carla::road::LaneId laneId, const uint16_t sectionId, RoadIDType_e roadMarkType, CarlaUniqueID_e type = RoadIDLaneID);
		CarlaUniqueID_t toCarla(const osi3::Identifier* identifier);
	}
}