#pragma once

#include <variant>

#include <carla/client/Actor.h>
#include <carla/client/Junction.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Waypoint.h>
#include <carla/road/RoadTypes.h>
#include <carla/road/element/Waypoint.h>
#include <carla/road/Map.h>
#include <carla/rpc/ActorId.h>
#include <carla/rpc/StationaryMapObject.h>

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
			ActorID = 1,
			RoadIDLaneID = 2,//also used for road marks, which are defined per lane in OpenDRIVE
			JuncID = 3,
			StationaryMapObject = 4,// currently not invertable
			EnvironmentObject = 4,
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
		std::unique_ptr<osi3::Identifier> toOSI(const uint32_t id, CarlaUniqueID_e type = ActorID);
		std::unique_ptr<osi3::Identifier> toOSI(const uint32_t id, int8_t special, const CarlaUniqueID_e type = RoadIDLaneID);
		std::unique_ptr<osi3::Identifier> toOSI(const uint32_t id, const int8_t special, const uint16_t special2, const CarlaUniqueID_e type = RoadIDLaneID);
		// stores roadMark  in the 2 upper bits of special 2
		std::unique_ptr<osi3::Identifier> toOSI(const carla::road::RoadId id, const carla::road::LaneId laneId, const uint16_t sectionId, RoadIDType_e roadMarkType, CarlaUniqueID_e type = RoadIDLaneID);
		CarlaUniqueID_t toCarla(const osi3::Identifier* identifier);

		std::unique_ptr<osi3::Identifier> getOSIActorId(carla::SharedPtr<const carla::client::Actor> actor);
		std::unique_ptr<osi3::Identifier> getOSITrafficLightId(carla::SharedPtr<const carla::client::TrafficLight> trafficLight, const int index);
		std::unique_ptr<osi3::Identifier> getOSIEnvironmentObjectId(const carla::rpc::StationaryMapObject& object);
		std::unique_ptr<osi3::Identifier> getOSIJunctionId(carla::SharedPtr<const carla::client::Junction> junction);
		std::unique_ptr<osi3::Identifier> getOSIWaypointId(carla::SharedPtr<const carla::client::Waypoint> waypoint);
		// same as above, but mark as inner or outer boundary id (second argument)
		std::unique_ptr<osi3::Identifier> getOSIWaypointBoundaryId(carla::SharedPtr<const carla::client::Waypoint> waypoint, const carla_osi::id_mapping::RoadIDType_e type);
		//std::unique_ptr<osi3::Identifier> getOSIWaypointBoundaryIds(carla::SharedPtr<const carla::client::Waypoint> waypoint);
		// Requires map object to differentiate between junctions and lanes
		std::unique_ptr<osi3::Identifier> getOSIWaypointId(const carla::road::element::Waypoint& waypoint, const carla::road::Map& map);
	}
}