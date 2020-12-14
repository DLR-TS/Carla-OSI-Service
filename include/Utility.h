#ifndef CARLAUTILITY_H
#define CARLAUTILITY_H

#define _USE_MATH_DEFINES

#include <iterator>
#include <math.h>
#include <optional>
#include <variant>

#include <carla/client/Actor.h>
#include <carla/client/Junction.h>
#include <carla/client/LightState.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector3D.h>
#include <carla/geom/Vector2D.h>
#include <carla/road/RoadTypes.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

//#include "pugixml.hpp"

//uncomment includes if needed
#include "osi_common.pb.h"
//#include "osi_datarecording.pb.h"
//#include "osi_detectedlane.pb.h"
//#include "osi_detectedobject.pb.h"
//#include "osi_detectedoccupant.pb.h"
//#include "osi_detectedroadmarking.pb.h"
//#include "osi_detectedtrafficlight.pb.h"
//#include "osi_detectedtrafficsign.pb.h"
//#include "osi_environment.pb.h"
//#include "osi_featuredata.pb.h"
#include "osi_groundtruth.pb.h"
//#include "osi_hostvehicledata.pb.h"
#include "osi_lane.pb.h"
//#include "osi_object.pb.h"
//#include "osi_occupant.pb.h"
#include "osi_roadmarking.pb.h"
#include "osi_sensordata.pb.h"
//#include "osi_sensorspecific.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_trafficlight.pb.h"
#include "osi_trafficsign.pb.h"
//#include "osi_version.pb.h"

//TODO no object needed => no class needed, but we also don't want to use namespaces...
//class CarlaUtility {
namespace CarlaUtility {

	//OSI uses a single identifier database
	//We need to avoid using ids of possibly existing in another type,
	//as Carla uses different counters for ids of actors, roads/lanes and junctions. (As does OpenDRIVE)
	//
	//LaneIDs are not globally unique, but relative to their road. Also, they are close to 0, as they have to be defined continuously from 0, up and/or down. Thus, they have to be combined with their RoadID
	typedef std::variant<carla::ActorId, std::tuple<carla::road::RoadId, carla::road::LaneId, uint16_t>, carla::road::JuncId> CarlaUniqueID_t;
	//
	//Carla ids have only 32 bits, which will be copied into the lower 32 bits of the OSI identifier. (RoadID in case of Lanes)
	//The type will be marked in the upper 8 bits of the osi3::Identifier's 64bit value, the remaining 24 upper bits are used for special sub-ids, e.g. the LaneID and road mark id. This limits the implementation to only 256 lanes per road.
	union IDUnion {
		// OSI id value
		uint64_t value;
		struct {
			// Carla/OpenDRIVE id (both uint32_t)
			uint32_t id;

			// sub-sub-id, e. g. road section and road markings of a lane
			uint16_t special2;
			
			// Carla/OpenDRIVE sub-id, for example, LaneID as part of a road id
			int8_t special;

			// Carla/OpenDRIVE id type
			//uint16_t type;
			uint8_t type;
				
		};
	};
	//osi3::Identifier* toOSI(CarlaUniqueID_t id);
	//CarlaUniqueID_t toCarla(osi3::Identifier* identifier);

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

	// Only specialized actors have a bounding box, therefore it has to be passed as additional argument
	osi3::StationaryObject* toOSI(const carla::SharedPtr<const carla::client::Actor> actor, carla::geom::BoundingBox& bbox);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Walker> walker);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> vehicle);
	// common part of above methods
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving_common(const carla::SharedPtr<const carla::client::Actor> actor, std::unique_ptr<osi3::BaseMoving> base);
	osi3::TrafficSign* toOSI(const carla::SharedPtr<const carla::client::TrafficSign> actor/*, const pugi::xml_document& xodr*/);
	std::vector<osi3::TrafficLight*> toOSI(const carla::SharedPtr<const carla::client::TrafficLight> actor/*, const pugi::xml_document& xodr*/);

	std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> toOSI(carla::client::Vehicle::LightState lightState);

	carla::rpc::VehicleLightState::LightState toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState);

	/*
	parsing from carla roadmarking to osi laneboundary

	some lane marking types define double lines, resulting in two lane boundaries
	*/
	std::pair<std::unique_ptr<osi3::LaneBoundary::Classification>, std::unique_ptr<osi3::LaneBoundary::Classification>>
		parseLaneBoundary(const carla::road::element::LaneMarking&);
	// \return tuple consisting of osi boundaries, left_lane_boundary_id, right_lane_boundary_id
	std::tuple<google::protobuf::RepeatedPtrField<osi3::LaneBoundary>,uint64_t, uint64_t> parseLaneBoundary(
		carla::client::Map::TopologyList::value_type laneSection);

	osi3::MovingObject_VehicleClassification_Type ParseVehicleType(const std::string& typeName);

	osi3::CameraSensorView* toOSICamera(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData);
	osi3::LidarSensorView* toOSILidar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData);
	osi3::RadarSensorView* toOSIRadar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData);

	carla::SharedPtr<carla::client::Vehicle> getParentVehicle(const carla::SharedPtr<const carla::client::Actor> actor);

	/**
	* Comparing the first two sorted containers, determine which elements are only in the first container @a rem_first or second container @a add_first
	*/
	template<class CurIt, class UpdIt, class AddIt, class RemIt>
	std::pair<AddIt, RemIt> twoWayDifference(CurIt first1, CurIt last1, UpdIt first2, UpdIt last2, AddIt add_first, RemIt rem_first) {
		while (first1 != last1) {
			if (first2 == last2) {
				rem_first = std::copy(first1, last1, rem_first);
				break;
			}
			else if (*first1 < *first2) {
				*rem_first++ = *first1++;
			}
			else {
				if (*first2 < *first1) {
					*add_first++ = *first2;
				}
				else {
					++first1;
				}
				++first2;
			}
		}
		if (first2 != last2) {
			add_first = std::copy(first2, last2, add_first);
		}
		return std::pair(add_first, rem_first);
	};

};

#endif !CARLAUTILITY_H
