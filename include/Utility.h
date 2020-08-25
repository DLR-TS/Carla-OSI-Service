#pragma once

#define _USE_MATH_DEFINES

#include <iterator>
#include <math.h>
#include <optional>
#include <variant>

#include <carla/client/Actor.h>
#include <carla/client/Junction.h>
#include <carla/client/LightState.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector3D.h>
#include <carla/geom/Vector2D.h>
#include <carla/road/RoadTypes.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

#include "pugixml.hpp"

#include "osi_common.pb.h"
#include "osi_datarecording.pb.h"
#include "osi_detectedlane.pb.h"
#include "osi_detectedobject.pb.h"
#include "osi_detectedoccupant.pb.h"
#include "osi_detectedroadmarking.pb.h"
#include "osi_detectedtrafficlight.pb.h"
#include "osi_detectedtrafficsign.pb.h"
#include "osi_environment.pb.h"
#include "osi_featuredata.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_hostvehicledata.pb.h"
#include "osi_lane.pb.h"
#include "osi_object.pb.h"
#include "osi_occupant.pb.h"
#include "osi_roadmarking.pb.h"
#include "osi_sensordata.pb.h"
#include "osi_sensorspecific.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_trafficlight.pb.h"
#include "osi_trafficsign.pb.h"
#include "osi_version.pb.h"

//TODO no object needed => no class needed, but we also don't want to use namespaces...
//class CarlaUtility {
namespace CarlaUtility {
	//std::variant<carla::client::Vehicle, carla::client::Walker, carla::client::Junction> boundingBoxType_variants;

	carla::geom::Vector3D mul(const carla::geom::Vector3D& vector, const float f);

	// Geometric translations
	// Coordinate system in Carla/UE4:	left-handed,	X->forward, rot-, Y->right, rot-,	Z->up, rot+
	// OSI/DIN ISO 8855:				right-handed,	X->forward, rot+, Y->left, rot+,	Z->up, rot+		(see also https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames#R6)

	osi3::Orientation3d* toOSI(const carla::geom::Rotation& rotation);
	std::pair<osi3::Dimension3d*, osi3::Vector3d*> toOSI(const carla::geom::BoundingBox& boundingBox);
	osi3::Vector3d* toOSI(const carla::geom::Vector3D& location);
	//carla::geom::Vector3D is a generalization of carla::geom::Location
	//osi3::Vector3d toOSI(const carla::geom::Location& location);
	osi3::Vector2d* toOSI(const carla::geom::Vector2D& vector);

	carla::geom::Rotation toCarla(const osi3::Orientation3d* orientation);
	carla::geom::BoundingBox toCarla(const osi3::Dimension3d* dimension, const osi3::Vector3d* position);
	carla::geom::Location toCarla(const osi3::Vector3d* position);
	carla::geom::Vector2D toCarla(const osi3::Vector2d* vector);

	//OSI uses a single identifier database
	//We need to avoid using ids of possibly existing in another type,
	//as Carla uses different counters for ids of actors, roads/lanes and junctions. (As does OpenDRIVE)
	//
	//LaneIDs are not globally unique, but relative to their road. Also, they are close to 0, as they have to be defined continuously from 0, up and/or down. Thus, they have to be combined with their RoadID
	typedef std::variant<carla::ActorId, std::pair<carla::road::RoadId, carla::road::LaneId>, carla::road::JuncId> CarlaUniqueID_t;
	//
	//Carla ids have only 32 bits, which will be copied into the lower 32 bits of the OSI identifier. (RoadID in case of Lanes)
	//The type will be marked in the upper 16 bits of the osi3::Identifier's 64bit value, the remaining 16 upper bits are used for the LaneID. This limits the implementation to only 65.534 lanes per road.
	union IDUnion {
		// OSI id value
		uint64_t value;
		struct {
			// Carla/OpenDRIVE id
			uint32_t id;
			// Carla/OpenDRIVE id type
			uint16_t type;
			// Carla/OpenDRIVE sub-id, for example, LaneID as part of a road id
			int16_t special;
		};
	};
	//osi3::Identifier* toOSI(CarlaUniqueID_t id);
	//CarlaUniqueID_t toCarla(osi3::Identifier* identifier);

	//should be the same index as used in CarlaUniqueID_t
	enum CarlaUniqueID_e : uint16_t {
		ActorID = 1,
		RoadIDLaneID = 2,
		JuncID = 3,
	};//Unused globally unique OpenDRIVE ids: object, outline, tunnel, bridge, signal, controller, junctionGroup, (some for railroads: switch, mainTrack, sideTrack, station, platform),

	//Since some Carla ids are typedefs of the same primitive type, the relevant type for CarlaUniqueID can not be deduced because of the ambiguity
	osi3::Identifier* toOSI(const uint32_t id, CarlaUniqueID_e type = ActorID);
	osi3::Identifier* toOSI(const carla::road::RoadId id, const carla::road::LaneId special, CarlaUniqueID_e type = RoadIDLaneID);
	CarlaUniqueID_t toCarla(const osi3::Identifier* identifier);

	////Create CarlaUniqueID_t with given type index and initial value. Used in carla id <-> OSI identifier conversion
	//template <uint32_t N = 1>//template meta programming
	//CarlaUniqueID_t getTyped(const uint32_t type, const uint32_t initialValue) {
	//	if (N == type) {
	//		return CarlaUniqueID_t((std::variant_alternative_t<N, CarlaUniqueID_t>)initialValue);
	//	}
	//	return getTyped<N + 1>(type, initialValue);
	//}
	//template <> //specialization as stop condition
	//CarlaUniqueID_t getTyped< std::variant_size_v<CarlaUniqueID_t> >(const uint32_t type, const uint32_t initialValue) {
	//	return std::monostate();
	//}


	osi3::StationaryObject* toOSIStationaryObject(const carla::SharedPtr<const carla::client::Actor> actor);
	osi3::BaseMoving* toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor);
	osi3::TrafficSign* toOSI(const carla::SharedPtr<const carla::client::TrafficSign> actor, const pugi::xml_document& xodr);
	std::vector<osi3::TrafficLight*> toOSI(const carla::SharedPtr<const carla::client::TrafficLight> actor, const pugi::xml_document& xodr);

	std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> toOSI(carla::client::Vehicle::LightState lightState);

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
	}
};