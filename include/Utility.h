#ifndef CARLAUTILITY_H
#define CARLAUTILITY_H

#define _USE_MATH_DEFINES

#include <iterator>
#include <math.h>
#include <optional>

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/BoundingBox.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector3D.h>
#include <carla/geom/Vector2D.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>
#include <carla/rpc/VehicleLightState.h>

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

	// Coordinate system in Carla/UE4:	left-handed,	X->forward, rot-, Y->right, rot-,	Z->up, rot+
	// OSI/DIN ISO 8855:				right-handed,	X->forward, rot+, Y->left, rot+,	Z->up, rot+		(see also https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames#R6)

	osi3::Orientation3d* toOSI(carla::geom::Rotation& rotation);
	std::pair<osi3::Dimension3d*, osi3::Vector3d*> toOSI(carla::geom::BoundingBox& boundingBox);
	osi3::Vector3d* toOSI(carla::geom::Vector3D& location);
	//carla::geom::Vector3D is a generalization of carla::geom::Location
	//osi3::Vector3d toOSI(carla::geom::Location& location);
	osi3::Vector2d* toOSI(carla::geom::Vector2D& vector);

	carla::geom::Rotation toCarla(const osi3::Orientation3d* orientation);
	carla::geom::BoundingBox toCarla(const osi3::Dimension3d* dimension, const osi3::Vector3d* position);
	carla::geom::Location toCarla(const osi3::Vector3d* position);
	carla::geom::Vector2D toCarla(const osi3::Vector2d* vector);

	carla::geom::Vector3D toCarlaVector(const osi3::Vector3d* position);

	osi3::Identifier* toOSI(carla::ActorId actorID);
	carla::ActorId toCarla(const osi3::Identifier* id);

	osi3::StationaryObject* toOSIStationaryObject(carla::SharedPtr< carla::client::Actor> actor);
	osi3::TrafficSign* toOSI(carla::SharedPtr< carla::client::TrafficSign> actor, pugi::xml_document& xodr);
	std::vector<osi3::TrafficLight*> toOSI(carla::SharedPtr< carla::client::TrafficLight> actor, pugi::xml_document& xodr);

	carla::rpc::VehicleLightState::LightState toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState);

	osi3::CameraSensorView* toOSICamera(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData);
	osi3::LidarSensorView* toOSILidar(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData);
	osi3::RadarSensorView* toOSIRadar(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData);

	carla::SharedPtr<carla::client::Vehicle> getParentVehicle(carla::SharedPtr<carla::client::Actor> actor);

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

#endif !CARLAUTILITY_H
