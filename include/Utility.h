/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

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
#include "carla/image/ImageView.h"
#include <carla/road/RoadTypes.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"
#include "carla_osi/TrafficSignals.h"

#include "boost/gil.hpp"
#include "boost/gil/io/write_view.hpp"
#include "boost/gil/extension/io/png.hpp"

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
//#include "osi_logicaldetectiondata.pb.h"
//#include "osi_logicallane.pb.h"
//#include "osi_object.pb.h"
//#include "osi_occupant.pb.h"
//#include "osi_referenceline.pb.h"
#include "osi_roadmarking.pb.h"
#include "osi_sensordata.pb.h"
//#include "osi_sensorspecific.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
//#include "osi_trafficcommand.pb.h"
#include "osi_trafficlight.pb.h"
#include "osi_trafficsign.pb.h"
//#include "osi_trafficupdate.pb.h"
//#include "osi_version.pb.h"

namespace CarlaUtility {

	// Only specialized actors have a bounding box, therefore it has to be passed as additional argument
	osi3::StationaryObject* toOSI(const carla::SharedPtr<const carla::rpc::EnvironmentObject> environmentObject, const std::string& model_reference, bool verbose);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Walker> walker);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> vehicle);
	// common part of above methods
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving_common(const carla::SharedPtr<const carla::client::Actor> actor, std::unique_ptr<osi3::BaseMoving> base);

	std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> toOSI(carla::client::Vehicle::LightState lightState);

	carla::rpc::VehicleLightState::LightState toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState);

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

#endif //!CARLAUTILITY_H
