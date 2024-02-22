/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef CARLAUTILITY_H
#define CARLAUTILITY_H

#include <unordered_set>

#include <carla/client/Sensor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/image/ImageView.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>
#include <carla/sensor/data/LidarData.h>

#include <osi_sensorview.pb.h>

#include <boost/gil.hpp>
#include <boost/gil/io/write_view.hpp>
#include <boost/gil/extension/io/png.hpp>

#include "carla_osi/TrafficSignals.h"

namespace CarlaUtility {

	osi3::StationaryObject* toOSI(const carla::rpc::EnvironmentObject& environmentObject, bool verbose);

	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Walker> walker);
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> vehicle);
	// common part of above methods
	std::unique_ptr<osi3::BaseMoving> toOSIBaseMoving_common(const carla::SharedPtr<const carla::client::Actor> actor, std::unique_ptr<osi3::BaseMoving> base);

	std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> toOSI(carla::client::Vehicle::LightState lightState);

	carla::rpc::VehicleLightState::LightState toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState);

	osi3::MovingObject_VehicleClassification_Type ParseVehicleType(const std::string& typeName);

	osi3::CameraSensorView* toOSICamera(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::data::Image> image);
	osi3::LidarSensorView* toOSILidar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData);
	osi3::RadarSensorView* toOSIRadar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData);

	carla::SharedPtr<carla::client::Vehicle> getParentVehicle(const carla::SharedPtr<const carla::client::Actor> actor);
	std::unique_ptr<osi3::Timestamp> parseTimestamp(const carla::client::Timestamp& carlaTime);

	std::string findBestMatchingCarToSpawn(const osi3::Dimension3d& dimension, const std::vector<std::tuple<std::string, carla::geom::Vector3D>>& replayVehicleBoundingBoxes,
		double& weightLength_X, double& weightWidth_Y, double& weightHeight_Z);

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
		return std::pair<AddIt, RemIt>(add_first, rem_first);
	};

};

#endif //!CARLAUTILITY_H
