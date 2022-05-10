/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef TRAFFICSIGNALS_H
#define TRAFFICSIGNALS_H

#include <carla/client/Map.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/TrafficLight.h>
#include <carla/geom/BoundingBox.h>
#include <carla/rpc/TrafficLightHead.h>

#include <osi_common.pb.h>
#include <osi_trafficlight.pb.h>
#include <osi_trafficsign.pb.h>

namespace carla_osi::traffic_signals {

	/// Translate given carla traffic sign actor to a corresponding osi traffic sign.
	std::unique_ptr<osi3::TrafficSign> getOSITrafficSign(const carla::SharedPtr<const carla::client::TrafficSign> actor,
		//const carla::SharedPtr<const carla::client::Map> map,//map is used to determine affected lanes
		const carla::geom::BoundingBox& bbox/*, const pugi::xml_document& xodr*/);
	
	/// Translate given carla traffic light actor to the corresponding list of osi traffic lights. 
	/// A traffic light in carla describes traffic light heads, while the osi structure describes only a bulb of a
	/// traffic light head, thus needing a set of traffic light structures to describe a traffic light head. 
	/// E.g. a common traffic light head with red, amber and green lights has to be described with 3 osi traffic lights
	/// This function returs hardcoded positions based on the default open drive traffic light used by CARLA
	std::vector<std::unique_ptr<osi3::TrafficLight>> getOSITrafficLight(const carla::SharedPtr<const carla::client::TrafficLight> actor/*,
		const carla::geom::BoundingBox& bbox*//*, const pugi::xml_document& xodr*/);

	/// Translate given carla traffic light actor to the corresponding list of osi traffic lights. 
	/// A traffic light in carla describes traffic light heads, while the osi structure describes only a bulb of a
	/// traffic light head, thus needing a set of traffic light structures to describe a traffic light head. 
	/// E.g. a common traffic light head with red, amber and green lights has to be described with 3 osi traffic lights
	std::vector<std::unique_ptr<osi3::TrafficLight>> getOSITrafficLight(const carla::SharedPtr<const carla::client::TrafficLight> actor,
		const std::vector<carla::rpc::TrafficLightHeads> heads);
}

#endif //!TRAFFICSIGNALS_H
