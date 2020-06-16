#ifndef OSIMESSAGES_H
#define OSIMESSAGES_H

#include <variant>
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_groundtruth.pb.h"

/**
* Implemented OSI Messages
*/
enum eOSIMessage {
	//Sensorview
	SensorViewMessage,

	GenericSensorViewMessage,
	RadarSensorViewMessage,
	LidarSensorViewMessage,
	CameraSensorViewMessage,
	UltrasonicSensorViewMessage,
	
	//SensorViewConfiguration
	SensorViewConfigurationMessage,

	GenericSensorViewConfigurationMessage,
	RadarSensorViewConfigurationMessage,
	LidarSensorViewConfigurationMessage,
	CameraSensorViewConfigurationMessage,
	UltrasonicSensorViewConfigurationMessage,

	//GroundTruth
	GroundTruthMessage,

	StationaryObjectMessage,
	MovingObjectMessage,
	TrafficSignMessage,
	TrafficLightMessage,
	RoadMarkingMessage,
	LaneBoundaryMessage,
	LaneMessage,
	OccupantMessage,

	//TrafficCommand (SL45)
	SL45TrafficCommandMessage,

	//InVehicleSensorData (SL45?)
	SL45InVehicleSensorDataMessage,
};

/**
* Address struct and union to convert integer in pointer and vice versa. See OSI Sensor Model Packaging Specification 
*/
struct address {
	union pointerUnion {
		struct {
			int lo;
			int hi;
		} base;
		unsigned long long address;
	} addr;
	int size;
};

/**
* Type definition of std::variant collection of all implemented OSI messages.
*/
typedef std::variant<
	//SensorView
	osi3::SensorView,
	osi3::GenericSensorView,
	osi3::RadarSensorView,
	osi3::LidarSensorView,
	osi3::CameraSensorView,
	osi3::UltrasonicSensorView,

	//SensorViewConfiguration
	osi3::SensorViewConfiguration,
	osi3::GenericSensorViewConfiguration,
	osi3::RadarSensorViewConfiguration,
	osi3::LidarSensorViewConfiguration,
	osi3::CameraSensorViewConfiguration,
	osi3::UltrasonicSensorViewConfiguration,

	//GroundTruth
	osi3::GroundTruth,
	osi3::StationaryObject,
	osi3::MovingObject,
	osi3::TrafficSign,
	osi3::TrafficLight,
	osi3::RoadMarking,
	osi3::LaneBoundary,
	osi3::Lane,
	osi3::Occupant

	//TrafficCommand (SL45)

	//InVehicleSensorData (SL45?)

> osiMessage_t;

#endif // !OSIMESSAGES_H
