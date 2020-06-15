#ifndef OSIMESSAGES_H
#define OSIMESSAGES_H

#include <variant>
#include "osi_sensorview.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_groundtruth.pb.h"

enum eSupportedMessages {
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
	GroundTruthMessage
};

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

typedef std::variant<osi3::SensorView,
	//SensorView
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
	osi3::GroundTruth
> osiMessages_t;

#endif // !OSIMESSAGES_H
