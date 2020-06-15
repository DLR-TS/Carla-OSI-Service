#ifndef OSIMESSAGES_H
#define OSIMESSAGES_H

#include <variant>
#include "osi_sensorview.pb.h"

enum eSupportedMessages {
	//Sensorview
	SensorViewMessage,
	GenericSensorViewMessage,
	RadarSensorViewMessage,
	LidarSensorViewMessage,
	CameraSensorViewMessage,
	UltrasonicSensorViewMessage
	
	//
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
	osi3::GenericSensorView,
	osi3::RadarSensorView,
	osi3::LidarSensorView,
	osi3::CameraSensorView,
	osi3::UltrasonicSensorView
> osiMessages_t;

#endif // !OSIMESSAGES_H
