#ifndef OSIMESSAGES_H
#define OSIMESSAGES_H

#include <variant>
#include "osi_sensorview.pb.h"
#include "osi_sensordata.pb.h"
#include "osi_sensorviewconfiguration.pb.h"
#include "osi_groundtruth.pb.h"

/**
* Implemented OSI Messages
*/
enum eOSIMessage {

	SensorViewMessage,	
	SensorViewConfigurationMessage,
	SensorDataMessage,
	GroundTruthMessage,
	SL45TrafficCommandMessage,
	SL45InVehicleSensorDataMessage
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
	osi3::SensorView,
	osi3::SensorViewConfiguration,
	osi3::SensorData,
	osi3::GroundTruth
	//TrafficCommand (SL45)
	//InVehicleSensorData (SL45?)

> osiMessage_t;

#endif // !OSIMESSAGES_H
