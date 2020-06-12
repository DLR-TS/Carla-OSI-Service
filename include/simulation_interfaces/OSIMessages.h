#ifndef OSIMESSAGES_H
#define OSIMESSAGES_H

#include <variant>
#include "osi_sensorview.pb.h"

enum eSupportedMessages {
	SensorViewMessage,
	GenericSensorViewMessage

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

typedef std::variant<osi3::SensorView, osi3::GenericSensorView> osiMessages_t;

#endif // !OSIMESSAGES_H
