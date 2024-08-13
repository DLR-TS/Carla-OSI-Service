
/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef MESSAGECACHE_H
#define MESSAGECACHE_H

#include <string>
#include <map>
#include <iostream>
#include "osi_trafficupdate.pb.h"

//Contains OSI messages (values) for variable names (keys).
//Can be used for output->input chaining without translating a message into Carla's world first if no corresponding role_name is present

class MessageCache {
private:
	std::map<std::string, std::string> varName2MessageMap;
	std::map<uint64_t, osi3::HostVehicleData> vehicleInternalStateMap;
public:
	void setMessage(const std::string& key, const std::string& message, bool verbose);
	std::string getMessage(const std::string& key, bool verbose);
	void setVehicleStates(const osi3::TrafficUpdate& message, bool verbose);
	osi3::HostVehicleData getVehicleState(const uint64_t& key, bool verbose);
};

#endif //!PARAMETERDEFINITONS_H
