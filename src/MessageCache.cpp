#include "MessageCache.h"

void MessageCache::setMessage(const std::string& key, const std::string& message, bool verbose) {
	if (verbose)
	{
		std::cout << __FUNCTION__ << key << " Message size: " << message.size() << std::endl;
	}
	varName2MessageMap[key] = message;
}

std::string MessageCache::getMessage(const std::string& key, bool verbose) {
	auto iter = varName2MessageMap.find(key);
	if (iter != varName2MessageMap.end()) {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << " Message size: " << iter->second.size() << std::endl;
		}
		return iter->second;
	}
	else {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << "No generic message found." << std::endl;
		}
		return "";
	}
}

void MessageCache::setVehicleStates(const osi3::TrafficUpdate& message, bool verbose) {
	for (auto& state : message.internal_state()) {
		uint64_t key = state.host_vehicle_id().value();
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << std::endl;
		}
		vehicleInternalStateMap[key] = state;
	}
}

osi3::HostVehicleData MessageCache::getVehicleState(const uint64_t& key, bool verbose) {
	auto iter = vehicleInternalStateMap.find(key);
	if (iter != vehicleInternalStateMap.end()) {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << std::endl;
		}
		return iter->second;
	}
	else {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << "No host vehicle message found." << std::endl;
		}
		return osi3::HostVehicleData();
	}
}
