#include "MessageCache.h"

void MessageCache::setMessage(const std::string& key, const std::string& message, bool verbose) {
	if (verbose)
	{
		std::cout << "Set message cache for " << key << " Message size: " << message.size() << std::endl;
	}
	varName2MessageMap[key] = message;
}

std::string MessageCache::getMessage(const std::string& key, bool verbose) {
	auto iter = varName2MessageMap.find(key);
	if (iter != varName2MessageMap.end()) {
		if (verbose)
		{
			std::cout << "Get message cache for " << key << " Message size: " << iter->second.size() << std::endl;
		}
		return iter->second;
	}
	else {
		if (verbose)
		{
			std::cout << "Get message cache for " << key << "No generic message found." << std::endl;
		}
		return "";
	}
}

void MessageCache::setVehicleStates(const osi3::TrafficUpdate& message, bool verbose) {
	for (auto& state : message.internal_state()) {
		uint64_t key = state.host_vehicle_id().value();
		if (verbose)
		{
			std::cout << "Set vehicle states for " << key << std::endl;
		}
		vehicleInternalStateMap[key] = state;
	}
}

osi3::HostVehicleData MessageCache::getVehicleState(const uint64_t& key, bool verbose) {
	auto iter = vehicleInternalStateMap.find(key);
	if (iter != vehicleInternalStateMap.end()) {
		if (verbose)
		{
			std::cout << "Get vehicle states for " << key << std::endl;
		}
		return iter->second;
	}
	else {
		if (verbose)
		{
			std::cout << "Get vehicle states for " << key << " No host vehicle message found. Create default message" << std::endl;
		}
		osi3::HostVehicleData hvd;
		hvd.mutable_vehicle_powertrain()->set_pedal_position_acceleration(0);
		hvd.mutable_vehicle_brake_system()->set_pedal_position_brake(0);
		hvd.mutable_vehicle_motion()->mutable_velocity()->set_x(3);//3 m/s > 10 km/h Start
		hvd.mutable_vehicle_motion()->mutable_velocity()->set_z(0);
		hvd.mutable_vehicle_motion()->mutable_velocity()->set_y(0);
		return hvd;
	}
}
