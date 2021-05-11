#include "CARLA_OSI_gRPC.h"

#include <limits.h>
#include "Utility.h"
#include "carla_osi/Identifiers.h"

void CARLA_OSI_client::StartServer(const bool nonBlocking)
{
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	grpc::ServerBuilder builder;
	builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
	builder.RegisterService(static_cast<CoSiMa::rpc::BaseInterface::Service*>(this));
	builder.RegisterService(static_cast<CoSiMa::rpc::CARLAInterface::Service*>(this));
	builder.RegisterService(&trafficCommandReceiver);
	// try to use unlimited message size
	builder.SetMaxMessageSize(INT_MAX);
	server = builder.BuildAndStart();
	std::cout << "Server listening on " << server_address << std::endl;
	if (!nonBlocking) {
		server->Wait();
	}
	else {
		server_thread = std::make_unique<std::thread>(&grpc::Server::Wait, server);
	}
}

void CARLA_OSI_client::StopServer()
{
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	if (server_thread)
		server_thread->join();
	server = nullptr;
	std::cout << "Server stopped" << std::endl;
}

grpc::Status CARLA_OSI_client::SetConfig(grpc::ServerContext * context, const CoSiMa::rpc::CarlaConfig * config, CoSiMa::rpc::Int32 * response)
{
	//TODO
	for (auto& sensorViewExtra : config->sensor_view_extras()) {
		CoSiMa::rpc::SensorViewSensorMountingPosition mountingPosition;
		mountingPosition.CopyFrom(sensorViewExtra.sensor_mounting_position());
		sensorMountingPositionMap.insert({ sensorViewExtra.prefixed_fmu_variable_name(), mountingPosition });
	}
	response->set_value(
		carlaInterface.initialise(config->carla_host(), config->carla_port(), config->transaction_timeout(), config->delta_seconds()));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::DoStep(grpc::ServerContext * context, const CoSiMa::rpc::Empty * request, CoSiMa::rpc::Double * response)
{
	response->set_value(carlaInterface.doStep());
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::GetStringValue(grpc::ServerContext * context, const CoSiMa::rpc::String * request, CoSiMa::rpc::Bytes * response)
{
	response->set_value(getAndSerialize(request->value()));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::SetStringValue(grpc::ServerContext * context, const CoSiMa::rpc::NamedBytes * request, CoSiMa::rpc::Int32 * response)
{
	response->set_value(deserializeAndSet(request->name(), request->value()));
	return grpc::Status::OK;
}

void CARLA_OSI_client::serializeTrafficCommand(const osi3::TrafficCommand & command)
{
	//command.traffic_participant_id() to role name and append to variable name
	std::string& role = carlaInterface.actorIdToRoleName(command.traffic_participant_id());
	varName2MessageMap["TrafficCommand{" + role + "}"] = command.SerializeAsString();
}

std::string_view CARLA_OSI_client::getPrefix(std::string_view name)
{
	// a prefix is surrounded by '#'
	if (2 < name.size() && '#' == name.front()) {
		std::string_view prefix = name.substr(1, name.find('#', 1) - 1);
		return prefix;
	}
	return std::string_view();
}

uint32_t CARLA_OSI_client::getIndex(const std::string_view osmp_name)
{
	if (']' == osmp_name.back()) {
		size_t index = osmp_name.rfind('[');
		//if brackets sourround something, try to parse as index
		if (std::string_view::npos != index && index < osmp_name.size() - 2) {
			auto chars = osmp_name.substr(index + 1, osmp_name.size() - 1);
			uint32_t value = 0;
			// parse as uint32_t
			auto result = std::from_chars(chars.data(), chars.data() + chars.size(), value);
			if (result.ec != std::errc::invalid_argument && result.ec != std::errc::result_out_of_range) {
				return value;
			}
		}
	}
	return 0;
}

int CARLA_OSI_client::deserializeAndSet(const std::string& base_name, const std::string& message) {
	auto prefix = getPrefix(base_name);
	if (0 < prefix.length() && 2 + prefix.length() == base_name.length()) {
		// variable has only a prefix and no name
		std::cerr << __FUNCTION__ << ": Tried to set a variable that has a prefix, but no name (name='" << base_name << "')." << std::endl;
		//TODO do we desire variables that have only a prefix and no name?
		return -2;
	}

	auto varName = std::string_view(&base_name.at(prefix.length() + 2));

	if (std::string::npos != varName.find("MotionCommand")) {
		// parse as MotionCommand and apply to ego vehicle
		setlevel4to5::MotionCommand motionCommand;
		if (!motionCommand.ParseFromString(message)) {
			std::cerr << "CARLA2OSIInterface::setStringValue: Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -322;
		}

		//TODO uncomment when receiveMotionCommand is fully implemented and remove error message
		//carlaInterface.receiveMotionCommand(motionCommand);
		std::cerr << "Implementation of MotionCommand is not finished yet" << std::endl;
	}
	else if (std::string::npos != varName.find("TrafficUpdate")) {
		// parse as TrafficUpdate and apply
		osi3::TrafficUpdate trafficUpdate;
		if (!trafficUpdate.ParseFromString(message)) {
			std::cerr << "CARLA2OSIInterface::setStringValue: Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -322;
		}

		carlaInterface.receiveTrafficUpdate(trafficUpdate);
	}
	else {
		//Cache unmapped messages so they can be retrieved as input
		//TODO how to map base_name for retrieval as input?
		varName2MessageMap[base_name] = message;
	}


	//TODO implement
	return 0;
}

std::string CARLA_OSI_client::getAndSerialize(const std::string& base_name) {
	auto prefix = getPrefix(base_name);
	auto varName = std::string_view(&base_name.at(prefix.length() + 2));
	std::shared_ptr<const grpc::protobuf::Message> message;

	if (0 < prefix.length() && 2 + prefix.length() == base_name.length()) {
		// variable has only a prefix and no name
		std::cerr << __FUNCTION__ << ": Tried to get a variable that has a prefix, but no name (name='" << base_name << "')." << std::endl;
		//TODO do we desire variables that have only a prefix and no name?
		//TODO return value or throw?
		return "-2";
	}

	//Test for a specific message type by name and try to retrieve it using the CARLA OSI interface
	if (std::string::npos != varName.rfind("OSMPSensorViewGroundTruth", 0)) {
		//OSMPSensorViewGroundTruth is not a OSMP variable prefix but used as a special name to retrieve a ground truth message as part of sensor view
		message = getSensorViewGroundTruth(base_name);
	}
	else if (std::string::npos != varName.rfind("OSMPSensorView", 0)) {
		// OSMPSensorViewIn
		message = carlaInterface.getSensorView(base_name);
	}
	else if (std::string::npos != varName.rfind("OSMPGroundTruth", 0)) {
		// OSMPGroundTruthInit
		message = carlaInterface.getLatestGroundTruth();
	}
	else if (std::string::npos != varName.rfind("OSMPTrafficCommandIn", 0)) {
		// OSMPTrafficCommandIn
		//TODO retrieve and serialize TrafficCommand when it is supported by CARLA2OSIInterface
		std::cerr << __FUNCTION__ << ": Requested unimplemented message of type OSMPTrafficCommand" << std::endl;
	}

	// if the CARLA OSI interface did provide a message, return its string serialization;
	if (message) {
		return message->SerializeAsString();
	}

	// Try lookup in variable cache, else return empty string
	auto iter = varName2MessageMap.find(base_name);
	if (iter != varName2MessageMap.end()) {
		return iter->second;
	}
	else {
		std::cerr << __FUNCTION__ << ": Could not find a variable named " << base_name << std::endl;
		return "";
	}
}

std::shared_ptr<osi3::SensorView> CARLA_OSI_client::getSensorViewGroundTruth(const std::string& varName) {
	// create empty sensor view
	auto sensorView = std::make_shared<osi3::SensorView>();
	// create empty ground truth as part of sensor view
	auto groundTruth = sensorView->mutable_global_ground_truth();
	// copy latest ground truth into previously created ground truth
	groundTruth->MergeFrom(*carlaInterface.getLatestGroundTruth());

	// if defined, set sensor mounting positions
	auto iter = sensorMountingPositionMap.find(varName);
	if (sensorMountingPositionMap.end() != iter) {
		copyMountingPositions(iter->second, sensorView);
	}

	// find or generate id for the named sensor
	auto id = sensorIds.find(varName);
	if (sensorIds.end() != id) {
		sensorView->mutable_sensor_id()->set_value(id->second);
	}
	else {
		carla_osi::id_mapping::IDUnion sensorId;
		//TODO make sure the type is not defiend in CarlaUtility::CarlaUniqueID_e
		sensorId.type = 100;
		sensorId.id = sensorIds.size() + 1;
		sensorIds[varName] = sensorId.value;
		sensorView->mutable_sensor_id()->set_value(sensorId.value);
	}

	return sensorView;
}

void CARLA_OSI_client::copyMountingPositions(const CoSiMa::rpc::SensorViewSensorMountingPosition& from, std::shared_ptr<osi3::SensorView> to)
{
	for (int i = 0; i < from.generic_sensor_mounting_position_size(); i++) {
		to->add_generic_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.generic_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.radar_sensor_mounting_position_size(); i++) {
		to->add_radar_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.radar_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.lidar_sensor_mounting_position_size(); i++) {
		to->add_lidar_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.lidar_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.camera_sensor_mounting_position_size(); i++) {
		to->add_camera_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.camera_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.ultrasonic_sensor_mounting_position_size(); i++) {
		to->add_ultrasonic_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.ultrasonic_sensor_mounting_position(i));
	}
}
