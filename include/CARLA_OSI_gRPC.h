/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef CARLAOSIGRPC_H
#define CARLAOSIGRPC_H

#include "CARLA2OSIInterface.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

#include "grpc_proto_files/base_interface/BaseInterface.grpc.pb.h"
#include "grpc_proto_files/base_interface/BaseInterface.pb.h"
#include "grpc_proto_files/base_interface/CARLAInterface.grpc.pb.h"
#include "grpc_proto_files/base_interface/CARLAInterface.pb.h"
#include "osi_common.pb.h"

#include "ScenarioRunner/TrafficCommandReceiver.h"

// client accessing the CARLA server and grpc service/server for CoSiMa base interface
class CARLA_OSI_client : public CoSiMa::rpc::CARLAInterface::Service, public CoSiMa::rpc::BaseInterface::Service {

	std::ofstream Logging;
	bool logEnabled = false;
	int logHeartbeatCounter = 0;
	int logHeartbeat = 0;

#pragma region fields for the grpc service
	std::shared_ptr<grpc::Server> server;
	const std::string server_address;
	const std::chrono::milliseconds transaction_timeout;
	std::unique_ptr<std::thread> server_thread;
#pragma endregion


#pragma region fields for the Carla OSI Interface
	CARLA2OSIInterface carlaInterface;
	// contains OSI messages (values) for variable names (keys). Can be used for output->input chaining without translating a message into Carla's world first if no corresponding role_name is present
	std::map<std::string, std::string> varName2MessageMap;
	// holds sensor position information for non-carla sensors. Maps prefixed_fmu_variable_name to mounting positions
	std::map < std::string, CoSiMa::rpc::SensorViewSensorMountingPosition, std::less<>> sensorMountingPositionMap;
	// ids for non-carla sensorViews
	std::map<std::string, uint64_t, std::less<>> sensorIds;
#pragma endregion fields for the Carla OSI Interface

	carla::srunner::TrafficCommandReceiver trafficCommandReceiver;

public:

	CARLA_OSI_client(const std::string& server_address)
		: server_address(server_address), transaction_timeout(std::chrono::milliseconds(5000)),
		trafficCommandReceiver(std::bind(&CARLA_OSI_client::serializeTrafficCommand, this, std::placeholders::_1)) {};

	CARLA_OSI_client(const std::string& server_address, const int heartbeatRate)
		: server_address(server_address), logHeartbeat(heartbeatRate), transaction_timeout(std::chrono::milliseconds(5000)),
		trafficCommandReceiver(std::bind(&CARLA_OSI_client::serializeTrafficCommand, this, std::placeholders::_1)) {};

	CARLA_OSI_client(const std::string& server_address, const std::chrono::milliseconds transaction_timeout)
		: server_address(server_address), transaction_timeout(transaction_timeout),
		trafficCommandReceiver(std::bind(&CARLA_OSI_client::serializeTrafficCommand, this, std::placeholders::_1)) {};

	~CARLA_OSI_client() {
		if (server)
			server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
		if (server_thread)
			server_thread->join();
	};

	virtual void StartServer(const bool nonBlocking = false);

	virtual void StopServer();

	virtual grpc::Status SetConfig(grpc::ServerContext* context, const CoSiMa::rpc::CarlaConfig* config, CoSiMa::rpc::Int32* response) override;

	// CARLA base interface service overrides
	//Only overriding Set/GetString and DoStep because other methods aren't supported by the Carla2OSI interface (yet?)
	virtual grpc::Status DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response) override;
	virtual grpc::Status GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::Bytes* response) override;
	virtual grpc::Status SetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::NamedBytes* request, CoSiMa::rpc::Int32* response) override;

private:
	// separate prefix, sourrounded by '#', from the given variable name
	virtual std::string_view getPrefix(const std::string_view base_name);
	// parse index from OSMP variable name, if present
	virtual uint32_t getIndex(const std::string_view osmp_name);

	virtual int deserializeAndSet(const std::string& base_name,const std::string& message);
	virtual std::string getAndSerialize(const std::string& base_name);

	// generate a SensorView that holds only ground truth. Can be used as input for osi3::SensorView generating OSI sensors;
	virtual std::shared_ptr<osi3::SensorView> getSensorViewGroundTruth(const std::string& name);
	static void copyMountingPositions(const CoSiMa::rpc::SensorViewSensorMountingPosition& from, std::shared_ptr<osi3::SensorView> to);

	// Serialize given trafficCommand into varName2MessageMap
	// Callback function passed to TrafficCommandReceiver
	void serializeTrafficCommand(const osi3::TrafficCommand& command);
};

#endif //!CARLAOSIGRPC_H
