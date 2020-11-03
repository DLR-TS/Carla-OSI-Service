#ifndef COSIMA_H
#define COSIMA_H

#include "CARLA2OSIInterface.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

#include "grpc_proto_files/base_interface/CARLAInterface.grpc.pb.h"
#include "grpc_proto_files/base_interface/CARLAInterface.pb.h"

// client accessing the CARLA server and grpc service/server for CoSiMa base interface
class CARLA_OSI_client : public CoSiMa::rpc::CARLAInterface::Service {

#pragma region fields for the grpc service
	std::shared_ptr<grpc::Server> server;
	const std::string server_address;
	const std::chrono::milliseconds transaction_timeout;
	std::unique_ptr<std::thread> server_thread;
	CARLA2OSIInterface carlaInterface;
#pragma endregion


	#pragma region fields for the Carla OSI Interface
	// contains OSI messages (values) for variable names (keys). Can be used for output->input chaining without translating a message into Carla's world first if no corresponding role_name is present
	std::map<std::string, std::string> varName2MessageMap;
	#pragma endregion fields for the Carla OSI Interface

public:

	CARLA_OSI_client(const std::string& server_address)
		: server_address(server_address), transaction_timeout(std::chrono::milliseconds(5000)) {};

	CARLA_OSI_client(const std::string& server_address, const std::chrono::milliseconds transaction_timeout)
		: server_address(server_address), transaction_timeout(transaction_timeout) {};

	~CARLA_OSI_client() {
		if (server)
			server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
		if (server_thread)
			server_thread->join();
	};

	virtual void StartServer(const bool nonBlocking = false);

	virtual void StopServer();

	virtual grpc::Status SetConfig(grpc::ServerContext* context, const CoSiMa::rpc::CarlaConfig* config, CoSiMa::rpc::Int32* response) override;

	//Only overriding Set/GetString and DoStep because other methods aren't supported by the Carla2OSI interface (yet?)
	virtual grpc::Status DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response) override;
	virtual grpc::Status GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::Bytes* response) override;
	virtual grpc::Status SetStringValue(grpc::ServerContext* context, const CoSiMa:: rpc::NamedBytes* request, CoSiMa::rpc::Int32* response) override;

private:
	// separate prefix, sourrounded by '#', from the given variable name
	virtual std::string_view getPrefix(std::string_view base_name);

	virtual int deserializeAndSet(std::string base_name, std::string message);
	virtual std::string getAndSerialize(std::string base_name);

	// generate a SensorView that holds only ground truth. Can be used as input for osi3::SensorView generating OSI sensors;
	virtual std::shared_ptr<osi3::SensorView> getSensorViewGroundTruth();
};

#endif // !COSIMA_H
