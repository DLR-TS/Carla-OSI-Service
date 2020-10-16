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

	std::shared_ptr<grpc::Server> server;
	const std::string server_address;
	const std::chrono::milliseconds transaction_timeout;
	std::unique_ptr<std::thread> server_thread;
	CARLA2OSIInterface carlaInterface;

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

	void StartServer(const bool nonBlocking = false);

	void StopServer();

	virtual grpc::Status SetConfig(grpc::ServerContext* context, const CoSiMa::rpc::CarlaConfig* config, CoSiMa::rpc::Int32* response) override;

	//Only overriding Set/GetString and DoStep because other methods aren't supported by the Carla2OSI interface (yet?)
	virtual grpc::Status DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response) override;
	virtual grpc::Status GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::String* response) override;
	virtual grpc::Status SetStringValue(grpc::ServerContext* context, const CoSiMa:: rpc::NamedString* request, CoSiMa::rpc::Int32* response) override;


};

#endif // !COSIMA_H
