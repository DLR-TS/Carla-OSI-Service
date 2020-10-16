#include "CARLA_OSI_gRPC.h"

void CARLA_OSI_client::StartServer(const bool nonBlocking)
{
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	grpc::ServerBuilder builder;
	builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
	builder.RegisterService(this);
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
	response->set_value(
		carlaInterface.initialise(config->carla_host(), config->carla_port(), config->transaction_timeout(), config->delta_seconds()));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::DoStep(grpc::ServerContext * context, const CoSiMa::rpc::Empty * request, CoSiMa::rpc::Double * response)
{
	response->set_value(carlaInterface.doStep());
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::GetStringValue(grpc::ServerContext * context, const CoSiMa::rpc::String * request, CoSiMa::rpc::String * response)
{
	response->set_value(carlaInterface.getStringValue(request->value()));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::SetStringValue(grpc::ServerContext * context, const CoSiMa::rpc::NamedString * request, CoSiMa::rpc::Int32 * response)
{
	response->set_value(carlaInterface.setStringValue(request->name(), request->value()));
	return grpc::Status::OK;
}
