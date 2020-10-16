#include "catch2/catch.hpp"

#include <chrono>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include "grpc_proto_files/base_interface/CARLAInterface.grpc.pb.h"
#include "grpc_proto_files/base_interface/CARLAInterface.pb.h"
#include "osi_sensorview.pb.h"

#include "CARLA_OSI_gRPC.h"

std::unique_ptr<grpc::ClientContext> CreateDeadlinedClientContext(double transactionTimeout) {
	// context to handle a rpc call - cannot be reused
	std::unique_ptr<grpc::ClientContext> context = std::make_unique<grpc::ClientContext>();
	// double to integer conversion
	std::chrono::duration timeout = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(transactionTimeout));
	// set deadline to transactionTimeout seconds from now
	context->set_deadline(std::chrono::system_clock::now() + timeout);
	return context;
}

TEST_CASE("CARLAInterface", "[CARLA_OSI_Client][CARLAInterface][.][RequiresCarlaServer]") {

	std::string gRPCHost = "localhost:51425";
	CoSiMa::rpc::CARLAInterface::Stub stub(grpc::CreateChannel(gRPCHost, grpc::InsecureChannelCredentials()));

	// client accessing the CARLA server and grpc service/server for CoSiMa base interface
	CARLA_OSI_client server(gRPCHost);

	const double transactionTimeout = 20;

	server.StartServer(true);

	SECTION("Supported rpcs") {
		const double deltaSeconds = 1 / 60.0;
		const uint16_t carlaPort = 2000u;
		const std::string carlaHost = "localhost";

		CoSiMa::rpc::CarlaConfig config;
		config.set_carla_host(carlaHost);
		config.set_carla_port(carlaPort);
		config.set_transaction_timeout(transactionTimeout);
		config.set_delta_seconds(deltaSeconds);
		auto response = CoSiMa::rpc::Int32();


		// timeout multiplied by 2 - parsing of stationary map objects takes some time
		auto status = stub.SetConfig(CreateDeadlinedClientContext(transactionTimeout * 2).get(), config, &response);

		REQUIRE(status.ok());
		REQUIRE(0 == response.value());

		osi3::SensorView someSensorView;
		std::string serializedData = someSensorView.SerializeAsString();
		std::string baseName = "#prefix#OSMPSensorViewOut";
		CoSiMa::rpc::NamedString namedString;
		namedString.set_name(baseName);
		namedString.set_value(serializedData);
		response.Clear();

		status = stub.SetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedString, &response);

		REQUIRE(status.ok());
		REQUIRE(0 == response.value());

		CoSiMa::rpc::String rpcBaseName, responseString;
		rpcBaseName.set_value(baseName);

		status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), rpcBaseName, &responseString);

		REQUIRE(status.ok());
		REQUIRE(0 == responseString.value().compare(serializedData));

		CoSiMa::rpc::Empty empty;
		CoSiMa::rpc::Double responseDouble;

		status = stub.DoStep(CreateDeadlinedClientContext(transactionTimeout).get(), empty, &responseDouble);

		REQUIRE(status.ok());
		REQUIRE(deltaSeconds == Approx(responseDouble.value()));
	}

	SECTION("Unsupported rpcs") {

		auto rpcBool = CoSiMa::rpc::Bool();
		auto namedBool = CoSiMa::rpc::NamedBool();
		auto rpcInt = CoSiMa::rpc::Int32();
		auto namedInt = CoSiMa::rpc::NamedInt32();
		auto rpcFloat = CoSiMa::rpc::Float();
		auto namedFloat = CoSiMa::rpc::NamedFloat();
		auto rpcDouble = CoSiMa::rpc::Double();
		auto namedDouble = CoSiMa::rpc::NamedDouble();

		auto request = CoSiMa::rpc::String();
		auto response = CoSiMa::rpc::Int32();


		CHECK_FALSE(stub.SetBoolValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedBool, &response).ok());
		CHECK_FALSE(stub.SetIntValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedInt, &response).ok());
		CHECK_FALSE(stub.SetFloatValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedFloat, &response).ok());
		CHECK_FALSE(stub.SetDoubleValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedDouble, &response).ok());

		CHECK_FALSE(stub.GetBoolValue(CreateDeadlinedClientContext(transactionTimeout).get(), request, &rpcBool).ok());
		CHECK_FALSE(stub.GetIntValue(CreateDeadlinedClientContext(transactionTimeout).get(), request, &rpcInt).ok());
		CHECK_FALSE(stub.GetFloatValue(CreateDeadlinedClientContext(transactionTimeout).get(), request, &rpcFloat).ok());
		CHECK_FALSE(stub.GetDoubleValue(CreateDeadlinedClientContext(transactionTimeout).get(), request, &rpcDouble).ok());
	}
}