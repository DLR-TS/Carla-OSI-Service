#include "catch2/catch.hpp"

#include <chrono>

#include <carla/client/BlueprintLibrary.h>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpc_proto_files/base_interface/CARLAInterface.grpc.pb.h>
#include <grpc_proto_files/base_interface/CARLAInterface.pb.h>

#include <osi_sensorview.pb.h>
#include <osi_trafficCommand.pb.h>

#include "testhelpers.h"

#include "CARLA_OSI_gRPC.h"
#include "carla_osi/Identifiers.h"
#include "ScenarioRunner/TrafficCommandReceiver.h"

std::unique_ptr<grpc::ClientContext> CreateDeadlinedClientContext(double transactionTimeout) {
	// context to handle a rpc call - cannot be reused
	std::unique_ptr<grpc::ClientContext> context = std::make_unique<grpc::ClientContext>();
	// double to integer conversion
	std::chrono::duration timeout = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(transactionTimeout));
	// set deadline to transactionTimeout seconds from now
	context->set_deadline(std::chrono::system_clock::now() + timeout);
	return context;
}

TEST_CASE("CARLA_OSI_Client", "[CARLA_OSI_Client][CARLAInterface][.][RequiresCarlaServer][gRPC]") {

	std::string gRPCHost = "localhost:51425";
	grpc::ChannelArguments channelArgs;
	// disable client message size limits
	channelArgs.SetMaxSendMessageSize(-1);
	channelArgs.SetMaxReceiveMessageSize(-1);
	auto channel = grpc::CreateCustomChannel(gRPCHost, grpc::InsecureChannelCredentials(), channelArgs);
	CoSiMa::rpc::BaseInterface::Stub stub(channel);
	CoSiMa::rpc::CARLAInterface::Stub carlaConfigStub(channel);

	// client accessing the CARLA server and grpc service/server for CoSiMa base interface
	CARLA_OSI_client server(gRPCHost);

	const double transactionTimeout = 25;

	server.StartServer(true);

	const double deltaSeconds = 1 / 60.0;
	const uint16_t carlaPort = 2000u;
	const std::string carlaHost = "localhost";
	std::string carlaMap = "Town01";

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(carlaHost, carlaPort, transactionTimeout, carlaMap);

	SECTION("Supported rpcs") {

		CoSiMa::rpc::CarlaConfig config;
		auto response = CoSiMa::rpc::Int32();


		// timeout multiplied by 2 - parsing of stationary map objects takes some time
		auto status = carlaConfigStub.SetConfig(CreateDeadlinedClientContext(transactionTimeout * 2).get(), config, &response);

		REQUIRE(status.ok());
		REQUIRE(0 == response.value());

		SECTION("No actors needed") {

			// simulate an unmapped sensor view input passed through this base interface
			osi3::SensorView someSensorView;
			std::string serializedData = someSensorView.SerializeAsString();
			std::string baseName = "#prefix#OSMPSensorViewOut";
			CoSiMa::rpc::NamedBytes namedString;
			namedString.set_name(baseName);
			namedString.set_value(serializedData);
			response.Clear();

			// write into cache
			status = stub.SetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), namedString, &response);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			REQUIRE(status.ok());
			REQUIRE(0 == response.value());

			// read previous message from cache
			CoSiMa::rpc::String rpcBaseName;
			CoSiMa::rpc::Bytes responseString;
			rpcBaseName.set_value(baseName);

			status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), rpcBaseName, &responseString);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			CHECK(status.ok());
			CHECK(0 == responseString.value().compare(serializedData));

			// check tick length
			CoSiMa::rpc::Empty empty;
			CoSiMa::rpc::Double responseDouble;

			status = stub.DoStep(CreateDeadlinedClientContext(transactionTimeout).get(), empty, &responseDouble);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			REQUIRE(status.ok());
			REQUIRE(deltaSeconds == Approx(responseDouble.value()));

			// request a ground truth message
			baseName = "#anotherPrefix#OSMPGroundTruthInit";
			rpcBaseName.set_value(baseName);
			responseString.Clear();

			status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), rpcBaseName, &responseString);

			if (!status.ok()) {
				std::cerr << status.error_message() << std::endl;
			}
			REQUIRE(status.ok());
			REQUIRE(0 < responseString.value().size());
		}
		SECTION("Spawn some actors to test functionality") {
			// Spawn a rgb camera sensor
			std::string role = "#arbitraryPrefix#OSMPSensorView";
			auto bpLibrary = world.GetBlueprintLibrary();
			auto iter = bpLibrary->Find("sensor.camera.rgb");
			auto cameraSensor = *iter;
			cameraSensor.SetAttribute("role_name", role);
			auto sensorLocation = carla::geom::Location(0, 0, 500);
			auto sensorRotation = carla::geom::Rotation(-90, 0, 0);
			world.SpawnActor(cameraSensor, carla::geom::Transform(sensorLocation, sensorRotation));
			CoSiMa::rpc::String rpcBaseName;
			CoSiMa::rpc::Bytes response;
			rpcBaseName.set_value(role);

			// no sensor output available yet, because the sensor had no time to tick
			auto status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), rpcBaseName, &response);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			CHECK(status.ok());
			CHECK(response.value().size() == 0);

			// perform tick to get sensor output
			CoSiMa::rpc::Empty empty;
			CoSiMa::rpc::Double responseDouble;

			status = stub.DoStep(CreateDeadlinedClientContext(transactionTimeout).get(), empty, &responseDouble);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			REQUIRE(status.ok());
			REQUIRE(deltaSeconds == Approx(responseDouble.value()));

			// get sensor output
			response.Clear();
			status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout * 1000).get(), rpcBaseName, &response);

			if (!status.ok()) {
				std::cout << status.error_message() << std::endl;
			}
			CHECK(status.ok());
			CHECK(response.value().size() >= 0);
		}
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

TEST_CASE("CARLA_OSI_Client SensorView MountingPosition", "[CARLA_OSI_Client][CARLAInterface][.][RequiresCarlaServer][gRPC][MountingPosition]") {
	// mock a sensorView with SensorView input and preconfigured mounting position

	std::string gRPCHost = "localhost:51425";
	grpc::ChannelArguments channelArgs;
	// disable client message size limits
	channelArgs.SetMaxSendMessageSize(-1);
	channelArgs.SetMaxReceiveMessageSize(-1);
	auto channel = grpc::CreateCustomChannel(gRPCHost, grpc::InsecureChannelCredentials(), channelArgs);
	CoSiMa::rpc::BaseInterface::Stub stub(channel);
	CoSiMa::rpc::CARLAInterface::Stub carlaConfigStub(channel);

	// client accessing the CARLA server and grpc service/server for CoSiMa base interface
	CARLA_OSI_client server(gRPCHost);

	const double transactionTimeout = 2500;

	server.StartServer(true);

	const double deltaSeconds = 1 / 60.0;
	const uint16_t carlaPort = 2000u;
	const std::string carlaHost = "localhost";
	std::string carlaMap = "Town01";

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(carlaHost, carlaPort, transactionTimeout, carlaMap);

	//prefixed fmu SensorView variable name
	std::string baseName = "#prefix#OSMPSensorViewGroundTruth";

	CoSiMa::rpc::CarlaConfig config;
	auto extras = config.add_sensor_view_extras();
	extras->set_prefixed_fmu_variable_name(baseName);
	auto mountingPosition = extras->mutable_sensor_mounting_position();
	auto position = mountingPosition->mutable_position();
	position->set_x(1.2);
	position->set_y(-0.34);
	position->set_z(.56);
	auto response = CoSiMa::rpc::Int32();


	// timeout multiplied by 2 - parsing of stationary map objects takes some time
	auto status = carlaConfigStub.SetConfig(CreateDeadlinedClientContext(transactionTimeout * 2).get(), config, &response);

	REQUIRE(status.ok());
	REQUIRE(0 == response.value());

	CoSiMa::rpc::String rpcBaseName;
	rpcBaseName.set_value(baseName);
	CoSiMa::rpc::Bytes serializedResponse;

	status = stub.GetStringValue(CreateDeadlinedClientContext(transactionTimeout).get(), rpcBaseName, &serializedResponse);

	REQUIRE(status.ok());
	REQUIRE(0 < serializedResponse.value().size());

	osi3::SensorView sensorViewGroundTruth;
	sensorViewGroundTruth.ParseFromString(serializedResponse.value());

	CHECK(sensorViewGroundTruth.has_sensor_id());
	CHECK(0 < sensorViewGroundTruth.sensor_id().value());
	REQUIRE(1 == sensorViewGroundTruth.generic_sensor_view_size());
	REQUIRE(position->x() == sensorViewGroundTruth.generic_sensor_view(0).view_configuration().mounting_position().position().x());
	REQUIRE(position->y() == sensorViewGroundTruth.generic_sensor_view(0).view_configuration().mounting_position().position().y());
	REQUIRE(position->z() == sensorViewGroundTruth.generic_sensor_view(0).view_configuration().mounting_position().position().z());
}
/*
TEST_CASE("CARLA_OSI_CLIENT TrafficCommand proxy", "[CARLA_OSI_CLIENT][TrafficCommandReceiver]") {
	std::string gRPCHost = "localhost:51425";
	grpc::ChannelArguments channelArgs;
	auto channel = grpc::CreateCustomChannel(gRPCHost, grpc::InsecureChannelCredentials(), channelArgs);
	::srunner::osi::client::OSIVehicleController::Stub stub(channel);
	CoSiMa::rpc::BaseInterface::Stub baseStub(channel);

	osi3::TrafficCommand trafficCommand;
	trafficCommand.mutable_traffic_participant_id()->set_value(0);
	srunner::osi::client::Float floatMessage;

	// client accessing the CARLA server and grpc service/server for CoSiMa base interface
	CARLA_OSI_client server(gRPCHost);

	server.StartServer(true);

	auto context = CreateDeadlinedClientContext(5.0);
	stub.SendCommand(context.get(), trafficCommand, &floatMessage);

	// Unmatched actor id resolves to empty role name
	CoSiMa::rpc::String rpcBaseName;
	rpcBaseName.set_value("TrafficCommand");
	CoSiMa::rpc::Bytes serializedResponse;
	osi3::TrafficCommand response;

	context = CreateDeadlinedClientContext(5.0);
	baseStub.GetStringValue(context.get(), rpcBaseName, &serializedResponse);
	REQUIRE(0 == serializedResponse.value().length());
	REQUIRE(0 == serializedResponse.ByteSizeLong());
}

TEST_CASE("CARLA_OSI_CLIENT TrafficCommand receiver 2", "[CARLA_OSI_CLIENT][TrafficCommandReceiver][.][RequiresCarlaServer][gRPC]") {
	std::string gRPCHost = "localhost:51425";
	grpc::ChannelArguments channelArgs;
	auto channel = grpc::CreateCustomChannel(gRPCHost, grpc::InsecureChannelCredentials(), channelArgs);
	::srunner::osi::client::OSIVehicleController::Stub osiStub(channel);
	CoSiMa::rpc::BaseInterface::Stub baseStub(channel);
	CoSiMa::rpc::CARLAInterface::Stub carlaStub(channel);

	osi3::TrafficCommand trafficCommand;
	trafficCommand.mutable_traffic_participant_id()->set_value(0);
	srunner::osi::client::Float floatMessage;

	// client accessing the CARLA server and grpc service/server for CoSiMa base interface
	CARLA_OSI_client server(gRPCHost);

	const double deltaSeconds = 1 / 60.0;
	const uint16_t carlaPort = 2000u;
	const std::string carlaHost = "localhost";
	std::string carlaMap = "Town10HD";
	const double transactionTimeout = 60;
	CoSiMa::rpc::CarlaConfig conf;

	//Use one of the predefined maps as OpenDRIVE based maps can cause crashes if a road has no predecessor/successor
	auto[client, world] = getCarlaDefaultWorld(carlaHost, carlaPort, transactionTimeout, carlaMap);

	auto recommendedSpawnPoints = world.GetMap()->GetRecommendedSpawnPoints();
	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto vehicleBlueprints = blueprintLibrary->Filter("vehicle.*");
	auto vehicle = vehicleBlueprints->at(0);
	std::string role = "TrafficCommand test target";
	vehicle.SetAttribute("role_name", role);
	
	carla::SharedPtr<carla::client::Actor> actor = world.SpawnActor(vehicle, recommendedSpawnPoints.at(0));

	auto attributes = actor->GetAttributes();
	auto it = std::find_if(attributes.begin(), attributes.end(), [](auto attr) {return "role_name" == attr.GetId(); });
	CHECK(it != attributes.end());
	CHECK(it->GetValue() == role);

	server.StartServer(true);

	CoSiMa::rpc::Int32 result;
	auto context = CreateDeadlinedClientContext(transactionTimeout);
	auto status = carlaStub.SetConfig(context.get(), conf, &result);
	CHECK(0 == result.value());
	CHECK(status.ok());
	
	context = CreateDeadlinedClientContext(transactionTimeout);
	status = osiStub.SendCommand(context.get(), trafficCommand, &floatMessage);
	REQUIRE(status.error_code() == grpc::StatusCode::OK);

	// Unmatched actor id resolves to empty role name
	CoSiMa::rpc::String rpcBaseName;
	rpcBaseName.set_value("TrafficCommand");
	CoSiMa::rpc::Bytes serializedResponse;
	osi3::TrafficCommand response;

	context = CreateDeadlinedClientContext(transactionTimeout);
	status = baseStub.GetStringValue(context.get(), rpcBaseName, &serializedResponse);
	REQUIRE(status.error_code() == grpc::StatusCode::OK);
	REQUIRE(0 < serializedResponse.value().length());
	REQUIRE(0 < serializedResponse.ByteSizeLong());
	REQUIRE(response.ParseFromString(serializedResponse.value()));
	REQUIRE(0 == response.traffic_participant_id().value());

	trafficCommand.Clear();
	trafficCommand.set_allocated_traffic_participant_id(carla_osi::id_mapping::getOSIActorId(actor).release());
	rpcBaseName.set_value("TrafficCommand{" + role + "}");
	serializedResponse.Clear();

	context = CreateDeadlinedClientContext(transactionTimeout);
	status = osiStub.SendCommand(context.get(), trafficCommand, &floatMessage);
	REQUIRE(status.error_code() == grpc::StatusCode::OK);
	
	context = CreateDeadlinedClientContext(transactionTimeout);
	status = baseStub.GetStringValue(context.get(), rpcBaseName, &serializedResponse);
	REQUIRE(status.error_code() == grpc::StatusCode::OK);
	REQUIRE(0 < serializedResponse.ByteSizeLong());
	REQUIRE(0 < serializedResponse.value().length());
	REQUIRE(response.ParseFromString(serializedResponse.value()));
	REQUIRE(trafficCommand.traffic_participant_id().value() == response.traffic_participant_id().value());
}
*/
