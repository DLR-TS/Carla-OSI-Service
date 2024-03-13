/**
@authors German Aerospace Center: Nils Wendorff, Björn Bahn, Danny Behnecke
*/

#ifndef CARLAOSIGRPC_H
#define CARLAOSIGRPC_H

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif

#include <thread>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpc_proto_files/base_interface/BaseInterface.grpc.pb.h>
#include <grpc_proto_files/base_interface/BaseInterface.pb.h>
#include <grpc_proto_files/base_interface/CARLAInterface.grpc.pb.h>
#include <grpc_proto_files/base_interface/CARLAInterface.pb.h>

#include "Semaphore.h"
#include "Logger.h"
#include "CARLA_TrafficUpdate.h"
#include "CARLA_SensorView.h"
#include "CARLA_TrafficCommand.h"
#include "CARLA_SensorViewConfiguration.h"
#include "carla_osi/Identifiers.h"
#include "ScenarioRunner/TrafficCommandReceiver.h"

// client accessing the CARLA server and grpc service/server for CoSiMa base interface
class CARLA_OSI_client : public CoSiMa::rpc::CARLAInterface::Service, public CoSiMa::rpc::BaseInterface::Service {

	const std::string serverAddress;

	//Scenario Runner Synchronisation
	Semaphore smphSignalCosimaToSR;
	Semaphore smphSignalSRToCosima;

	std::shared_ptr<RuntimeParameter> runtimeParameter = std::make_shared<RuntimeParameter>();

	//thread to release Carla from step mode in async mode if simulation stopped.
	std::unique_ptr<std::thread> watchdog_thread;

#pragma region fields for the grpc service
	std::shared_ptr<grpc::Server> server;
	const std::chrono::milliseconds transaction_timeout;
	std::unique_ptr<std::thread> server_thread;
#pragma endregion

#pragma region fields for the Carla OSI Interface
	std::shared_ptr<CARLAInterface> carla = std::make_shared<CARLAInterface>();
	std::unique_ptr<TrafficUpdater> trafficUpdater = std::make_unique<TrafficUpdater>();
	std::shared_ptr<SensorViewer> sensorViewer = std::make_shared<SensorViewer>();
	std::unique_ptr<SensorViewConfiger> sensorViewConfiger = std::make_unique<SensorViewConfiger>();
	std::unique_ptr<TrafficCommander> trafficCommander = std::make_unique<TrafficCommander>();
	std::unique_ptr<Logger> logger = std::make_unique<Logger>();

	// contains OSI messages (values) for variable names (keys). Can be used for output->input chaining without translating a message into Carla's world first if no corresponding role_name is present
	std::map<std::string, std::string> varName2MessageMap; //important!

#pragma endregion fields for the Carla OSI Interface

	carla::srunner::TrafficCommandReceiver trafficCommandReceiver;

	std::shared_ptr<osi3::TrafficCommand> trafficCommandForEgoVehicle;

public:

	CARLA_OSI_client(const std::string& server_address)
		: transaction_timeout(std::chrono::milliseconds(5000)),
		serverAddress(server_address),
		trafficCommandReceiver(std::bind(&CARLA_OSI_client::saveTrafficCommand, this, std::placeholders::_1)) {};

	CARLA_OSI_client(const std::string& server_address, const std::chrono::milliseconds transaction_timeout)
		: transaction_timeout(transaction_timeout),
		serverAddress(server_address),
		trafficCommandReceiver(std::bind(&CARLA_OSI_client::saveTrafficCommand, this, std::placeholders::_1)) {};

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
	virtual grpc::Status DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response) override;
	virtual grpc::Status GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::Bytes* response) override;
	virtual grpc::Status SetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::NamedBytes* request, CoSiMa::rpc::Int32* response) override;

private:

	virtual int deserializeAndSet(const std::string& base_name, const std::string& message);
	virtual std::string getAndSerialize(const std::string& base_name);

	// Callback function passed to TrafficCommandReceiver
	float saveTrafficCommand(const osi3::TrafficCommand& command);

	static void watchdog(CARLA_OSI_client* client);
	bool watchdogInitDone = false;
	bool watchdogDoStepCalled = true;
	int sensorCounter = 0;

	/**
	From FMU to internal sensor description format
	*/
	OSTARSensorConfiguration toSensorDescriptionInternal(osi3::SensorViewConfiguration& sensorViewConfiguration);

	/**
	From CoSiMa Configuration to internal sensor description format
	*/
	OSTARSensorConfiguration toSensorDescriptionInternal(const CoSiMa::rpc::OSISensorViewExtras& sensorViewConfiguration);
};

#endif //!CARLAOSIGRPC_H
