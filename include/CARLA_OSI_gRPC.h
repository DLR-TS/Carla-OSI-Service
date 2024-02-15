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

	RuntimeParameter runtimeParameter;

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

	/**
	From FMU to internal sensor description format
	*/
	Sensor toSensorDescriptionInternal(osi3::SensorViewConfiguration& sensorViewConfiguration) {
		runtimeParameter.carlaSensors = true;

		Sensor sensor;
		sensor.sensorViewConfiguration.CopyFrom(sensorViewConfiguration);
		sensor.id = sensorViewConfiguration.sensor_id().value();

		//save all mounting bositions in base. Only one sensor possible
		if (sensorViewConfiguration.generic_sensor_view_configuration_size()) {
			sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.generic_sensor_view_configuration()[0].mounting_position());
			runtimeParameter.carlasensortypes.emplace(GENERIC);
			sensor.type = GENERIC;
		} else if (sensorViewConfiguration.radar_sensor_view_configuration_size()) {
			sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.radar_sensor_view_configuration()[0].mounting_position());
			runtimeParameter.carlasensortypes.emplace(RADAR);
			sensor.type = RADAR;
		} else if (sensorViewConfiguration.lidar_sensor_view_configuration_size()) {
			sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.lidar_sensor_view_configuration()[0].mounting_position());
			runtimeParameter.carlasensortypes.emplace(LIDAR);
			sensor.type = LIDAR;
		} else if (sensorViewConfiguration.camera_sensor_view_configuration_size()) {
			sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.camera_sensor_view_configuration()[0].mounting_position());
			runtimeParameter.carlasensortypes.emplace(CAMERA);
			sensor.type = CAMERA;
		} else if (sensorViewConfiguration.ultrasonic_sensor_view_configuration_size()) {
			sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.ultrasonic_sensor_view_configuration()[0].mounting_position());
			runtimeParameter.carlasensortypes.emplace(ULTRASONIC);
			sensor.type = ULTRASONIC;
		}
        return sensor;
    }

	/**
	From CoSiMa Configuration to internal sensor description format
	*/
	Sensor toSensorDescriptionInternal(const CoSiMa::rpc::OSISensorViewExtras& sensorViewConfiguration) {
		runtimeParameter.carlaSensors = true;

		Sensor sensor;
		sensor.prefixed_fmu_variable_name = sensorViewConfiguration.prefixed_fmu_variable_name();
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.sensor_mounting_position());

		if (sensorViewConfiguration.sensor_type() == "generic") {
			sensor.type = GENERIC;
			runtimeParameter.carlasensortypes.emplace(GENERIC);
			sensor.sensorViewConfiguration.add_generic_sensor_view_configuration();
		}
		else if (sensorViewConfiguration.sensor_type() == "radar") {
			sensor.type = RADAR;
			runtimeParameter.carlasensortypes.emplace(RADAR);
			auto* radar = sensor.sensorViewConfiguration.add_radar_sensor_view_configuration();
			radar->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
			radar->set_field_of_view_vertical(sensorViewConfiguration.field_of_view_vertical());
			radar->set_emitter_frequency(sensorViewConfiguration.emitter_frequency());
		}
		else if (sensorViewConfiguration.sensor_type() == "lidar") {
			sensor.type = LIDAR;
			runtimeParameter.carlasensortypes.emplace(LIDAR);
			auto* lidar = sensor.sensorViewConfiguration.add_lidar_sensor_view_configuration();
			lidar->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
			lidar->set_field_of_view_vertical(sensorViewConfiguration.field_of_view_vertical());
			lidar->set_emitter_frequency(sensorViewConfiguration.emitter_frequency());
		}
		else if (sensorViewConfiguration.sensor_type() == "camera") {
			sensor.type = CAMERA;
			runtimeParameter.carlasensortypes.emplace(CAMERA);
			auto* camera = sensor.sensorViewConfiguration.add_camera_sensor_view_configuration();
			camera->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
			camera->set_number_of_pixels_horizontal(sensorViewConfiguration.number_of_pixels_horizontal());
			camera->set_number_of_pixels_vertical(sensorViewConfiguration.number_of_pixels_vertical());
			runtimeParameter.carlasensortypes.emplace(CAMERA);
		}
		else if (sensorViewConfiguration.sensor_type() == "ultrasonic") {
			sensor.type = ULTRASONIC;
			runtimeParameter.carlasensortypes.emplace(ULTRASONIC);
			sensor.sensorViewConfiguration.add_ultrasonic_sensor_view_configuration();
		}
		return sensor;
    }

};

#endif //!CARLAOSIGRPC_H
