#pragma once

#include <functional>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

#include <google/protobuf/empty.pb.h>

#include "grpc_proto_files/srunner/ScenarioRunner.grpc.pb.h"
#include "grpc_proto_files/srunner/ScenarioRunner.pb.h"

namespace carla::srunner {
	typedef std::function<void(const osi3::TrafficCommand&)> TrafficCommandCallback;

	class TrafficCommandReceiver : public carla::grpc::srunner::OSIVehicleController::Service {

		TrafficCommandCallback _callback;

	public:
		TrafficCommandReceiver(TrafficCommandCallback callback) : _callback(callback) {}
		TrafficCommandReceiver() {}

		virtual void setCallback(TrafficCommandCallback callback);

		virtual ::grpc::Status SendCommand(::grpc::ServerContext* context, const osi3::TrafficCommand* command, google::protobuf::Empty* response) override;
	};
}