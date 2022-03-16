#include "ScenarioRunner/TrafficCommandReceiver.h"

void carla::srunner::TrafficCommandReceiver::setCallback(TrafficCommandCallback callback)
{
	_callback = callback;
}

grpc::Status carla::srunner::TrafficCommandReceiver::SendCommand(::grpc::ServerContext * context, const osi3::TrafficCommand * command, ::srunner::osi::client::Float * response)
{
	if (_callback) {
		response->set_value(_callback(*command));
	}
	return ::grpc::Status::OK;
}
