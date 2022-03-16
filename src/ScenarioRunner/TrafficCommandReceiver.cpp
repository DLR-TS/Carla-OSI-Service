#include "ScenarioRunner/TrafficCommandReceiver.h"

void carla::srunner::TrafficCommandReceiver::setCallback(TrafficCommandCallback callback)
{
	_callback = callback;
}

grpc::Status carla::srunner::TrafficCommandReceiver::SendCommand(::grpc::ServerContext * context, const osi3::TrafficCommand * command, ::srunner::osi::client::Float * response)
{
	if (_callback) {
		float time_step = _callback(*command);
		std::cout << "Time step as response: " << time_step << std::endl;
		response->set_value(time_step);
	}
	return ::grpc::Status::OK;
}
