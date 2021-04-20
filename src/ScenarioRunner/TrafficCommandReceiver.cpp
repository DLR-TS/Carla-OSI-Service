#include "ScenarioRunner/TrafficCommandReceiver.h"

void carla::srunner::TrafficCommandReceiver::setCallback(TrafficCommandCallback callback)
{
	_callback = callback;
}

grpc::Status carla::srunner::TrafficCommandReceiver::SendCommand(::grpc::ServerContext * context, const osi3::TrafficCommand * command, google::protobuf::Empty * response)
{
	if (_callback) _callback(*command);
	return ::grpc::Status::OK;
}
