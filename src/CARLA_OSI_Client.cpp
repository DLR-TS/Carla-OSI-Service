#include "CARLA_OSI_gRPC.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client" << std::endl << std::endl;

	std::cout << std::filesystem::current_path() << std::endl << std::endl;

	std::string server_address = "0.0.0.0:51425";
	int logHeartbeatRate = 0;//each simulation step
	if (1 < argc) {
		server_address = argv[1];
	}
	if (2 < argc) {
		logHeartbeatRate = std::stoi(argv[2]);
		std::cout << "Log just each " << logHeartbeatRate << " simulation step." << std::endl;
	}

	CARLA_OSI_client client(server_address, logHeartbeatRate);

	client.StartServer();

	return 0;
}
