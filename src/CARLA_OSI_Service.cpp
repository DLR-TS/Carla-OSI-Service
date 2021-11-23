#include "CARLA_OSI_gRPC.h"

bool isNumber(const std::string& str)
{
	for (char const &c : str) {
		if (std::isdigit(c) == 0) return false;
	}
	return true;
}

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client\n" <<
		"Compiled for Carla OSI Version 0.9.10\n" << std::endl;

	std::cout << "Current directory: " << std::filesystem::current_path() << "\n" << std::endl;
	//Server address deliberately chosen to accept any connection
	std::string server_address = "0.0.0.0:51425";
	int logHeartbeatRate = -1;//no logs
	bool debug = false;

	for (int i = 1; i < argc; i++) {
		if (isNumber(argv[i])) {
			logHeartbeatRate = std::stoi(argv[i]);
			std::cout << "Log just each " << logHeartbeatRate << " simulation step." << std::endl;
		}
		else if (std::string(argv[i]) == "-d") {
			debug = true;
			std::cout << "Running with additional debug prints." << std::endl;
		}
		else {
			server_address = argv[i];
			std::cout << "Server listens on: " << server_address << std::endl;
		}
	}
	if (logHeartbeatRate == -1) {
		std::cout << "no-log option enabled" << std::endl;
	}

	CARLA_OSI_client client(server_address, logHeartbeatRate, debug);

	client.StartServer();

	return 0;
}

