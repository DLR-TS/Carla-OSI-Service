#include "CARLA_OSI_gRPC.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client\n" <<
		"Compiled for Carla Version 0.9.13\n" << std::endl;

	std::cout << "Current directory: " << fs::current_path() << "\n" << std::endl;

	//Server address deliberately chosen to accept any connection
	std::string server_address = "0.0.0.0:51425";
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		const std::string parameter = std::string(argv[i]);
		if (parameter == "-v") {
			verbose = true;
			std::cout << "Verbose messages enabled." << std::endl;
		} else if (parameter.find(':') == std::string::npos) {
			//listen to messages from all ips
			server_address = "0.0.0.0:" + parameter;
		} else {
			server_address = parameter;
		}
	}

	CARLA_OSI_client client(server_address, verbose);
	client.StartServer();

	return 0;
}
