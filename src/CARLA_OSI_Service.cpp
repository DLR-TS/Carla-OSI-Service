#include "CARLA_OSI_gRPC.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client\n" <<
		"Compiled for Carla OSI Version 0.9.10\n" << std::endl;

	std::cout << "Current directory: " << std::filesystem::current_path() << "\n" << std::endl;
	RuntimeParameter runtimeParameter;

	for (int i = 1; i < argc; i++) {
		std::string parameter = std::string(argv[i]);
		if (parameter == "-d" || parameter == "-v") {
			runtimeParameter.verbose = true;
			std::cout << "Running with additional debug prints." << std::endl;
		}
		else if (parameter == "-async") {
			runtimeParameter.sync = false;
			std::cout << "Running in asychronous mode." << std::endl;
		}
		else {
			runtimeParameter.serverAddress = argv[i];
			std::cout << "Server listens on: " << runtimeParameter.serverAddress << std::endl;
		}
	}

	CARLA_OSI_client client(runtimeParameter);

	client.StartServer();

	return 0;
}

