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
			std::cout << "Running with additional debug prints.\n";
		}
		else if (std::string(argv[i]) == "-sr") {
			runtimeParameter.scenarioRunnerDoesTick = true;
			std::cout << "Wait for scenario runner connection.\n";
		}
		else if (parameter == "-async") {
			runtimeParameter.sync = false;
			std::cout << "Running in asychronous mode.\n";
		}
		else if (parameter == "-noStaticObjects") {
			runtimeParameter.staticObjectsInGroundTruthMessage = false;
			std::cout << "Does not send static objects in ground truth messages.\n";
		}
		else {
			runtimeParameter.serverAddress = argv[i];
			std::cout << "Server listens on: " << runtimeParameter.serverAddress << "\n";
		}
	}
	std::cout << std::endl;

	CARLA_OSI_client client(runtimeParameter);
	client.StartServer();

	return 0;
}

