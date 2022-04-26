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
		else if (parameter == "-sr") {
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
		else if (parameter == "-l" || parameter == "-log") {
			runtimeParameter.log = true;
			runtimeParameter.logFileName = std::string(argv[++i]);
			std::cout << "Log to std::cout and " << runtimeParameter.logFileName << ".csv\n";
		}
		else if (parameter == "-dynamicTimestamps") {
			runtimeParameter.dynamicTimestamps = true;
			std::cout << "Set timestamp accordingly to realtime."
				<< "The step size differs accordingly to calculation times of the connected models."
				<< "Only use this option with sync.\n";
		}
		else if (parameter == "-filter") {
			runtimeParameter.filter = true;
			runtimeParameter.filterString = std::string(argv[++i]);
			std::cout << "Filter for static objects active. Use: " << runtimeParameter.filterString << "\n";
		}
		else if (parameter == "-resumeAfter") {
			runtimeParameter.resumeCarlaAsyncSeconds = atoi(argv[++i]);
			std::cout << "Resume Carla (Anti - Freeze) after seconds: " << runtimeParameter.resumeCarlaAsyncSeconds << "\n";
		}
		else if (parameter == "-h" || parameter == "--help") {
			std::cout << "Normal options for Carla OSI Service:\n"
				<< "-async            : simulator runs asynchronous\n"
				<< "-d or -v          : verbose log\n"
				<< "-sr               : connection with scenario runner\n"
				<< "-log <path>       : log <path>.csv\n"
				<< "<ip>:<port>       : listening ip range (see gRPC) and port\n\n" 
				<< "Experimental options:\n"
				<< "-filter <filter>  : filter static objects depending on name\n"
				<< "-resumeAfter <s>  : resume Carla asynchronous after s seconds (Anti-Freeze)\n"
				<< "-noStaticObjects  : sending ground truth messages only with dynamic objects\n"
				<< "-dynamicTimestamps: dynamic timestamps for special real time mode" << std::endl;
			exit(0);
		}
		else {
			runtimeParameter.serverAddress = argv[i];
			std::cout << "Server listens on: " << runtimeParameter.serverAddress << "\n";
		}
	}
	std::cout << std::endl;

	if (runtimeParameter.dynamicTimestamps && !runtimeParameter.sync) {
		std::cout << "Invalid parameter combination. Can not run asynchron with dynamic step sizes.\n"
			<< "Dynamic step sizes option shall be used if connected models work in realtime.\n"
			<< "The main simulation shall compute accordingly to the elapsed time." << std::endl;
		exit(0);
	}

	CARLA_OSI_client client(runtimeParameter);
	client.StartServer();

	return 0;
}
