#include "CARLA_OSI_gRPC.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client\n" <<
		"Compiled for Carla OSI Version 0.9.13\n" << std::endl;

	std::cout << "Current directory: " << std::filesystem::current_path() << "\n" << std::endl;
	RuntimeParameter runtimeParameter;
	bool cityObjectLabelFilterSet = false;

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
			std::cout << "Log to std::cout and " << runtimeParameter.logFileName << "\n";
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
		else if (parameter == "-carlaSensors") {
			runtimeParameter.carlaSensors = true;
			std::cout << "Use listeners on sensors spawned in Carla.\n";
		}
		else if (parameter == "-CityObjectLabel") {
			cityObjectLabelFilterSet = true;
			std::string cityObjectLabelFilter = std::string(argv[++i]);
			if (cityObjectLabelFilter.find("None") != std::string::npos) runtimeParameter.options.None = true;
			if (cityObjectLabelFilter.find("Buildings") != std::string::npos) runtimeParameter.options.Buildings = true;
			if (cityObjectLabelFilter.find("Fences") != std::string::npos) runtimeParameter.options.Fences = true;
			if (cityObjectLabelFilter.find("Other") != std::string::npos) runtimeParameter.options.Other = true;
			if (cityObjectLabelFilter.find("Poles") != std::string::npos) runtimeParameter.options.Poles = true;
			if (cityObjectLabelFilter.find("RoadLines") != std::string::npos) runtimeParameter.options.RoadLines = true;
			if (cityObjectLabelFilter.find("Roads") != std::string::npos) runtimeParameter.options.Roads = true;
			if (cityObjectLabelFilter.find("Sidewalks") != std::string::npos) runtimeParameter.options.Sidewalks = true;
			if (cityObjectLabelFilter.find("TrafficSigns") != std::string::npos) runtimeParameter.options.TrafficSigns = true;
			if (cityObjectLabelFilter.find("Vegetation") != std::string::npos) runtimeParameter.options.Vegetation = true;
			if (cityObjectLabelFilter.find("Walls") != std::string::npos) runtimeParameter.options.Walls = true;
			if (cityObjectLabelFilter.find("Ground") != std::string::npos) runtimeParameter.options.Ground = true;
			if (cityObjectLabelFilter.find("Bridge") != std::string::npos) runtimeParameter.options.Bridge = true;
			if (cityObjectLabelFilter.find("RailTrack") != std::string::npos) runtimeParameter.options.RailTrack = true;
			if (cityObjectLabelFilter.find("GuardRail") != std::string::npos) runtimeParameter.options.GuardRail = true;
			if (cityObjectLabelFilter.find("TrafficLight") != std::string::npos) runtimeParameter.options.TrafficLight = true;
			if (cityObjectLabelFilter.find("Static") != std::string::npos) runtimeParameter.options.Static = true;
			if (cityObjectLabelFilter.find("Water") != std::string::npos) runtimeParameter.options.Water = true;
			if (cityObjectLabelFilter.find("Terrain") != std::string::npos) runtimeParameter.options.Terrain = true;
			if (cityObjectLabelFilter.find("Any") != std::string::npos) runtimeParameter.options.Any = true;
		}
		else if (parameter == "-h" || parameter == "--help") {
			std::cout << "Normal options for Carla OSI Service:\n"
				<< "-async            : simulator runs asynchronous\n"
				<< "-d or -v          : verbose log\n"
				<< "-sr               : connection with scenario runner\n"
				<< "-log <path>       : log data in file: <path>\n"
				<< "-CityObjectLabel <labels>: all labels in one string\n"
				<< "<ip>:<port>       : listening ip range (see gRPC) and port\n\n" 
				<< "Experimental options:\n"
				<< "-filter <filter>  : filter static objects depending on name\n"
				<< "-resumeAfter <s>  : resume Carla asynchronous after s seconds (Anti-Freeze)\n"
				<< "-noStaticObjects  : sending ground truth messages only with dynamic objects\n"
				<< "-carlaSensors     : listen to sensors spawned in carla\n"
				<< "-dynamicTimestamps: dynamic timestamps for special real time mode" << std::endl;
			exit(0);
		}
		else {
			runtimeParameter.serverAddress = argv[i];
			std::cout << "Server listens on: " << runtimeParameter.serverAddress << "\n";
		}
	}

	//set options to any if no filter is set by user
	if (cityObjectLabelFilterSet) {
		std::cout << "CityObjectLabel Filter: Any\n";
		runtimeParameter.options.Any = true;
	}
	std::cout << std::endl;

	//simple check
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
