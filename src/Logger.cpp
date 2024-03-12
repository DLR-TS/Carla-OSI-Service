#include "Logger.h"

void Logger::writeLog(std::shared_ptr<const osi3::GroundTruth> groundTruth) {

	std::string separator = ",";

	if (!logFile.is_open()) {
		logFile.open(runtimeParameter->logFileName);
		if (runtimeParameter->verbose) {
			std::cout << "Write to " << runtimeParameter->logFileName << std::endl;
		}
		std::string header = "Timestamp" + separator + "Actor_ID" + separator +
			"Actor_x" + separator + "Actor_y" + separator + "Actor_heading\n";
		logFile << header;
		std::cout << header;
	}

	double seconds = carla->world->GetSnapshot().GetTimestamp().elapsed_seconds;

	//log all vehicles
	logData logData;
	for (const auto& movingObject : groundTruth->moving_object()) {
		logData.id = std::to_string(movingObject.id().value());
		logData.x = movingObject.base().position().x();
		logData.y = movingObject.base().position().y();
		logData.yaw = movingObject.base().orientation().yaw() * 180 / M_PI;

		logFile << seconds << separator
			<< logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw << "\n";
		std::cout << seconds << separator
			<< logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw << "\n";
	}
	//end line and flush data
	logFile << std::flush;
	std::cout << std::flush;
}
