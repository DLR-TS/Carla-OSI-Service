/**
@authors German Aerospace Center: Björn Bahn
*/

#ifndef CARLALOGGER_H
#define CARLALOGGER_H

#include <fstream>

#include "CARLA_Module.h"

class Logger : public CARLAModule {

	std::ofstream logFile;
	struct logData { std::string id; double x, y, yaw; };

public:
	/**
	Write the log with given ground truth data.
	*/
	void writeLog(std::shared_ptr<const osi3::GroundTruth> groundTruth);

};

#endif //!CARLALOGGER_H
