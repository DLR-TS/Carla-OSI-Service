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
	Write the log.
	*/
	void writeLog();

};

#endif //!CARLALOGGER_H
