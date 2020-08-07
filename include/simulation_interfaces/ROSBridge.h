#ifndef ROSBRIDGE_H
#define ROSBRIDGE_H

#include "simulation_interfaces/iSimulationData.h"
#include "../mapper/Mapper.h"

class ROSBridge : iSimulationData
{
public:
	ROSBridge(std::shared_ptr<Mapper> mapper) : iSimulationData(mapper){};

	int init(std::string scenario, float starttime, int mode) override;
	int connect(std::string) override;
	int disconnect() override;

	int writeToInternalState() override;
	int readFromInternalState() override;
	int doStep(double stepSize = 1) override;


};

#endif // !ROSBRIDGE_H