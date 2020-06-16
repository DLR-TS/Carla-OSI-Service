#ifndef OSIBRIDGE_H
#define OSIBRIDGE_H

#include <map>
#include <string>
#include "iSimulationData.h"
#include "../Mapper/Mapper.h"
#include "../mapper/OSIMapper.h"
#include "OSIMessages.h"
#include "osi_sensorview.pb.h"


class OSIBridge : iSimulationData
{
public:
	OSIBridge(std::shared_ptr<Mapper> mapper) : iSimulationData(mapper) {};

	int init(std::string scenario, float starttime, int mode) override;
	int connect(std::string) override;
	int disconnect() override;

	int writeToInternalState() override;
	int readFromInternalState() override;
	int doStep(double stepSize = 1) override;

private:
	osi3::SensorView sensorView;
	osi3::SensorViewConfiguration sensorViewConfiguration;
	osi3::GroundTruth groundTruth;
	/**
	* OSI message type and its location
	*/
	std::map<eOSIMessage, address> addressInformation;
};

#endif // !OSIBRIDGE_H
