#ifndef OSIBRIDGE_H
#define OSIBRIDGE_H

#include <map>
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
	osi3::GenericSensorView genericSensorView;
	osi3::RadarSensorView radarSensorView;
	osi3::LidarSensorView lidarSensorView;
	osi3::CameraSensorView cameraSensorView;
	osi3::UltrasonicSensorView ultrasonicSensorView;

	std::map<eSupportedMessages, address> addressInformation;
};

#endif // !OSIBRIDGE_H
