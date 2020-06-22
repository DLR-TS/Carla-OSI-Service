#ifndef OSMPBRIDGE_H
#define OSMPBRIDGE_H
#include "iSimulationData.h"
#include "../Mapper/Mapper.h"
#include "../mapper/OSIMapper.h"
#include "OSIMessages.h"
#include "OSIBridge.h"
#include "osi_sensorview.pb.h"


class OSMPBridge : public OSIBridge
{
public:
	OSMPBridge(std::shared_ptr<Mapper> mapper) : OSIBridge(mapper) {};

	int readOSMP(int hi, int lo, int size, eOSIMessage messageType);

	//int init(std::string scenario, float starttime, int mode) override;
	//int connect(std::string) override;
	//int disconnect() override;

};

#endif // !OSMPBRIDGE_H
