#ifndef OSMPBRIDGE_H
#define OSMPBRIDGE_H
#include "iSimulationData.h"
#include "../Mapper/Mapper.h"
#include "../mapper/OSIMapper.h"
#include "OSIMessages.h"
#include "OSIBridge.h"
#include "FMIBridge.h"

class OSMPBridge : public OSIBridge, FMIBridge
{
public:
	OSMPBridge(std::shared_ptr<Mapper> mapper) : OSIBridge(mapper), FMIBridge(mapper){};

	int init(std::string scenario, float starttime, int mode) override;
	int connect(std::string) override;
	int disconnect() override;

	int writeToInternalState() override;
	int readFromInternalState() override;
	int doStep(double stepSize = 1) override;

	/**
	save the values together in an address map
	*/
	void saveToAddressMap(std::string name, int value);

	/**
	Parse string value to OSI Message Enum.
	\param std::string messageType string to parse into its enum value representation
	\return messagetype enum value of given string
	*/
	eOSIMessage getMessageType(std::string messageType);

	/**
	Temporary storage for osmp messages (name, size, address)
	*/
	std::map<std::string, address> addresses;
};

#endif // !OSMPBRIDGE_H
