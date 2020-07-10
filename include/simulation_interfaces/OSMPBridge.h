#ifndef OSMPBRIDGE_H
#define OSMPBRIDGE_H
#define NOMINMAX
#include "fmi4cpp/fmi4cpp.hpp"
#include "iSimulationData.h"
#include "../Mapper/Mapper.h"
#include "../mapper/OSIMapper.h"
#include "OSIMessages.h"
#include "OSIBridge.h"
#include "FMIBridge.h"

class OSMPBridge : public OSIBridge
{
public:
	OSMPBridge(std::shared_ptr<Mapper> mapper) : OSIBridge(mapper){};

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

private:
	//fmi4cpp::fmi4cppFMUstate state;
	std::unique_ptr<fmi4cpp::fmi2::cs_fmu> coSimFMU;
	std::shared_ptr<fmi4cpp::fmi2::cs_slave> coSimSlave;
};

#endif // !OSMPBRIDGE_H
