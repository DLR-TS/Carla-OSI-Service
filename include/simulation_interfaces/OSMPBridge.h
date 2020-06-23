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

	/**
	read OSI Message from interface
	\param int size size of osi messge array
	\param int lo lo value
	\param int hi hi value
	\param std::string messageType Message Type for the given address
	\return success status
	*/
	int readOSMP(int hi, int lo, int size, std::string messageType);
	/**
	write OSI Message to interface
	\param int& size size of osi messge array
	\param int& lo lo value
	\param int& hi hi value
	\param std::string messageType Message Type for the returned address
	\return success status
	*/
	int writeOSMP(int& hi, int& lo, int& size, std::string messageType);

private:
	/**
	Parse string value to OSI Message Enum.
	\param std::string messageType string to parse into its enum value representation
	\return messagetype enum value of given string
	*/
	eOSIMessage getMessageType(std::string messageType);
};

#endif // !OSMPBRIDGE_H
