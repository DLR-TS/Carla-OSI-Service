#ifndef OSIMAPPER_H
#define OSIMAPPER_H

#include "Mapper.h"
#include "simulation_interfaces/OSIMessages.h"

class OSIMapper : Mapper {

public:
	OSIMapper() : Mapper() {};
	int readConfiguration(configVariants_t config) override;

	/**
	Map an OSI message in to internalState.
	The messages are stored as serialized strings with the interfacename pattern: <message>_<sub_message>_<messageindex>_<submessageindex>
	\param message The message as string to be stored.
	\param messageType messge type of variant message.
	\param index of list
	*/
	void mapOSIToInternalState(std::string message, eOSIMessage messageType);
	
	/**
	Map information form internalState to OSI message.
	\param messageType Messge type of message to be created.
	\return OSI message as string
	*/
	std::string mapOSIFromInternalState(eOSIMessage messageType);

	/** Prefix string for cosima to evaluate content as an OSI Message
	*/
	std::string prefix = "#";
}; 

#endif // !OSIMAPPER_H
