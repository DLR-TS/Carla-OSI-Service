#ifndef OSIMAPPER_H
#define OSIMAPPER_H

#include "Mapper.h"
#include "simulation_interfaces/OSIMessages.h"
#include "osi_sensorview.pb.h"

class OSIMapper : Mapper {

public:
	OSIMapper() : Mapper() {};
	int readConfiguration(configVariants_t config) override;

	/**
	Map an OSI message in to internalState.
	\param message The message to be stored.
	\param messageType messge type of variant message.
	*/
	void mapToInternalState(osiMessages_t message, eSupportedMessages messageType);
	
	/**
	Map information form internalState to OSI message.
	\param messageType Messge type of message to be created.
	\return filled OSI message
	*/
	osiMessages_t mapFromInternalState(eSupportedMessages messageType);//osiMessages_t
}; 

#endif // !OSIMAPPER_H
