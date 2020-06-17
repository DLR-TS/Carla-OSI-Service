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
	The messages are stored as serialized strings with the interfacename pattern: <message>_<sub_message>_<messageindex>_<submessageindex>
	\param message The message to be stored.
	\param messageType messge type of variant message.
	\param index of list
	*/
	void mapOSIToInternalState(osiMessage_t message, eOSIMessage messageType, int index = 0);
	
	/**
	Map information form internalState to OSI message.
	\param messageType Messge type of message to be created.
	\return filled OSI message
	*/
	osiMessage_t mapFromInternalState(eOSIMessage messageType);//osiMessage_t
}; 

#endif // !OSIMAPPER_H
