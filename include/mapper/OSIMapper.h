#ifndef OSIMAPPER_H
#define OSIMAPPER_H

#include "Mapper.h"
#include "open-simulation-interface/build/osi_environment.pb.h"

class OSIMapper : Mapper {

public:
	OSIMapper() : Mapper() {};
	int readConfiguration(configVariants_t config) override;

};

#endif // !OSIMAPPER_H
