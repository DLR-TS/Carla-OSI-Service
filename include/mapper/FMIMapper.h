#ifndef FMIMAPPER_H
#define FMIMAPPER_H

#include "Mapper.h"
#include "../simulation_interfaces/FMIBridge.h"
//windef.h compatibility
#define NOMINMAX
#include "fmi4cpp/fmi4cpp.hpp"

class FMIMapper : Mapper {

	friend class FMIBridge;
public:
	FMIMapper() : Mapper() {};
	int readConfiguration(configVariants_t config) override;

	void setConfiguration(/*FMIConfig*/);

};

#endif // !FMIMAPPER_H
