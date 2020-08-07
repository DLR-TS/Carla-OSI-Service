#include "simulation_interfaces/iSimulationData.h"

int iSimulationData::mapToInterfaceSystem(std::shared_ptr<BaseSystemInterface> baseInterface) {
	return mapper->searchInput(baseInterface);
}

int iSimulationData::mapFromInterfaceSystem(std::shared_ptr<BaseSystemInterface> baseInterface) {
	return mapper->writeOutput(baseInterface);
}

std::shared_ptr<Mapper> iSimulationData::getMapper() {
	return mapper;
}