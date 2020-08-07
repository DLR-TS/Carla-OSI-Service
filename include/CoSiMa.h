#ifndef COSIMA_H
#define COSIMA_H

#include <iostream>
#include <vector>
#include <string>
#include "base_interfaces/BaseSystemInterface.h"
#include "base_interfaces/DominionInterface.h"
#include "simulation_interfaces/iSimulationData.h"
#include "configreader/YAMLConfigReader.h"
#include "SimulationInterfaceFactory.h"


void simulationLoop(std::vector<std::shared_ptr<iSimulationData>> &simulationInterfaces, std::shared_ptr <BaseSystemInterface> &baseSystem);

#endif // !COSIMA_H
