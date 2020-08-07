#include "catch2/catch.hpp"

#include "SimulationInterfaceFactory.h"

TEST_CASE("SimulationInterfaceFactory") {

	SimulationInterfaceFactory factory;
	std::shared_ptr<iSimulationData> fmiInterface = factory.makeInterface(eSimulatorName::FMI);
	REQUIRE(fmiInterface != nullptr);
	
	std::shared_ptr<iSimulationData> rosInterface = factory.makeInterface(eSimulatorName::ROS);
	REQUIRE(rosInterface != nullptr);

	std::shared_ptr<iSimulationData> sumoInterface = factory.makeInterface(eSimulatorName::SUMO);
	REQUIRE(sumoInterface != nullptr);

	std::shared_ptr<iSimulationData> ueInterface = factory.makeInterface(eSimulatorName::UNREAL);
	REQUIRE(ueInterface != nullptr);

	std::shared_ptr<iSimulationData> vtdInterface = factory.makeInterface(eSimulatorName::VTD);
	REQUIRE(vtdInterface != nullptr);

	//std::shared_ptr<iSimulationData> osiInterface = factory.makeInterface(eSimulatorName::OSI);
	//REQUIRE(osiInterface != nullptr);

	std::shared_ptr<iSimulationData> errorInterface = factory.makeInterface(eSimulatorName::SIMULATORNAME_ERROR);
	REQUIRE(errorInterface == nullptr);
}
