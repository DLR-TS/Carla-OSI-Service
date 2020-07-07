#include "catch2/catch.hpp"

#include "mapper/Mapper.h"
#include "mapper/FMIMapper.h"
#include "simulation_interfaces/FMIBridge.h"
#include "configreader/StandardYAMLConfig.h"
#include "MockBaseSimulator.cpp"
#include "MockInterfaceSimulator.cpp"
#include "simulation_interfaces/OSMPBridge.h"


TEST_CASE("OSMP Test") {

	const auto osiMapper = new OSIMapper();
	auto mapper = std::shared_ptr<Mapper>((Mapper*)osiMapper);
	OSMPBridge bridge(mapper);

	SECTION("OSMP methods")
	{
		//names from https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/blob/master/examples/OSMPDummySensor/modelDescription.in.xml

		std::string name1 = "OSMPSensorViewIn.base.lo";
		std::string name2 = "OSMPSensorViewIn.base.hi";
		std::string name3 = "OSMPSensorViewIn.size";
		std::string name4 = "OSMPSensorViewInConfigRequest.base.hi";
		std::string name5 = "OSMPSensorViewInConfigRequest.base.lo";
		std::string name6 = "OSMPSensorViewInConfigRequest.size";
		std::string name7 = "OSMPSensorDataOut.base.lo";
		std::string name8 = "OSMPSensorDataOut.base.hi";
		std::string name9 = "OSMPSensorDataOut.size";
		std::string name10 = "OSMPSensorViewInConfig.base.lo";
		std::string name11 = "OSMPSensorViewInConfig.base.hi";
		std::string name12 = "OSMPSensorViewInConfig.size";

		SECTION("Interpret the OSMPNames correct") {

			REQUIRE(bridge.getMessageType(name1) == eOSIMessage::SensorViewMessage);
			REQUIRE(bridge.getMessageType(name2) == eOSIMessage::SensorViewMessage);
			REQUIRE(bridge.getMessageType(name3) == eOSIMessage::SensorViewMessage);
			REQUIRE(bridge.getMessageType(name4) == eOSIMessage::SensorViewConfigurationMessage);
			REQUIRE(bridge.getMessageType(name5) == eOSIMessage::SensorViewConfigurationMessage);
			REQUIRE(bridge.getMessageType(name6) == eOSIMessage::SensorViewConfigurationMessage);
			REQUIRE(bridge.getMessageType(name7) == eOSIMessage::SensorDataMessage);
			REQUIRE(bridge.getMessageType(name8) == eOSIMessage::SensorDataMessage);
			REQUIRE(bridge.getMessageType(name9) == eOSIMessage::SensorDataMessage);
			REQUIRE(bridge.getMessageType(name10) == eOSIMessage::SensorViewConfigurationMessage);
			REQUIRE(bridge.getMessageType(name11) == eOSIMessage::SensorViewConfigurationMessage);
			REQUIRE(bridge.getMessageType(name12) == eOSIMessage::SensorViewConfigurationMessage);
		}

		SECTION("Fill addresses correct") {
			bridge.saveToAddressMap(name1, 1234);
			bridge.saveToAddressMap(name2, 5678);
			bridge.saveToAddressMap(name3, 90);

			REQUIRE(bridge.addresses.size() == 1);
			REQUIRE(bridge.addresses.at("OSMPSensorViewIn").size == 90);
			REQUIRE(bridge.addresses.at("OSMPSensorViewIn").addr.base.lo == 1234);
			REQUIRE(bridge.addresses.at("OSMPSensorViewIn").addr.base.hi == 5678);

			bridge.saveToAddressMap(name4, 91);
			bridge.saveToAddressMap(name8, 92);
			bridge.saveToAddressMap(name12, 93);

			REQUIRE(bridge.addresses.size() == 4);
		}
	}
}
