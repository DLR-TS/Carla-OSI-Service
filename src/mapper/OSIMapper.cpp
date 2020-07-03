#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t configVariants) {

	std::cout << "Read Configuration of OSI Mapper" << std::endl;

	if (std::get_if<OSIInterfaceConfig>(&configVariants) == nullptr) {
		std::cout << "Called with wrong configuration variant!" << std::endl;
		return 1;
	}

	OSIInterfaceConfig interfaceConfig = std::get<OSIInterfaceConfig>(configVariants);

	for (auto input : interfaceConfig.inputs) {
		config.stringInputList.push_back(NamesAndIndex(input.base_name, input.interface_name, (int)state->strings.size()));
		state->strings.push_back(std::string());
	}
	for (auto output : interfaceConfig.outputs) {
		config.stringOutputList.push_back(NamesAndIndex(output.base_name, output.interface_name, (int)state->strings.size()));
		state->strings.push_back(std::string());
	}
	return 0;
}

void OSIMapper::mapOSIToInternalState(osiMessage_t message, eOSIMessage messageType) {
	switch (messageType)
	{
	case SensorViewMessage:
	{
		if (std::get_if<osi3::SensorView>(&message) == nullptr) {
			std::cout << "Called SensorView with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorView sensorView = std::get<osi3::SensorView>(message);
		mapToInternalState(sensorView.SerializeAsString(), "SensorView", STRINGCOSIMA);
	}
	case SensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::SensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called SensorViewConfiguration with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorViewConfiguration sensorViewConfiguration = std::get<osi3::SensorViewConfiguration>(message);
		mapToInternalState(sensorViewConfiguration.SerializeAsString(), "SensorViewConfiguration", STRINGCOSIMA);
	}
	case SensorDataMessage:
	{
		if (std::get_if<osi3::SensorData>(&message) == nullptr) {
			std::cout << "Called SensorData with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorData groundTruth = std::get<osi3::SensorData>(message);
		mapToInternalState(groundTruth.SerializeAsString(), "SensorData", STRINGCOSIMA);
	}
	case GroundTruthMessage:
	{
		if (std::get_if<osi3::GroundTruth>(&message) == nullptr) {
			std::cout << "Called SensorViewConfiguration with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GroundTruth groundTruth = std::get<osi3::GroundTruth>(message);
		mapToInternalState(groundTruth.SerializeAsString(), "GroundTruth", STRINGCOSIMA);
	}
	case SL45TrafficCommandMessage:
	{
		//if (std::get_if<osi3::SensorViewConfiguration>(&message) == nullptr) {
		//	std::cout << "Called SensorViewConfiguration with wrong osi variant!" << std::endl;
		//	return;
		//}
		//osi3::SensorViewConfiguration sensorViewConfiguration = std::get<osi3::SensorViewConfiguration>(message);
		//mapToInternalState(sensorViewConfiguration.SerializeAsString(), "SensorViewConfiguration", STRINGCOSIMA);
	}
	case SL45InVehicleSensorDataMessage:
	{
		//if (std::get_if<osi3::SensorViewConfiguration>(&message) == nullptr) {
		//	std::cout << "Called SensorViewConfiguration with wrong osi variant!" << std::endl;
		//	return;
		//}
		//osi3::SensorViewConfiguration sensorViewConfiguration = std::get<osi3::SensorViewConfiguration>(message);
		//mapToInternalState(sensorViewConfiguration.SerializeAsString(), "SensorViewConfiguration", STRINGCOSIMA);
	}
	}
}

//osiMessage_t
osiMessage_t OSIMapper::mapOSIFromInternalState(eOSIMessage messageType) {
	switch (messageType)
	{
	case SensorViewMessage:
	{
		values_t sensorViewVariant = mapFromInternalState("SensorView", STRINGCOSIMA);
		osi3::SensorView sensorView;
		sensorView.ParseFromString(std::get<std::string>(sensorViewVariant));
		return (const osiMessage_t)sensorView;
	}
	case SensorViewConfigurationMessage:
	{
		values_t sensorViewConfigurationVariant = mapFromInternalState("SensorViewConfiguration", STRINGCOSIMA);
		osi3::SensorViewConfiguration sensorViewConfiguration;
		sensorViewConfiguration.ParseFromString(std::get<std::string>(sensorViewConfigurationVariant));
		return (const osiMessage_t)sensorViewConfiguration;
	}
	case SensorDataMessage:
	{
		values_t sensorDataVariant = mapFromInternalState("SensorData", STRINGCOSIMA);
		osi3::SensorData sensorData;
		sensorData.ParseFromString(std::get<std::string>(sensorDataVariant));
		return (const osiMessage_t)sensorData;
	}
	case GroundTruthMessage:
	{
		values_t groundTruthVariant = mapFromInternalState("GroundTruth", STRINGCOSIMA);
		osi3::GroundTruth groundTruth;
		groundTruth.ParseFromString(std::get<std::string>(groundTruthVariant));
		return (const osiMessage_t)groundTruth;
	}
	case SL45TrafficCommandMessage:
	{
		//values_t SL45TrafficCommandVariant = mapFromInternalState("SL45TrafficCommand", STRINGCOSIMA);
		//osi3::??? SL45TrafficCommand;
		//SL45TrafficCommand.ParseFromString(std::get<std::string>(SL45TrafficCommandVariant));
		//return SL45TrafficCommand;
	}
	case SL45InVehicleSensorDataMessage:
	{
		//values_t SL45InVehicleSensorDataVariant = mapFromInternalState("SL45InVehicleSensorDat", STRINGCOSIMA);
		//osi3::??? SL45InVehicleSensorDat;
		//SL45InVehicleSensorDat.ParseFromString(std::get<std::string>(SL45InVehicleSensorDataVariant));
		//return SL45InVehicleSensorDat;
	}
	}
}
