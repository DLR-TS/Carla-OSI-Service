#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t configVariants) {

	std::cout << "Read Configuration of OSI Mapper" << std::endl;

	if (std::get_if<OSIInterfaceConfig>(&configVariants) == nullptr) {
		std::cout << "Called with wrong configuration variant!" << std::endl;
		return 1;
	}

	OSIInterfaceConfig interfaceConfig = std::get<OSIInterfaceConfig>(configVariants);
	//overwrite prefix default value if setin configuration
	if (interfaceConfig.prefix.length() != 0) {
		this->prefix = interfaceConfig.prefix;
	}
	for (auto input : interfaceConfig.inputs) {
		config.stringInputList.push_back(NamesAndIndex(prefix + input.base_name, input.interface_name, (int)state->strings.size()));
		state->strings.push_back(std::string());
	}
	for (auto output : interfaceConfig.outputs) {
		config.stringOutputList.push_back(NamesAndIndex(prefix + output.base_name, output.interface_name, (int)state->strings.size()));
		state->strings.push_back(std::string());
	}
	return 0;
}

void OSIMapper::mapOSIToInternalState(std::string message, eOSIMessage messageType) {
	switch (messageType)
	{
	case SensorViewMessage:
		mapToInternalState(message, "SensorView", STRINGCOSIMA);
		break;
	case SensorViewConfigurationMessage:
		mapToInternalState(message, "SensorViewConfiguration", STRINGCOSIMA);
		break;
	case SensorDataMessage:
		mapToInternalState(message, "SensorData", STRINGCOSIMA);
		break;
	case GroundTruthMessage:
		mapToInternalState(message, "GroundTruth", STRINGCOSIMA);
		break;
	case SL45TrafficCommandMessage:
		//mapToInternalState(message, "SensorViewConfiguration", STRINGCOSIMA);
		break;
	case SL45InVehicleSensorDataMessage:
		//mapToInternalState(message, "SensorViewConfiguration", STRINGCOSIMA);
		break;
	}
}

//osiMessage_t
std::string OSIMapper::mapOSIFromInternalState(eOSIMessage messageType) {
	switch (messageType)
	{
	case SensorViewMessage:
	{
		values_t sensorViewVariant = mapFromInternalState("SensorView", STRINGCOSIMA);
		osi3::SensorView sensorView;
		sensorView.ParseFromString(std::get<std::string>(sensorViewVariant));
		return sensorView.SerializeAsString();
	}
	break;
	case SensorViewConfigurationMessage:
	{
		values_t sensorViewConfigurationVariant = mapFromInternalState("SensorViewConfiguration", STRINGCOSIMA);
		osi3::SensorViewConfiguration sensorViewConfiguration;
		sensorViewConfiguration.ParseFromString(std::get<std::string>(sensorViewConfigurationVariant));
		return sensorViewConfiguration.SerializeAsString();
	}
	break;
	case SensorDataMessage:
	{
		values_t sensorDataVariant = mapFromInternalState("SensorData", STRINGCOSIMA);
		osi3::SensorData sensorData;
		sensorData.ParseFromString(std::get<std::string>(sensorDataVariant));
		return sensorData.SerializeAsString();
	}
	break;
	case GroundTruthMessage:
	{
		values_t groundTruthVariant = mapFromInternalState("GroundTruth", STRINGCOSIMA);
		osi3::GroundTruth groundTruth;
		groundTruth.ParseFromString(std::get<std::string>(groundTruthVariant));
		return groundTruth.SerializeAsString();
	}
	break;
	case SL45TrafficCommandMessage:
	{
		//values_t SL45TrafficCommandVariant = mapFromInternalState("SL45TrafficCommand", STRINGCOSIMA);
		//osi3::??? SL45TrafficCommand;
		//SL45TrafficCommand.ParseFromString(std::get<std::string>(SL45TrafficCommandVariant));
		//return SL45TrafficCommand.SerializeAsString();
	}
	break;
	case SL45InVehicleSensorDataMessage:
	{
		//values_t SL45InVehicleSensorDataVariant = mapFromInternalState("SL45InVehicleSensorDat", STRINGCOSIMA);
		//osi3::??? SL45InVehicleSensorDat;
		//SL45InVehicleSensorDat.ParseFromString(std::get<std::string>(SL45InVehicleSensorDataVariant));
		//return SL45InVehicleSensorDat.SerializeAsString();
	}
	break;
	}
}
