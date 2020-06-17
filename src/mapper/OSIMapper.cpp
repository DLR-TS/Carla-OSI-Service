#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t config) {

	//todo reading of lo and hi in addressInformation (currently in OSIBridge) from FMI

	return 0;
}

void OSIMapper::mapOSIToInternalState(osiMessage_t message, eOSIMessage messageType, int messageIndex) {
	switch (messageType) {
	case SensorViewMessage:
	{
		if (std::get_if<osi3::SensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorView sensorView = std::get<osi3::SensorView>(message);
		if (sensorView.has_version()) {
			osi3::InterfaceVersion *version = sensorView.mutable_version();
		}
		if (sensorView.has_timestamp()) {
			osi3::Timestamp *timestamp = sensorView.mutable_timestamp();
		}
		if (sensorView.has_sensor_id()) {
			osi3::Identifier *sensorId = sensorView.mutable_sensor_id();
		}
		if (sensorView.has_mounting_position()) {
			osi3::MountingPosition *mountingPosition = sensorView.mutable_mounting_position();
		}
		if (sensorView.has_mounting_position_rmse()) {
			osi3::MountingPosition *mountingPositionRmse = sensorView.mutable_mounting_position_rmse();
		}
		if (sensorView.has_host_vehicle_data()) {
			osi3::HostVehicleData *hostVehicleData = sensorView.mutable_host_vehicle_data();
		}
		if (sensorView.has_global_ground_truth()) {
			osi3::GroundTruth *globalGroundTruth = sensorView.mutable_global_ground_truth();
		}
		if (sensorView.has_host_vehicle_id()) {
			osi3::Identifier *hostVehicleId = sensorView.mutable_host_vehicle_id();
		}

		//todo
		//write into internalstate: version, timestamp, sensor_id, mounting_position, mounting_position_rmse, host_vehicle_data, global_ground_truth, host_vehicle_id

		for (int index = 0; index < sensorView.generic_sensor_view_size(); index++) {
			osi3::GenericSensorView genericSensorView = sensorView.generic_sensor_view(index);
			mapOSIToInternalState(genericSensorView, GenericSensorViewMessage, index);
		}
		for (int index = 0; index < sensorView.radar_sensor_view_size(); index++) {
			osi3::RadarSensorView radarSensorView = sensorView.radar_sensor_view(index);
			mapOSIToInternalState(radarSensorView, RadarSensorViewMessage, index);
		}
		for (int index = 0; index < sensorView.lidar_sensor_view_size(); index++) {
			osi3::LidarSensorView lidarSensorView = sensorView.lidar_sensor_view(index);
			mapOSIToInternalState(lidarSensorView, LidarSensorViewMessage, index);
		}
		for (int index = 0; index < sensorView.camera_sensor_view_size(); index++) {
			osi3::CameraSensorView cameraSensorView = sensorView.camera_sensor_view(index);
			mapOSIToInternalState(cameraSensorView, CameraSensorViewMessage, index);
		}
		for (int index = 0; index < sensorView.ultrasonic_sensor_view_size(); index++) {
			osi3::UltrasonicSensorView ultrasonicSensorView = sensorView.ultrasonic_sensor_view(index);
			mapOSIToInternalState(ultrasonicSensorView, UltrasonicSensorViewMessage, index);
		}
	}
	break;
	case GenericSensorViewMessage:
	{
		if (std::get_if<osi3::GenericSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GenericSensorView genericSensorView = std::get<osi3::GenericSensorView>(message);

		std::string interfaceName = "GenericSensorView_";
		interfaceName.append(std::to_string(messageIndex));
		mapToInternalState(genericSensorView.SerializeAsString(), interfaceName, STRINGCOSIMA);

		//todo
		//read view_configuration
	}
	break;
	{
		if (std::get_if<osi3::RadarSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RadarSensorView radarSensorView = std::get<osi3::RadarSensorView>(message);

		//todo
		//read view_configuration
		for (int index = 0; index < radarSensorView.reflection_size(); index++) {
			osi3::RadarSensorView_Reflection reflection = radarSensorView.reflection(index);
			std::string interfaceName = "RadarSensorView_Reflection_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));

			mapToInternalState(reflection.SerializeAsString(), interfaceName, STRINGCOSIMA);
			//todo
			//read signal_strenght, time_of_flight, doppler_shift, source_horizontal_angle, source_vertical_angle
		}
	}
	break;
	case LidarSensorViewMessage:
	{
		if (std::get_if<osi3::LidarSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LidarSensorView lidarSensorView = std::get<osi3::LidarSensorView>(message);

		//todo
		//read view_configuration
		for (int index = 0; index < lidarSensorView.reflection_size(); index++) {
			osi3::LidarSensorView_Reflection reflection = lidarSensorView.reflection(index);
			std::string interfaceName = "LidarSensorView_Reflection_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));

			mapToInternalState(reflection.SerializeAsString(), interfaceName, STRINGCOSIMA);

			//read signal_strenght, time_of_flight, doppler_shift, source_horizontal_angle, source_vertical_angle
		}
	}
	break;
	case CameraSensorViewMessage:
	{
		if (std::get_if<osi3::CameraSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::CameraSensorView cameraSensorView = std::get<osi3::CameraSensorView>(message);
		std::string interfaceName = "CameraSensorView_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(cameraSensorView.SerializeAsString(), interfaceName, STRINGCOSIMA);

		//read view_configuration
		//read image_data
	}
	break;
	case UltrasonicSensorViewMessage:
	{
		if (std::get_if<osi3::UltrasonicSensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::UltrasonicSensorView ultrasonicSensorView = std::get<osi3::UltrasonicSensorView>(message);
		std::string interfaceName = "UltrasonicSensorView_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(ultrasonicSensorView.SerializeAsString(), interfaceName, STRINGCOSIMA);

		//read view_configuration
	}
	break;
	case SensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::SensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorViewConfiguration sensorViewConfiguration = std::get<osi3::SensorViewConfiguration>(message);
		if (sensorViewConfiguration.has_version()) {
			osi3::InterfaceVersion *version = sensorViewConfiguration.mutable_version();
		}
		if (sensorViewConfiguration.has_sensor_id()) {
			osi3::Identifier *sensorId = sensorViewConfiguration.mutable_sensor_id();
		}
		if (sensorViewConfiguration.has_mounting_position()) {
			osi3::MountingPosition *mountingPosition = sensorViewConfiguration.mutable_mounting_position();
		}
		if (sensorViewConfiguration.has_mounting_position_rmse()) {
			osi3::MountingPosition *mountingPositionRmse = sensorViewConfiguration.mutable_mounting_position_rmse();
		}
		if (sensorViewConfiguration.has_field_of_view_horizontal()) {
			double fieldOfViewHorizontal = sensorViewConfiguration.field_of_view_horizontal();
		}
		if (sensorViewConfiguration.has_field_of_view_vertical()) {
			double fieldOfViewVertical = sensorViewConfiguration.field_of_view_vertical();
		}
		if (sensorViewConfiguration.has_range()) {
			double range = sensorViewConfiguration.range();
		}
		if (sensorViewConfiguration.has_update_cycle_time()) {
			osi3::Timestamp *updateCycleTime = sensorViewConfiguration.mutable_update_cycle_time();
		}
		if (sensorViewConfiguration.has_update_cycle_offset()) {
			osi3::Timestamp *updateCycleOffset = sensorViewConfiguration.mutable_update_cycle_offset();
		}
		if (sensorViewConfiguration.has_simulation_start_time()) {
			osi3::Timestamp *simulationStartTime = sensorViewConfiguration.mutable_simulation_start_time();
		}

		//todo
		//write into internalstate: version, sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical, range, update_cycle_time, update_cycle_offset, simulation_start_time

		for (int index = 0; index < sensorViewConfiguration.generic_sensor_view_configuration_size(); index++) {
			osi3::GenericSensorViewConfiguration genericSensorViewConfiguration = sensorViewConfiguration.generic_sensor_view_configuration(index);
			mapOSIToInternalState(genericSensorViewConfiguration, GenericSensorViewConfigurationMessage, index);
		}
		for (int index = 0; index < sensorViewConfiguration.radar_sensor_view_configuration_size(); index++) {
			osi3::RadarSensorViewConfiguration radarSensorViewConfiguration = sensorViewConfiguration.radar_sensor_view_configuration(index);
			mapOSIToInternalState(radarSensorViewConfiguration, RadarSensorViewConfigurationMessage, index);
		}
		for (int index = 0; index < sensorViewConfiguration.lidar_sensor_view_configuration_size(); index++) {
			osi3::LidarSensorViewConfiguration lidarSensorConfiguration = sensorViewConfiguration.lidar_sensor_view_configuration(index);
			mapOSIToInternalState(lidarSensorConfiguration, LidarSensorViewConfigurationMessage, index);
		}
		for (int index = 0; index < sensorViewConfiguration.camera_sensor_view_configuration_size(); index++) {
			osi3::CameraSensorViewConfiguration cameraSensorConfiguration = sensorViewConfiguration.camera_sensor_view_configuration(index);
			mapOSIToInternalState(cameraSensorConfiguration, CameraSensorViewConfigurationMessage, index);
		}
		for (int index = 0; index < sensorViewConfiguration.ultrasonic_sensor_view_configuration_size(); index++) {
			osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration = sensorViewConfiguration.ultrasonic_sensor_view_configuration(index);
			mapOSIToInternalState(ultrasonicSensorConfiguration, UltrasonicSensorViewConfigurationMessage, index);
		}
	}
	break;
	case GenericSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::GenericSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GenericSensorViewConfiguration genericSensorViewConfiguration = std::get<osi3::GenericSensorViewConfiguration>(message);

		std::string interfaceName = "GenericSensorViewConfiguration_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(genericSensorViewConfiguration.SerializeAsString(), interfaceName, STRINGCOSIMA);

		//read sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical
	}
	break;
	case RadarSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::RadarSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RadarSensorViewConfiguration radarSensorViewConfiguration = std::get<osi3::RadarSensorViewConfiguration>(message);

		//todo
		//read sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical, number_of_rays_horizontal, number_of_rays_vertical, max_number_of_interactions, emitter_frequency
		for (int index = 0; index < radarSensorViewConfiguration.tx_antenna_diagram_size(); index++) {
			osi3::RadarSensorViewConfiguration_AntennaDiagramEntry txAntennaDiagram = radarSensorViewConfiguration.tx_antenna_diagram(index);

			std::string interfaceName = "RadarSensorViewConfiguration_TxAntennaDiagram_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));

			mapToInternalState(txAntennaDiagram.SerializeAsString(), interfaceName, STRINGCOSIMA);

			//read horizontal_angle, vertical_angle, response
		}
		for (int index = 0; index < radarSensorViewConfiguration.rx_antenna_diagram_size(); index++) {
			osi3::RadarSensorViewConfiguration_AntennaDiagramEntry rxAntennaDiagram = radarSensorViewConfiguration.rx_antenna_diagram(index);
			std::string interfaceName = "RadarSensorViewConfiguration_RxAntennaDiagram_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));

			mapToInternalState(rxAntennaDiagram.SerializeAsString(), interfaceName, STRINGCOSIMA);

			//read horizontal_angle, vertical_angle, response
		}
	}
	break;
	case LidarSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::LidarSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LidarSensorViewConfiguration lidarSensorViewConfiguration = std::get<osi3::LidarSensorViewConfiguration>(message);

		//todo
		//read sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical, number_of_rays_horizontal, number_of_rays_vertical, max_number_of_interactions, emitter_frequency, num_of_pixels
		for (int index = 0; index < lidarSensorViewConfiguration.directions_size(); index++) {
			osi3::Vector3d directions = lidarSensorViewConfiguration.directions(index);
			std::string interfaceName = "LidarSensorViewConfiguration_Directions_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));

			mapToInternalState(directions.SerializeAsString(), interfaceName, STRINGCOSIMA);
		}
		for (int index = 0; index < lidarSensorViewConfiguration.timings_size(); index++) {
			uint32_t timings = lidarSensorViewConfiguration.timings(index);
			std::string interfaceName = "LidarSensorViewConfiguration_Timings_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));
			//todo check if cast from uint to int is ok
			mapToInternalState((int)timings, interfaceName, INTEGERCOSIMA);
		}
	}
	break;
	case CameraSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::CameraSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::CameraSensorViewConfiguration cameraSensorViewConfiguration = std::get<osi3::CameraSensorViewConfiguration>(message);

		//todo
		//read sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical, number_of_pixels_horizontal, number_of_pixels_vertical
		for (int index = 0; index < cameraSensorViewConfiguration.channel_format_size(); index++) {
			osi3::CameraSensorViewConfiguration_ChannelFormat channelFormat = cameraSensorViewConfiguration.channel_format(index);
			std::string interfaceName = "CameraSensorViewConfiguration_ChannelFormat_";
			interfaceName.append(std::to_string(messageIndex));
			interfaceName.append("_");
			interfaceName.append(std::to_string(index));
			//map enum value
			mapToInternalState(channelFormat, interfaceName, STRINGCOSIMA);
		}
	}
	break;
	case UltrasonicSensorViewConfigurationMessage:
	{
		if (std::get_if<osi3::UltrasonicSensorViewConfiguration>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::UltrasonicSensorViewConfiguration ultrasonicSensorViewConfiguration = std::get<osi3::UltrasonicSensorViewConfiguration>(message);
		std::string interfaceName = "UltrasonicSensorViewConfiguration_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(ultrasonicSensorViewConfiguration.SerializeAsString(), interfaceName, STRINGCOSIMA);
		//read sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical
	}
	break;
	case GroundTruthMessage:
	{
		if (std::get_if<osi3::GroundTruth>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::GroundTruth groundTruth = std::get<osi3::GroundTruth>(message);
		if (groundTruth.has_version()) {
			osi3::InterfaceVersion *version = groundTruth.mutable_version();
		}
		if (groundTruth.has_timestamp()) {
			osi3::Timestamp *timestamp = groundTruth.mutable_timestamp();
		}
		if (groundTruth.has_host_vehicle_id()) {
			osi3::Identifier *hostVehicleId = groundTruth.mutable_host_vehicle_id();
		}
		if (groundTruth.has_environmental_conditions()) {
			osi3::EnvironmentalConditions *environmentConditions = groundTruth.mutable_environmental_conditions();
		}
		if (groundTruth.has_country_code()) {
			uint32_t countryCode = groundTruth.country_code();
		}
		if (groundTruth.has_proj_string()) {
			std::string projString = groundTruth.proj_string();
		}
		if (groundTruth.has_map_reference()) {
			std::string mapReference = groundTruth.map_reference();
		}

		//todo
		//write into internalstate: version, timestamp, host_vehicle_id, environmental_conditions, country_code, proj_string, map_reference

		for (int index = 0; index < groundTruth.stationary_object_size(); index++) {
			osi3::StationaryObject stationaryObject = groundTruth.stationary_object(index);
			mapOSIToInternalState(stationaryObject, StationaryObjectMessage, index);
		}
		for (int index = 0; index < groundTruth.moving_object_size(); index++) {
			osi3::MovingObject movingObject = groundTruth.moving_object(index);
			mapOSIToInternalState(movingObject, MovingObjectMessage, index);
		}
		for (int index = 0; index < groundTruth.traffic_sign_size(); index++) {
			osi3::TrafficSign trafficSign = groundTruth.traffic_sign(index);
			mapOSIToInternalState(trafficSign, TrafficSignMessage, index);
		}
		for (int index = 0; index < groundTruth.traffic_light_size(); index++) {
			osi3::TrafficLight trafficLight = groundTruth.traffic_light(index);
			mapOSIToInternalState(trafficLight, TrafficLightMessage, index);
		}
		for (int index = 0; index < groundTruth.road_marking_size(); index++) {
			osi3::RoadMarking roadMarking = groundTruth.road_marking(index);
			mapOSIToInternalState(roadMarking, RoadMarkingMessage, index);
		}
		for (int index = 0; index < groundTruth.lane_boundary_size(); index++) {
			osi3::LaneBoundary laneBoundary = groundTruth.lane_boundary(index);
			mapOSIToInternalState(laneBoundary, LaneBoundaryMessage, index);
		}
		for (int index = 0; index < groundTruth.lane_size(); index++) {
			osi3::Lane lane = groundTruth.lane(index);
			mapOSIToInternalState(lane, LaneMessage, index);
		}
		for (int index = 0; index < groundTruth.occupant_size(); index++) {
			osi3::Occupant occupant = groundTruth.occupant(index);
			mapOSIToInternalState(occupant, OccupantMessage, index);
		}
	}
	break;
	case MovingObjectMessage:
	{
		if (std::get_if<osi3::MovingObject>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::MovingObject movingObject = std::get<osi3::MovingObject>(message);
		std::string interfaceName = "MovingObject_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(movingObject.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case TrafficSignMessage:
	{
		if (std::get_if<osi3::TrafficSign>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::TrafficSign trafficSign = std::get<osi3::TrafficSign>(message);
		std::string interfaceName = "TrafficSign_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(trafficSign.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case TrafficLightMessage:
	{
		if (std::get_if<osi3::TrafficLight>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::TrafficLight trafficLight = std::get<osi3::TrafficLight>(message);
		std::string interfaceName = "TrafficLight_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(trafficLight.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case RoadMarkingMessage:
	{
		if (std::get_if<osi3::RoadMarking>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RoadMarking roadMarking = std::get<osi3::RoadMarking>(message);
		std::string interfaceName = "RoadMarking_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(roadMarking.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case LaneBoundaryMessage:
	{
		if (std::get_if<osi3::LaneBoundary>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LaneBoundary laneBoundary = std::get<osi3::LaneBoundary>(message);
		std::string interfaceName = "LaneBoundary_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(laneBoundary.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case LaneMessage:
	{
		if (std::get_if<osi3::Lane>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::Lane lane = std::get<osi3::Lane>(message);
		std::string interfaceName = "Lane_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(lane.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	case OccupantMessage:
	{
		if (std::get_if<osi3::Occupant>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::Occupant occupant = std::get<osi3::Occupant>(message);
		std::string interfaceName = "Occupant_";
		interfaceName.append(std::to_string(messageIndex));

		mapToInternalState(occupant.SerializeAsString(), interfaceName, STRINGCOSIMA);
	}
	break;
	}
}

//osiMessage_t
osiMessage_t OSIMapper::mapFromInternalState(eOSIMessage messageType) {
	switch (messageType) {
	case SensorViewMessage:
	{
		osi3::SensorView sensorView;

		//todo
		return (const osiMessage_t)sensorView;
	}
	case GenericSensorViewMessage:
	{
		osi3::GenericSensorView genericSensorView;

		//todo
		return (const osiMessage_t)genericSensorView;
	}
	case RadarSensorViewMessage:
	{
		osi3::RadarSensorView radarSensorView;

		//todo
		return (const osiMessage_t)radarSensorView;
	}
	case LidarSensorViewMessage:
	{
		osi3::LidarSensorView lidarSensorView;

		//todo
		return (const osiMessage_t)lidarSensorView;
	}
	case CameraSensorViewMessage:
	{
		osi3::CameraSensorView cameraSensorView;

		//todo
		return (const osiMessage_t)cameraSensorView;
	}
	case UltrasonicSensorViewMessage:
	{
		osi3::UltrasonicSensorView ultrasonicSensorView;

		//todo
		return (const osiMessage_t)ultrasonicSensorView;
	}
	case SensorViewConfigurationMessage:
	{
		osi3::SensorViewConfiguration sensorViewConfiguration;

		//todo
		return (const osiMessage_t)sensorViewConfiguration;
	}
	case GenericSensorViewConfigurationMessage:
	{
		osi3::GenericSensorViewConfiguration genericSensorViewConfiguration;

		//todo
		return (const osiMessage_t)genericSensorViewConfiguration;
	}
	case RadarSensorViewConfigurationMessage:
	{
		osi3::RadarSensorViewConfiguration radarSensorViewConfiguration;

		//todo
		return (const osiMessage_t)radarSensorViewConfiguration;
	}
	case LidarSensorViewConfigurationMessage:
	{
		osi3::LidarSensorViewConfiguration lidarSensorViewConfiguration;

		//todo
		return (const osiMessage_t)lidarSensorViewConfiguration;
	}
	case CameraSensorViewConfigurationMessage:
	{
		osi3::CameraSensorViewConfiguration cameraSensorConfiguration;

		//todo
		return (const osiMessage_t)cameraSensorConfiguration;
	}
	case UltrasonicSensorViewConfigurationMessage:
	{
		osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration;

		//todo
		return (const osiMessage_t)ultrasonicSensorConfiguration;
	}
	case GroundTruthMessage:
	{
		osi3::GroundTruth groundTruth;

		//todo
		return (const osiMessage_t)groundTruth;
	}
	case StationaryObjectMessage:
	{
		osi3::StationaryObject stationaryObject;

		//todo
		return (const osiMessage_t)stationaryObject;
	}
	case MovingObjectMessage:
	{
		osi3::MovingObject movingObject;

		//todo
		return (const osiMessage_t)movingObject;
	}
	case TrafficSignMessage:
	{
		osi3::TrafficSign trafficSign;

		//todo
		return (const osiMessage_t)trafficSign;
	}
	case TrafficLightMessage:
	{
		osi3::TrafficLight trafficLight;

		//todo
		return (const osiMessage_t)trafficLight;
	}
	case RoadMarkingMessage:
	{
		osi3::RoadMarking roadMarking;

		//todo
		return (const osiMessage_t)roadMarking;
	}
	case LaneBoundaryMessage:
	{
		osi3::LaneBoundary laneBoundary;

		//todo
		return (const osiMessage_t)laneBoundary;
	}
	case LaneMessage:
	{
		osi3::Lane lane;

		//todo
		return (const osiMessage_t)lane;
	}
	case OccupantMessage:
	{
		osi3::Occupant occupant;

		//todo
		return (const osiMessage_t)occupant;
	}
	case SL45TrafficCommandMessage:
	{
		//TrafficCommand (SL45)
		//todo
		break;
	}
	case SL45InVehicleSensorDataMessage:
	{
		//InVehicleSensorData (SL45?)
		//todo
		break;
	}

	std::cout << "Error: Message Type not implemented in mapFromInternalState()" << std::endl;
	}
}
