#include "mapper/OSIMapper.h"

int OSIMapper::readConfiguration(configVariants_t config) {

	//todo reading of lo and hi in addressInformation (currently in OSIBridge) from FMI

	return 0;
}


void OSIMapper::mapToInternalState(osiMessages_t message, eSupportedMessages messageType, int index) {
	int size;
	switch (messageType) {
	case SensorViewMessage:
	{
		if (std::get_if<osi3::SensorView>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::SensorView sensorView = std::get<osi3::SensorView>(message);
		//int index not used here
		osi3::InterfaceVersion *version = sensorView.mutable_version();
		osi3::Timestamp *timestamp = sensorView.mutable_timestamp();
		osi3::Identifier *id = sensorView.mutable_sensor_id();
		osi3::MountingPosition *mountingPosition = sensorView.mutable_mounting_position();
		osi3::MountingPosition *mountingPositionRmse = sensorView.mutable_mounting_position_rmse();
		osi3::HostVehicleData *hostVehicleData = sensorView.mutable_host_vehicle_data();
		osi3::GroundTruth *globalGroundTruth = sensorView.mutable_global_ground_truth();
		osi3::Identifier *hostVehicleId = sensorView.mutable_host_vehicle_id();

		//todo
		//write into internalstate: version, timestamp, sensor_id, mounting_position, mounting_position_rmse, host_vehicle_data, global_ground_truth, host_vehicle_id

		size = sensorView.generic_sensor_view_size();
		for (int index = 0; index < size; index++) {
			osi3::GenericSensorView genericSensorView = sensorView.generic_sensor_view(index);
			mapToInternalState(genericSensorView, GenericSensorViewMessage, index);
		}
		size = sensorView.radar_sensor_view_size();
		for (int index = 0; index < size; index++) {
			osi3::RadarSensorView radarSensorView = sensorView.radar_sensor_view(index);
			mapToInternalState(radarSensorView, RadarSensorViewMessage, index);
		}
		size = sensorView.lidar_sensor_view_size();
		for (int index = 0; index < size; index++) {
			osi3::LidarSensorView lidarSensorView = sensorView.lidar_sensor_view(index);
			mapToInternalState(lidarSensorView, LidarSensorViewMessage, index);
		}
		size = sensorView.camera_sensor_view_size();
		for (int index = 0; index < size; index++) {
			osi3::CameraSensorView cameraSensorView = sensorView.camera_sensor_view(index);
			mapToInternalState(cameraSensorView, CameraSensorViewMessage, index);
		}
		size = sensorView.ultrasonic_sensor_view_size();
		for (int index = 0; index < size; index++) {
			osi3::UltrasonicSensorView ultrasonicSensorView = sensorView.ultrasonic_sensor_view(index);
			mapToInternalState(ultrasonicSensorView, UltrasonicSensorViewMessage, index);
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
			//todo
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

		//todo
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

		//todo
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
		//int index not used here
		osi3::InterfaceVersion *version = sensorViewConfiguration.mutable_version();
		osi3::Identifier *id = sensorViewConfiguration.mutable_sensor_id();
		osi3::MountingPosition *mountingPosition = sensorViewConfiguration.mutable_mounting_position();
		osi3::MountingPosition *mountingPositionRmse = sensorViewConfiguration.mutable_mounting_position_rmse();
		double fieldOfViewHorizontal = sensorViewConfiguration.field_of_view_horizontal();
		double fieldOfViewVertical = sensorViewConfiguration.field_of_view_vertical();
		double range = sensorViewConfiguration.range();
		osi3::Timestamp *updateCycleTime = sensorViewConfiguration.mutable_update_cycle_time();
		osi3::Timestamp *updateCycleOffset = sensorViewConfiguration.mutable_update_cycle_offset();
		osi3::Timestamp *simulationStartTime = sensorViewConfiguration.mutable_simulation_start_time();

		//todo
		//write into internalstate: version, sensor_id, mounting_position, mounting_position_rmse, field_of_view_horizontal, field_of_view_vertical, range, update_cycle_time, update_cycle_offset, simulation_start_time

		size = sensorViewConfiguration.generic_sensor_view_configuration_size();
		for (int index = 0; index < size; index++) {
			osi3::GenericSensorViewConfiguration genericSensorViewConfiguration = sensorViewConfiguration.generic_sensor_view_configuration(index);
			mapToInternalState(genericSensorViewConfiguration, GenericSensorViewConfigurationMessage, index);
		}
		size = sensorViewConfiguration.radar_sensor_view_configuration_size();
		for (int index = 0; index < size; index++) {
			osi3::RadarSensorViewConfiguration radarSensorViewConfiguration = sensorViewConfiguration.radar_sensor_view_configuration(index);
			mapToInternalState(radarSensorViewConfiguration, RadarSensorViewConfigurationMessage, index);
		}
		size = sensorViewConfiguration.lidar_sensor_view_configuration_size();
		for (int index = 0; index < size; index++) {
			osi3::LidarSensorViewConfiguration lidarSensorConfiguration = sensorViewConfiguration.lidar_sensor_view_configuration(index);
			mapToInternalState(lidarSensorConfiguration, LidarSensorViewConfigurationMessage, index);
		}
		size = sensorViewConfiguration.camera_sensor_view_configuration_size();
		for (int index = 0; index < size; index++) {
			osi3::CameraSensorViewConfiguration cameraSensorConfiguration = sensorViewConfiguration.camera_sensor_view_configuration(index);
			mapToInternalState(cameraSensorConfiguration, CameraSensorViewConfigurationMessage, index);
		}
		size = sensorViewConfiguration.ultrasonic_sensor_view_configuration_size();
		for (int index = 0; index < size; index++) {
			osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration = sensorViewConfiguration.ultrasonic_sensor_view_configuration(index);
			mapToInternalState(ultrasonicSensorConfiguration, UltrasonicSensorViewConfigurationMessage, index);
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

		//todo
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
			//todo
			//read horizontal_angle, vertical_angle, response
		}
		for (int index = 0; index < radarSensorViewConfiguration.rx_antenna_diagram_size(); index++) {
			osi3::RadarSensorViewConfiguration_AntennaDiagramEntry rxAntennaDiagram = radarSensorViewConfiguration.rx_antenna_diagram(index);
			//todo
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
			//todo
		}
		for (int index = 0; index < lidarSensorViewConfiguration.timings_size(); index++) {
			uint32_t timings = lidarSensorViewConfiguration.timings(index);
			//todo
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
			//todo
			//map enum value
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

		//todo
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
		//int index not used here
		osi3::InterfaceVersion *version = groundTruth.mutable_version();
		osi3::Timestamp *timestamp = groundTruth.mutable_timestamp();
		osi3::Identifier *hostVehicleId = groundTruth.mutable_host_vehicle_id();
		osi3::EnvironmentalConditions *environmentConditions = groundTruth.mutable_environmental_conditions();
		//uint32
		int countryCode = groundTruth.country_code();
		std::string projString = groundTruth.proj_string();
		std::string mapReference = groundTruth.map_reference();

		//todo
		//write into internalstate: version, timestamp, host_vehicle_id, environmental_conditions, country_code, proj_string, map_reference

		size = groundTruth.stationary_object_size();
		for (int index = 0; index < size; index++) {
			osi3::StationaryObject stationaryObject = groundTruth.stationary_object(index);
			mapToInternalState(stationaryObject, StationaryObjectMessage, index);
		}
		size = groundTruth.moving_object_size();
		for (int index = 0; index < size; index++) {
			osi3::MovingObject movingObject = groundTruth.moving_object(index);
			mapToInternalState(movingObject, MovingObjectMessage, index);
		}
		size = groundTruth.traffic_sign_size();
		for (int index = 0; index < size; index++) {
			osi3::TrafficSign trafficSign = groundTruth.traffic_sign(index);
			mapToInternalState(trafficSign, TrafficSignMessage, index);
		}
		size = groundTruth.traffic_light_size();
		for (int index = 0; index < size; index++) {
			osi3::TrafficLight trafficLight = groundTruth.traffic_light(index);
			mapToInternalState(trafficLight, TrafficLightMessage, index);
		}
		size = groundTruth.road_marking_size();
		for (int index = 0; index < size; index++) {
			osi3::RoadMarking roadMarking = groundTruth.road_marking(index);
			mapToInternalState(roadMarking, RoadMarkingMessage, index);
		}
		size = groundTruth.lane_boundary_size();
		for (int index = 0; index < size; index++) {
			osi3::LaneBoundary laneBoundary = groundTruth.lane_boundary(index);
			mapToInternalState(laneBoundary, LaneBoundaryMessage, index);
		}
		size = groundTruth.lane_size();
		for (int index = 0; index < size; index++) {
			osi3::Lane lane = groundTruth.lane(index);
			mapToInternalState(lane, LaneMessage, index);
		}
		size = groundTruth.occupant_size();
		for (int index = 0; index < size; index++) {
			osi3::Occupant occupant = groundTruth.occupant(index);
			mapToInternalState(occupant, OccupantMessage, index);
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

		//todo
	}
	break;
	case TrafficSignMessage:
	{
		if (std::get_if<osi3::TrafficSign>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::TrafficSign trafficSign = std::get<osi3::TrafficSign>(message);

		//todo
	}
	break;
	case TrafficLightMessage:
	{
		if (std::get_if<osi3::TrafficLight>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::TrafficLight trafficLight = std::get<osi3::TrafficLight>(message);

		//todo
	}
	break;
	case RoadMarkingMessage:
	{
		if (std::get_if<osi3::RoadMarking>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::RoadMarking roadMarking = std::get<osi3::RoadMarking>(message);

		//todo
	}
	break;
	case LaneBoundaryMessage:
	{
		if (std::get_if<osi3::LaneBoundary>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::LaneBoundary laneBoundary = std::get<osi3::LaneBoundary>(message);

		//todo
	}
	break;
	case LaneMessage:
	{
		if (std::get_if<osi3::Lane>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::Lane land = std::get<osi3::Lane>(message);

		//todo
	}
	break;
	case OccupantMessage:
	{
		if (std::get_if<osi3::Occupant>(&message) == nullptr) {
			std::cout << "Called with wrong osi variant!" << std::endl;
			return;
		}
		osi3::Occupant occupant = std::get<osi3::Occupant>(message);

		//todo
	}
	break;
	}
}

//osiMessages_t
osiMessages_t OSIMapper::mapFromInternalState(eSupportedMessages messageType) {
	switch (messageType) {
	case SensorViewMessage:
	{
		osi3::SensorView sensorView;

		//todo
		return (const osiMessages_t)sensorView;
	}
	case GenericSensorViewMessage:
	{
		osi3::GenericSensorView genericSensorView;

		//todo
		return (const osiMessages_t)genericSensorView;
	}
	case RadarSensorViewMessage:
	{
		osi3::RadarSensorView radarSensorView;

		//todo
		return (const osiMessages_t)radarSensorView;
	}
	case LidarSensorViewMessage:
	{
		osi3::LidarSensorView lidarSensorView;

		//todo
		return (const osiMessages_t)lidarSensorView;
	}
	case CameraSensorViewMessage:
	{
		osi3::CameraSensorView cameraSensorView;

		//todo
		return (const osiMessages_t)cameraSensorView;
	}
	case UltrasonicSensorViewMessage:
	{
		osi3::UltrasonicSensorView ultrasonicSensorView;

		//todo
		return (const osiMessages_t)ultrasonicSensorView;
	}
	case SensorViewConfigurationMessage:
	{
		osi3::SensorViewConfiguration sensorViewConfiguration;

		//todo
		return (const osiMessages_t)sensorViewConfiguration;
	}
	case GenericSensorViewConfigurationMessage:
	{
		osi3::GenericSensorViewConfiguration genericSensorViewConfiguration;

		//todo
		return (const osiMessages_t)genericSensorViewConfiguration;
	}
	case RadarSensorViewConfigurationMessage:
	{
		osi3::RadarSensorViewConfiguration radarSensorViewConfiguration;

		//todo
		return (const osiMessages_t)radarSensorViewConfiguration;
	}
	case LidarSensorViewConfigurationMessage:
	{
		osi3::LidarSensorViewConfiguration lidarSensorViewConfiguration;

		//todo
		return (const osiMessages_t)lidarSensorViewConfiguration;
	}
	case CameraSensorViewConfigurationMessage:
	{
		osi3::CameraSensorViewConfiguration cameraSensorConfiguration;

		//todo
		return (const osiMessages_t)cameraSensorConfiguration;
	}
	case UltrasonicSensorViewConfigurationMessage:
	{
		osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration;

		//todo
		return (const osiMessages_t)ultrasonicSensorConfiguration;
	}
	case GroundTruthMessage:
	{
		osi3::GroundTruth groundTruth;

		//todo
		return (const osiMessages_t)groundTruth;
	}
	std::cout << "Error: Message Type not implemented in mapFromInternalState()" << std::endl;
	}
}
