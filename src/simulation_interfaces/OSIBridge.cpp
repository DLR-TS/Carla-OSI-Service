#include <string>
#include "simulation_interfaces/OSIBridge.h"

int OSIBridge::init(std::string scenario, float starttime, int mode) {
	return 0;
}

int OSIBridge::connect(std::string) {
	return 0;
}

int OSIBridge::disconnect() {
	return 0;
}

int OSIBridge::writeToInternalState() {

	for (auto info : addressInformation) {
		//parse osi message from string to object

		bool parseSuccess = true;
		int size = 0;
		switch (info.first) {
		case SensorViewMessage:
			parseSuccess = sensorView.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}

			size = sensorView.generic_sensor_view_size();
			for (int index = 0; index < size; index++) {
				osi3::GenericSensorView genericSensorView = sensorView.generic_sensor_view(index);
				//mapper->mapToInternalState(genericSensorView, GenericSensorViewMessage, index);
			}
			size = sensorView.radar_sensor_view_size();
			for (int index = 0; index < size; index++) {
				osi3::RadarSensorView radarSensorView = sensorView.radar_sensor_view(index);
				//mapper->mapToInternalState(radarSensorView, RadarSensorViewMessage, index);
			}
			size = sensorView.lidar_sensor_view_size();
			for (int index = 0; index < size; index++) {
				osi3::LidarSensorView lidarSensorView = sensorView.lidar_sensor_view(index);
				//mapper->mapToInternalState(lidarSensorView, LidarSensorViewMessage, index);
			}
			size = sensorView.camera_sensor_view_size();
			for (int index = 0; index < size; index++) {
				osi3::CameraSensorView cameraSensorView = sensorView.camera_sensor_view(index);
				//mapper->mapToInternalState(cameraSensorView, CameraSensorViewMessage, index);
			}
			size = sensorView.ultrasonic_sensor_view_size();
			for (int index = 0; index < size; index++) {
				osi3::UltrasonicSensorView ultrasonicSensorView = sensorView.ultrasonic_sensor_view(index);
				//mapper->mapToInternalState(ultrasonicSensorView, UltrasonicSensorViewMessage, index);
			}
			break;
		case SensorViewConfigurationMessage:
			parseSuccess = sensorViewConfiguration.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}

			size = sensorViewConfiguration.generic_sensor_view_configuration_size();
			for (int index = 0; index < size; index++) {
				osi3::GenericSensorViewConfiguration genericSensorViewConfiguration = sensorViewConfiguration.generic_sensor_view_configuration(index);
				//mapper->mapToInternalState(genericSensorViewConfiguration, GenericSensorViewConfigurationMessage, index);
			}
			size = sensorViewConfiguration.radar_sensor_view_configuration_size();
			for (int index = 0; index < size; index++) {
				osi3::RadarSensorViewConfiguration radarSensorViewConfiguration = sensorViewConfiguration.radar_sensor_view_configuration(index);
				//mapper->mapToInternalState(radarSensorViewConfiguration, RadarSensorViewConfigurationMessage, index);
			}
			size = sensorViewConfiguration.lidar_sensor_view_configuration_size();
			for (int index = 0; index < size; index++) {
				osi3::LidarSensorViewConfiguration lidarSensorConfiguration = sensorViewConfiguration.lidar_sensor_view_configuration(index);
				//mapper->mapToInternalState(lidarSensorConfiguration, LidarSensorConfigurationMessage, index);
			}
			size = sensorViewConfiguration.camera_sensor_view_configuration_size();
			for (int index = 0; index < size; index++) {
				osi3::CameraSensorViewConfiguration cameraSensorConfiguration = sensorViewConfiguration.camera_sensor_view_configuration(index);
				//mapper->mapToInternalState(cameraSensorConfiguration, CameraSensorConfigurationMessage, index);
			}
			size = sensorViewConfiguration.ultrasonic_sensor_view_configuration_size();
			for (int index = 0; index < size; index++) {
				osi3::UltrasonicSensorViewConfiguration ultrasonicSensorConfiguration = sensorViewConfiguration.ultrasonic_sensor_view_configuration(index);
				//mapper->mapToInternalState(ultrasonicSensorConfiguration, UltrasonicSensorConfigurationMessage, index);
			}
			break;
		case GroundTruthMessage:
			parseSuccess = groundTruth.ParseFromArray((const void*)info.second.addr.address, info.second.size);
			if (parseSuccess == false) {
				return 1;
			}

			size = groundTruth.stationary_object_size();
			for (int index = 0; index < size; index++) {
				osi3::StationaryObject stationaryObject = groundTruth.stationary_object(index);
				//mapper->mapToInternalState(stationaryObject, StationaryObjectMessage, index);
			}
			size = groundTruth.moving_object_size();
			for (int index = 0; index < size; index++) {
				osi3::MovingObject movingObject = groundTruth.moving_object(index);
				//mapper->mapToInternalState(movingObject, MovingObjectMessage, index);
			}
			size = groundTruth.traffic_sign_size();
			for (int index = 0; index < size; index++) {
				osi3::TrafficSign trafficSign = groundTruth.traffic_sign(index);
				//mapper->mapToInternalState(trafficSign, TrafficSignMessage, index);
			}
			size = groundTruth.traffic_light_size();
			for (int index = 0; index < size; index++) {
				osi3::TrafficLight trafficLight = groundTruth.traffic_light(index);
				//mapper->mapToInternalState(trafficLight, TrafficLightMessage, index);
			}
			size = groundTruth.road_marking_size();
			for (int index = 0; index < size; index++) {
				osi3::RoadMarking roadMarking = groundTruth.road_marking(index);
				//mapper->mapToInternalState(roadMarking, RoadMarkingMessage, index);
			}
			size = groundTruth.lane_boundary_size();
			for (int index = 0; index < size; index++) {
				osi3::LaneBoundary laneBoundary = groundTruth.lane_boundary(index);
				//mapper->mapToInternalState(laneBoundary, LaneBoundaryMessage, index);
			}
			size = groundTruth.lane_size();
			for (int index = 0; index < size; index++) {
				osi3::Lane lane = groundTruth.lane(index);
				//mapper->mapToInternalState(lane, LaneMessage, index);
			}
			size = groundTruth.occupant_size();
			for (int index = 0; index < size; index++) {
				osi3::Occupant occupant = groundTruth.occupant(index);
				//mapper->mapToInternalState(occuopant, OccupantMessage, index);
			}
			break;
		}
	}

	return 0;
}

int OSIBridge::doStep(double stepSize) {
	return 0;
}

int OSIBridge::readFromInternalState() {
	for (auto info : addressInformation) {
		//parse osi message from string to object

		switch (info.first) {
		case SensorViewMessage:
			//sensorView = mapper->mapFromInternalState
			sensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case GenericSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//genericSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case RadarSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//radarSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case LidarSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//lidarSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case CameraSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//cameraSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		case UltrasonicSensorViewMessage:
			//genericSensorView = mapper->mapFromInternalState
			//ultrasonicSensorView.SerializeToArray((void*)info.second.addr.address, info.second.size);
			break;
		}
	}

	return 0;
}
