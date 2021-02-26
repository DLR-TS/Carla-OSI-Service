#include "Utility.h"

#include "carla_osi/Geometry.h"
#include "carla_osi/Identifiers.h"
#include "carla_osi/TrafficSignals.h"

osi3::StationaryObject* CarlaUtility::toOSI(const carla::SharedPtr< const carla::client::Actor> actor, carla::geom::BoundingBox& bbox)
{
	osi3::StationaryObject* prop = new osi3::StationaryObject();
	prop->set_allocated_id(carla_osi::id_mapping::getOSIActorId(actor).release());

	osi3::BaseStationary* base = prop->mutable_base();
	// bounding boxes are only available for Junction, Vehicle and Walker, not for Actor as generalization (though there is a protected GetBoundingBox() member in ActorState)
	// also mentioned in https://github.com/carla-simulator/carla/issues/3186, https://github.com/carla-simulator/carla/issues/3025 and https://github.com/carla-simulator/carla/issues/1766
	// The attribute behind the protected field is misused for traffic signs and traffic lights and holds their active area instead
	//auto [dimension, position] = CarlaUtility::toOSI( actor-> Get BoundingBox() );
	// The bounding box has to be given as argument. To circumvent the described limitation, World::GetActorBoundingBox(ActorId) is added
	auto[dimension, position] = carla_osi::geometry::toOSI(bbox);
	base->set_allocated_dimension(dimension.release());
	auto transform = actor->GetTransform();
	base->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	base->set_allocated_orientation(carla_osi::geometry::toOSI(transform.rotation).release());

	//TODO How to get base_polygon from actor? (https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1BaseStationary.html#aa1db348acaac2d5a2ba0883903d962cd)

	//TODO Carla doesn't seem to offer information needed for osi3::StationaryObject::Classification. Using default instance
	auto classification = prop->mutable_classification();//creates default instance as side-effect
	classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_OTHER);
	//TODO fill with information from OpenDRIVE file, if available

	prop->set_model_reference(actor->GetTypeId());

	return prop;
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	// but bounding boxes only exist for Vehicle and Walker specializations
	base->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	base->set_allocated_orientation(carla_osi::geometry::toOSI(transform.rotation).release());

	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Walker> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	//TODO verify relation of bouding box origin and actor.GetLocation() for Walkers
	auto bbox = actor->GetBoundingBox();
	// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in carla_osi::geometry::toOSI
	auto[dimension, location] = carla_osi::geometry::toOSI(actor->GetBoundingBox());
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations - assume bbox is not rotated
	base->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	base->set_allocated_orientation(carla_osi::geometry::toOSI(transform.rotation).release());

	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	auto bbox = actor->GetBoundingBox();
	// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in carla_osi::geometry::toOSI
	auto[dimension, location] = carla_osi::geometry::toOSI(bbox);
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations, but
	// vehicle bounding boxes shouldn't be rotated
	base->set_allocated_position(carla_osi::geometry::toOSI(transform.location).release());
	base->set_allocated_orientation(carla_osi::geometry::toOSI(transform.rotation).release());


	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving_common(const carla::SharedPtr<const carla::client::Actor> actor, std::unique_ptr<osi3::BaseMoving> base)
{
	auto transform = actor->GetTransform();

	//TODO determine contour on z-plane
	//auto contour = base->mutable_base_polygon();
	//contour->Add

	// velocity and acceleration as part of ground truth are given in global coordinate system
	//TODO reference frame of actor velocity is not documented might be local and has to be transformed
	base->set_allocated_velocity(carla_osi::geometry::toOSI(actor->GetVelocity()).release());
	base->set_allocated_acceleration(carla_osi::geometry::toOSI(actor->GetAcceleration()).release());
	auto angularVelocity = actor->GetAngularVelocity();//Carla uses Vector3d instead of Rotation as type
	base->set_allocated_orientation_rate(carla_osi::geometry::toOSI(
		carla::geom::Rotation(angularVelocity.y, angularVelocity.z, angularVelocity.x)).release());

	//TODO Carla has no rotational acceleration
	//base->set_allocated_orientation_acceleration

	return base;
}

osi3::TrafficSign* CarlaUtility::toOSI(const carla::SharedPtr<const carla::client::TrafficSign> actor/*, const pugi::xml_document& xodr*/)
{
	//deprecated (moved to carla_osi::traffic_signals)
	return carla_osi::traffic_signals::getOSITrafficSign(actor).release();
}

std::vector<osi3::TrafficLight*> CarlaUtility::toOSI(const carla::SharedPtr<const carla::client::TrafficLight> actor/*, const  pugi::xml_document& xodr*/)
{
	//deprecated (moved to carla_osi::traffic_signals)
	auto lights = carla_osi::traffic_signals::getOSITrafficLight(actor);
	std::vector<osi3::TrafficLight*> lightPtrs;
	for (auto& light : lights) {
		lightPtrs.push_back(light.release());
	}
	return lightPtrs;
}

std::unique_ptr<osi3::MovingObject_VehicleClassification_LightState> CarlaUtility::toOSI(carla::client::Vehicle::LightState vehicleLights)
{
	auto lightState = std::make_unique<osi3::MovingObject_VehicleClassification_LightState>();

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Position) {
		// has no mapping
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::LowBeam) {
		lightState->set_head_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_head_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::HighBeam) {
		lightState->set_high_beam(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_high_beam(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Brake) {
		lightState->set_brake_light_state(osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL);
	}
	else {
		lightState->set_brake_light_state(osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights &
		((uint32_t)carla::client::Vehicle::LightState::RightBlinker & (uint32_t)carla::client::Vehicle::LightState::LeftBlinker)) {
		// Both indicator lights
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_WARNING);
	}
	else if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::RightBlinker) {
		// Only right indicator light
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_RIGHT);
	}
	else if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::LeftBlinker) {
		// Only left indicator light
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_LEFT);
	}
	else {
		lightState->set_indicator_state(osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Reverse) {
		lightState->set_reversing_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_reversing_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	// Setting both, front and rear fog lights because Carla does not differentiate
	//TODO do Carla vehicles have both front and rear fog lights or only front or rear?
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Fog) {
		lightState->set_front_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
		lightState->set_rear_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON);
	}
	else {
		lightState->set_front_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
		lightState->set_rear_fog_light(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	}

	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Interior) {
		// has no mapping
	}

	//TODO how to determine type of special illumination in Carla? OSI field can only be set if supported by the vehicle, which cannot be determined from Carla's light state
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Special1) {
		lightState->set_emergency_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE);
	}
	//else {
	//	lightState->set_emergency_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	//}
	if ((uint32_t)vehicleLights & (uint32_t)carla::client::Vehicle::LightState::Special2) {
		lightState->set_service_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER);
	}
	//else {
	//	lightState->set_service_vehicle_illumination(osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF);
	//}

	return lightState;
}

osi3::CameraSensorView* CarlaUtility::toOSICamera(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	//Contains RGBA uint8 values
	auto image = boost::dynamic_pointer_cast<const carla::sensor::data::Image>(sensorData);
	if (!image) return nullptr;
	auto height = image->GetHeight();
	auto width = image->GetWidth();
	double aspect = ((double)width) / ((double)height);
	auto pixelCount = height * width;

	//Buffer for RGB OSI raw image
	auto rgb = new char[pixelCount * 3];

	//TODO use faster pixel format conversion or use osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_OTHER
	for (size_t i = 0; i < pixelCount; i++) {
		auto pixel = image->at(i);
		rgb[3 * i] = pixel.r;
		rgb[3 * i + 1] = pixel.g;
		rgb[3 * i + 2] = pixel.b;
	}

	osi3::CameraSensorView* cameraSensorView = new osi3::CameraSensorView();
	cameraSensorView->set_image_data(rgb);

	auto config = cameraSensorView->mutable_view_configuration();
	config->add_channel_format(osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_RGB_U8_LIN);
	// Carla only reports horizontal field of view
	auto fov = image->GetFOVAngle() * M_PI / 180.0;
	config->set_field_of_view_horizontal(fov);
	// guess vertical fov based on aspect ratio
	config->set_field_of_view_horizontal(fov / aspect);
	config->set_number_of_pixels_horizontal(width);
	config->set_number_of_pixels_vertical(height);
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());

	//TODO calculate sensor position in vehicle coordinates
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)


	return cameraSensorView;
}

osi3::LidarSensorView* CarlaUtility::toOSILidar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::LidarMeasurement>(sensorData);
	std::optional<double> rotationFrequency;
	std::optional<double> upperFov;
	std::optional<double> lowerFov;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("rotation_frequency" == attribute.GetId()) {
			rotationFrequency = std::stod(attribute.GetValue());
		}
		else if ("upper_fov" == attribute.GetId()) {
			upperFov = std::stod(attribute.GetValue());
		}
		else if ("lower_fov" == attribute.GetId()) {
			upperFov = std::stod(attribute.GetValue());
		}
	}
	std::optional<double> vFov;
	if (upperFov && lowerFov) {
		//upper and lower field of view are given in degree
		vFov = (upperFov.value() - lowerFov.value()) * M_PI / 180.0;
	}
	uint32_t numPixels;//OSI field uses uint32_t
	for (size_t i = 0; i < measurement->GetChannelCount(); i++) {
		numPixels += measurement->GetPointCount(i);
	}

	//TODO find translation from Carla point cloud to OSI Reflections. OSI uses signal strength, time of flight, doppler shift and normal to surface as measurements instead of simple hit point positions.
	//TODO Maybe use the osi3::FeatureData-based osi3::LidarDetection instead of a osi3::SensorView, which is similar to Carla's Lidar output

	osi3::LidarSensorView* lidarSensorView = new osi3::LidarSensorView();

	auto config = lidarSensorView->mutable_view_configuration();
	//TODO get lidar directions
	//config->add_directions()
	if (rotationFrequency) {
		config->set_emitter_frequency(rotationFrequency.value());
	}
	if (vFov) {
		config->set_field_of_view_vertical(vFov.value());
	}
	config->set_field_of_view_horizontal(M_PI * 2);
	config->set_max_number_of_interactions(1);

	//TODO calculate sensor position in vehicle coordinates, that is relative to the vehicles rear
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)

	//TODO OSI expects a constant number of pixels per message, but Carla only reports new values of the angle sweeped during the last frame
	config->set_num_of_pixels(numPixels);
	//TODO number of rays (horizontal/vertical) of lidar
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());

	return lidarSensorView;
}

osi3::RadarSensorView* CarlaUtility::toOSIRadar(const carla::SharedPtr<const carla::client::Sensor> sensor, const carla::SharedPtr<const carla::sensor::SensorData> sensorData)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::RadarMeasurement>(sensorData);
	std::optional<double> hFov;
	std::optional<double> vFov;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("horizontal_fov" == attribute.GetId()) {
			hFov = std::stod(attribute.GetValue());
		}
		else if ("vertical_fov" == attribute.GetId()) {
			vFov = std::stod(attribute.GetValue());
		}
	}

	auto radarSensorview = new osi3::RadarSensorView();

	//TODO find translation from Carla point cloud to OSI Reflections. OSI uses signal strength, time of flight, doppler shift and source vertical and horizontal angle as measurements instead of simple hit point positions.
	//TODO Maybe use the osi3::FeatureData-based osi3::RadarDetection instead of a osi3::SensorView, which is similar to Carla's Radar output

	auto config = radarSensorview->mutable_view_configuration();
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());
	if (hFov) {
		config->set_field_of_view_horizontal(hFov.value());
	}
	if (vFov) {
		config->set_field_of_view_vertical(vFov.value());
	}
	config->set_max_number_of_interactions(1);
	//TODO number of rays (horizontal/vertical)
	//TODO rx and tx antenna diagrams
	//TODO calculate sensor position in vehicle coordinates, that is relative to the vehicles rear
	//config->set_allocated_mounting_position(position)
	//config->set_allocated_mounting_position_rmse(rmse)

	return nullptr;
}

carla::SharedPtr<carla::client::Vehicle> CarlaUtility::getParentVehicle(const carla::SharedPtr<const carla::client::Actor> actor)
{
	auto current = actor->GetParent();
	while (current && 0 != current->GetTypeId().rfind("vehicle", 0)) {
		current = current->GetParent();
	}
	auto vehicle = boost::dynamic_pointer_cast<carla::client::Vehicle>(current);
	return vehicle;
}

carla::rpc::VehicleLightState::LightState CarlaUtility::toCarla(osi3::MovingObject_VehicleClassification_LightState* indicatorState) {
	//aggregate all received light states
	std::set<carla::rpc::VehicleLightState::LightState> receivedStates;

	if (indicatorState->has_indicator_state()) {
		switch (indicatorState->indicator_state()) {
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_LEFT:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LeftBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_RIGHT:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::RightBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_WARNING:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LeftBlinker);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::RightBlinker);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_IndicatorState_INDICATOR_STATE_OTHER:
			break;
		default:
			break;
		}
	}
	//same effect as rear_fog_light
	if (indicatorState->has_front_fog_light()) {
		switch (indicatorState->front_fog_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Fog);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as front_fog_light
	if (indicatorState->has_rear_fog_light()) {
		switch (indicatorState->rear_fog_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Fog);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_head_light()) {
		switch (indicatorState->head_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::LowBeam);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_high_beam()) {
		switch (indicatorState->high_beam()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::HighBeam);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_reversing_light()) {
		switch (indicatorState->reversing_light()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Reverse);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	if (indicatorState->has_brake_light_state()) {
		switch (indicatorState->brake_light_state()) {
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_NORMAL:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_STRONG:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Brake);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_BrakeLightState_BRAKE_LIGHT_STATE_OTHER:
			break;
		default:
			break;
		}
	}
	//not part of carla
	if (indicatorState->has_license_plate_illumination_rear()) {
		switch (indicatorState->license_plate_illumination_rear()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as service_vehicle_illumination
	//(Special1 and Special2: This is reserved for certain vehicles that can have special lights, like a siren.)
	if (indicatorState->has_emergency_vehicle_illumination()) {
		switch (indicatorState->emergency_vehicle_illumination()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special1);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special2);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}
	//same effect as emergency_vehicle_illumination
	//(Special1 and Special2: This is reserved for certain vehicles that can have special lights, like a siren.)
	if (indicatorState->has_service_vehicle_illumination()) {
		switch (indicatorState->service_vehicle_illumination()) {
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_ON:
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special1);
			receivedStates.emplace(carla::rpc::VehicleLightState::LightState::Special2);
			break;
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OFF:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_UNKNOWN:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_OTHER:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_BLUE_AND_RED:
		case osi3::MovingObject_VehicleClassification_LightState_GenericLightState_GENERIC_LIGHT_STATE_FLASHING_AMBER:
			break;
		default:
			break;
		}
	}

	//aggregate all received light states into one state
	carla::rpc::VehicleLightState::LightState state = carla::rpc::VehicleLightState::LightState::None;
	for (carla::rpc::VehicleLightState::LightState receivedState : receivedStates) {
		state = carla::rpc::VehicleLightState::LightState(
			(carla::rpc::VehicleLightState::flag_type)state | (carla::rpc::VehicleLightState::flag_type)receivedState);
	}
	return state;
}

osi3::MovingObject_VehicleClassification_Type CarlaUtility::ParseVehicleType(const std::string & value)
{
	if (std::string::npos != value.find("OTHER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_OTHER;
	}
	if (std::string::npos != value.find("SMALL_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR;
	}
	if (std::string::npos != value.find("COMPACT_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_COMPACT_CAR;
	}
	if (std::string::npos != value.find("MEDIUM_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR;
	}
	if (std::string::npos != value.find("LUXURY_CAR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR;
	}
	if (std::string::npos != value.find("DELIVERY_VAN")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_DELIVERY_VAN;
	}
	if (std::string::npos != value.find("HEAVY_TRUCK")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_HEAVY_TRUCK;
	}
	if (std::string::npos != value.find("SEMITRAILER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_SEMITRAILER;
	}
	if (std::string::npos != value.find("TRAILER")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAILER;
	}
	if (std::string::npos != value.find("MOTORBIKE")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE;
	}
	if (std::string::npos != value.find("BICYCLE")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_BICYCLE;
	}
	if (std::string::npos != value.find("BUS")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_BUS;
	}
	if (std::string::npos != value.find("TRAM")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAM;
	}
	if (std::string::npos != value.find("TRAIN")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_TRAIN;
	}
	if (std::string::npos != value.find("WHEELCHAIR")) {
		return osi3::MovingObject_VehicleClassification_Type_TYPE_WHEELCHAIR;
	}
	return osi3::MovingObject_VehicleClassification_Type_TYPE_UNKNOWN;
}

