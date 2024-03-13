#include "Utility.h"

osi3::StationaryObject* CarlaUtility::toOSI(const carla::rpc::EnvironmentObject& environmentObject, bool verbose) {
	osi3::StationaryObject* prop = new osi3::StationaryObject();
	osi3::BaseStationary* base = prop->mutable_base();
	osi3::StationaryObject_Classification* classification = prop->mutable_classification();

	auto dimpos = Geometry::getInstance()->toOSI(environmentObject.bounding_box);
	auto& dimension = std::get<0>(dimpos);

	if (!verbose && dimension->length() * dimension->width() * dimension->height() >= 1000) {
		std::cout << "Large volume of stationary object detected. Name: " << environmentObject.name << std::endl;
	}

	if (verbose) {
		std::cout << "OSI-Dimensions: " << dimension->length() << " " << dimension->width() << " " << dimension->height()
			<< " OSI-Position: " << environmentObject.transform.location.x << " " << environmentObject.transform.location.y << " " << environmentObject.transform.location.z
			<< " OSI-Rotation: " << environmentObject.transform.rotation.roll << " " << environmentObject.transform.rotation.pitch << " " << environmentObject.transform.rotation.yaw
			<< " Name: " << environmentObject.name
			<< std::endl;
	}

	base->set_allocated_dimension(dimension.release());
	base->set_allocated_position(Geometry::getInstance()->toOSI(environmentObject.transform.location).release());
	base->set_allocated_orientation(Geometry::getInstance()->toOSI(environmentObject.transform.rotation).release());

	switch (environmentObject.type)
	{
	default:
		//will be set to other if no other tag is available
	case carla::rpc::CityObjectLabel::Pedestrians:
	case carla::rpc::CityObjectLabel::RoadLines://road line
	case carla::rpc::CityObjectLabel::Roads://road
	case carla::rpc::CityObjectLabel::Sidewalks://sidewalks, also includes a possibly delimiting curb, traffic islands (the walkable part), and pedestrian zones
	case carla::rpc::CityObjectLabel::Ground:
	case carla::rpc::CityObjectLabel::Water:
	case carla::rpc::CityObjectLabel::RailTrack:
	case carla::rpc::CityObjectLabel::Static:
	case carla::rpc::CityObjectLabel::Terrain://Grass, ground-level vegetation, soil or sand. These areas are not meant to be driven on. This label includes a possibly delimiting curb.
		//std::cerr << "Encountered an unmappable stationary map object of value " << (int)mapObject.type << std::endl;
		// no break by design
	case carla::rpc::CityObjectLabel::Other://other
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_OTHER);
		break;
	case carla::rpc::CityObjectLabel::Buildings://buildings
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_BUILDING);
		break;
	case carla::rpc::CityObjectLabel::Fences:
	case carla::rpc::CityObjectLabel::GuardRail:
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_BARRIER);
		break;
	case carla::rpc::CityObjectLabel::Poles://poles
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_POLE);
		break;
	case carla::rpc::CityObjectLabel::Vegetation://vegetation, also includes trees (cannot differentiate from StationaryObject_Classification_Type_TYPE_VEGETATION)
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_VEGETATION);
		break;
	case carla::rpc::CityObjectLabel::Walls://walls
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_WALL);
		break;
	case carla::rpc::CityObjectLabel::Bridge:
		classification->set_type(osi3::StationaryObject_Classification_Type_TYPE_BRIDGE);
		break;
	case carla::rpc::CityObjectLabel::None://should have no collision, also should not be returned as part of stationaryObject
	case carla::rpc::CityObjectLabel::Sky:
		//unmapped
		break;
	case carla::rpc::CityObjectLabel::Dynamic://should be parsed as osi3::MovingObject
	case carla::rpc::CityObjectLabel::Vehicles://vehicles should be mapped to osi3::MovingObject, even though the corresponding StationaryObject returned by Carla will never move
	case carla::rpc::CityObjectLabel::TrafficSigns://traffic signs without their poles are part of osi3::TrafficSign
	case carla::rpc::CityObjectLabel::TrafficLight://traffic light boxes without their poles are part of osi3::TrafficLight
		//TODO Parse as respective Type (see previous comments)
		break;
	}

	prop->set_allocated_id(carla_osi::id_mapping::toOSI(environmentObject.id).release());

	prop->set_model_reference(environmentObject.name);
	return prop;
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Actor> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	// but bounding boxes only exist for Vehicle and Walker specializations
	base->set_allocated_position(Geometry::getInstance()->toOSI(transform.location).release());
	base->set_allocated_orientation(Geometry::getInstance()->toOSI(transform.rotation).release());

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
	auto dimpos = Geometry::getInstance()->toOSI(actor->GetBoundingBox());
	auto& dimension = std::get<0>(dimpos);
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations - assume bbox is not rotated
	base->set_allocated_position(Geometry::getInstance()->toOSI(transform.location).release());
	base->set_allocated_orientation(Geometry::getInstance()->toOSI(transform.rotation).release());

	return CarlaUtility::toOSIBaseMoving_common(actor, std::move(base));
}

std::unique_ptr<osi3::BaseMoving> CarlaUtility::toOSIBaseMoving(const carla::SharedPtr<const carla::client::Vehicle> actor)
{
	auto base = std::make_unique<osi3::BaseMoving>();
	auto transform = actor->GetTransform();
	// transform.location might not match the bounding box center if bounding box is not located at origin (in local coordinates)
	auto bbox = actor->GetBoundingBox();
	// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in carla_osi::geometry::toOSI
	auto dimpos = Geometry::getInstance()->toOSI(bbox);
	auto& dimension = std::get<0>(dimpos);
	base->set_allocated_dimension(dimension.release());
	transform.location += bbox.location;
	//TODO libCarla_client has no arithmetics for rotations, but
	// vehicle bounding boxes shouldn't be rotated
	base->set_allocated_position(Geometry::getInstance()->toOSI(transform.location).release());
	base->set_allocated_orientation(Geometry::getInstance()->toOSI(transform.rotation).release());


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
	base->set_allocated_velocity(Geometry::getInstance()->toOSIVelocity(actor->GetVelocity()).release());
	base->set_allocated_acceleration(Geometry::getInstance()->toOSIVelocity(actor->GetAcceleration()).release());
	auto angularVelocity = actor->GetAngularVelocity();//Carla uses Vector3d instead of Rotation as type
	base->set_allocated_orientation_rate(Geometry::getInstance()->toOSI(
		carla::geom::Rotation(angularVelocity.y, angularVelocity.z, angularVelocity.x)).release());

	//TODO Carla has no rotational acceleration
	//base->set_allocated_orientation_acceleration

	return base;
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

osi3::CameraSensorView* CarlaUtility::toOSICamera(const carla::SharedPtr<const carla::client::Sensor> sensor,
	const carla::SharedPtr<const carla::sensor::SensorData> sensorData, const OSTARSensorConfiguration& sensorConfig)
{
	auto image = boost::dynamic_pointer_cast<const carla::sensor::data::Image>(sensorData);
	//Contains RGBA uint8 values
	if (!image) return nullptr;
	auto height = image->GetHeight();
	auto width = image->GetWidth();
	double aspect = ((double)width) / ((double)height);
	assert(height * width == image->size());

	/// CARLA provides images with alpha channel, but OSI only has channel formats without alpha channel -> drop transparency

	//Boost GIL image view for color conversion
	const carla::sensor::data::ImageTmpl<carla::sensor::data::Color>& ref = *image;
	const boost::gil::bgra8c_view_t rgbaView = carla::image::ImageView::MakeView(ref);
		/*=boost::gil::interleaved_view(width,
		height, reinterpret_cast<boost::gil::rgba8c_ptr_t>(image->data()),
		sizeof(carla::sensor::data::Image::value_type)  * width);*/

	//Buffer for RGB raw image
	boost::gil::rgb8_ptr_t rgb = new boost::gil::rgb8_pixel_t[image->size()];
	//Boost GIL image view for rgb buffer
	boost::gil::rgb8_view_t rgbView = boost::gil::interleaved_view((std::ptrdiff_t)width,
		(std::ptrdiff_t)height, rgb,
		sizeof(boost::gil::rgb8_pixel_t)  * width);

	// perform the actual conversion
	boost::gil::copy_and_convert_pixels(rgbaView, rgbView);

	//Debug
	//boost::gil::write_view("CARLA_camera_image_" + std::to_string(rand()) + ".png", rgbView, boost::gil::png_tag());

	// Fill OSI CameraSensorView Message
	osi3::CameraSensorView* cameraSensorView = new osi3::CameraSensorView();
	cameraSensorView->set_image_data(rgb, image->size() * sizeof(boost::gil::rgb8_pixel_t));

	auto config = cameraSensorView->mutable_view_configuration();
	config->add_channel_format(osi3::CameraSensorViewConfiguration_ChannelFormat_CHANNEL_FORMAT_RGB_U8_LIN);
	// Carla only reports horizontal field of view
	auto fov = image->GetFOVAngle() * M_PI / 180.0;
	config->set_field_of_view_horizontal(fov);
	// guess vertical fov based on aspect ratio
	config->set_field_of_view_vertical(fov / aspect);
	config->set_number_of_pixels_horizontal(width);
	config->set_number_of_pixels_vertical(height);
	config->set_samples_per_pixel(1);
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());

	//TODO calculate sensor position in vehicle coordinates
	config->set_allocated_mounting_position(Geometry::getInstance()->toOSI(sensor->GetTransform()).release());
	// not given in CARLA
	//config->set_allocated_mounting_position_rmse(rmse)

	return cameraSensorView;
}

osi3::LidarSensorView* CarlaUtility::toOSILidar(const carla::SharedPtr<const carla::client::Sensor> sensor,
	const carla::SharedPtr<const carla::sensor::SensorData> sensorData, const OSTARSensorConfiguration& sensorConfig)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::SemanticLidarMeasurement>(sensorData);
	double rotationFrequency, upperFov, lowerFov, range, horizontalFoV, sensorTick;
	int pointsPerSecond, channels;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("rotation_frequency" == attribute.GetId()) {
			rotationFrequency = std::stod(attribute.GetValue());
		}
		else if ("upper_fov" == attribute.GetId()) {
			upperFov = std::stod(attribute.GetValue());
		}
		else if ("lower_fov" == attribute.GetId()) {
			lowerFov = std::stod(attribute.GetValue());
		}
		else if ("horizontal_fov" == attribute.GetId()) {
			horizontalFoV = std::stod(attribute.GetValue());
		}
		else if ("range" == attribute.GetId()) {
			range = std::stod(attribute.GetValue());
		}
		else if ("sensor_tick" == attribute.GetId()) {
			sensorTick = std::stod(attribute.GetValue());
		}
		else if ("points_per_second" == attribute.GetId()) {
			pointsPerSecond = std::stoi(attribute.GetValue());
		}
		else if ("channels" == attribute.GetId()) {
			channels = std::stoi(attribute.GetValue());
		}
	}
	double vFov;
	if (upperFov && lowerFov) {
		//upper and lower field of view are given in degree
		vFov = upperFov - lowerFov;
	}
	uint32_t numPixels{0};//OSI field uses uint32_t
	for (size_t i = 0; i < measurement->GetChannelCount(); i++) {
		numPixels += measurement->GetPointCount(i);
	}

	double senderHz = OSTAR_LIDAR_DEFAULT_HZ;
	if (sensorConfig.sensorViewConfiguration.lidar_sensor_view_configuration().size()
		&& sensorConfig.sensorViewConfiguration.lidar_sensor_view_configuration(0).has_emitter_frequency())
		senderHz = sensorConfig.sensorViewConfiguration.lidar_sensor_view_configuration(0).emitter_frequency();

	osi3::LidarSensorView* lidarSensorView = new osi3::LidarSensorView();

	auto config = lidarSensorView->mutable_view_configuration();
	
	config->set_emitter_frequency(senderHz);
	config->set_field_of_view_vertical(vFov * M_PI / 180.0);
	config->set_field_of_view_horizontal(horizontalFoV * M_PI / 180.0);
	config->set_max_number_of_interactions(1);

	//TODO OSI expects a constant number of pixels per message, but Carla only reports new values of the angle sweeped during the last frame
	config->set_num_of_pixels(numPixels);
	//TODO number of rays (horizontal/vertical) of lidar
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());

	//TODO calculate sensor position in vehicle coordinates
	config->set_allocated_mounting_position(Geometry::getInstance()->toOSI(sensor->GetTransform()).release());
	// not given in CARLA
	//config->set_allocated_mounting_position_rmse(rmse)

	std::unordered_set<float> horizontal, vertical;
	for (const carla::sensor::data::SemanticLidarDetection& singlemeasure : *measurement)
	{
		auto* reflection = lidarSensorView->add_reflection();
		//TODO reflection->set_signal_strength(singlemeasure.intensity);
		double distance = std::sqrt((double) singlemeasure.point.x * (double)singlemeasure.point.x
			+ (double) singlemeasure.point.y * (double) singlemeasure.point.y
			+ (double) singlemeasure.point.z * (double) singlemeasure.point.z);
		reflection->set_time_of_flight((2 * distance) / OSTAR_LIGHT_SPEED);//double distance / lightspeed in m/s
		//double receiverHz = senderHz * (OSTAR_LIGHT_SPEED + singlemeasure.velocity) / (OSTAR_LIGHT_SPEED - singlemeasure.velocity);
		//reflection->set_doppler_shift(receiverHz - senderHz);

		//TODO singlemeasure.cos_inc_angle;
		//osi3::Vector3d* normalToSurface = new osi3::Vector3d;
		//normalToSurface->set_x(singlemeasure.point.x);
		//normalToSurface->set_y(singlemeasure.point.y);
		//normalToSurface->set_z(singlemeasure.point.z);
		//reflection->set_allocated_normal_to_surface(normalToSurface);
		osi3::Identifier* identifier = new osi3::Identifier();
		identifier->set_value(singlemeasure.object_idx);
		reflection->set_allocated_object_id(identifier);
		//horizontal.insert(singlemeasure.);
		//vertical.insert(singlemeasure.);
	}

	//config->set_number_of_rays_horizontal((uint32_t)horizontal.size());
	//config->set_number_of_rays_vertical((uint32_t)vertical.size());

	return lidarSensorView;
}

osi3::RadarSensorView* CarlaUtility::toOSIRadar(const carla::SharedPtr<const carla::client::Sensor> sensor,
	const carla::SharedPtr<const carla::sensor::SensorData> sensorData, const OSTARSensorConfiguration& sensorConfig)
{
	auto measurement = boost::dynamic_pointer_cast<const carla::sensor::data::RadarMeasurement>(sensorData);
	double hFov, vFov, range, sensorTick;
	int pointsPerSecond;
	auto attributes = sensor->GetAttributes();
	for (auto attribute : attributes) {
		if ("horizontal_fov" == attribute.GetId()) {
			hFov = std::stod(attribute.GetValue());
		}
		else if ("vertical_fov" == attribute.GetId()) {
			vFov = std::stod(attribute.GetValue());
		}
		else if ("range" == attribute.GetId()) {
			range = std::stod(attribute.GetValue());
		}
		else if ("sensor_tick" == attribute.GetId()) {
			sensorTick = std::stod(attribute.GetValue());
		}
		else if ("points_per_second" == attribute.GetId()) {
			pointsPerSecond = std::stoi(attribute.GetValue());
		}
	}

	double senderHz = OSTAR_RADAR_DEFAULT_HZ;
	if (sensorConfig.sensorViewConfiguration.radar_sensor_view_configuration().size()
		&& sensorConfig.sensorViewConfiguration.radar_sensor_view_configuration(0).has_emitter_frequency())
		senderHz = sensorConfig.sensorViewConfiguration.radar_sensor_view_configuration(0).emitter_frequency();

	auto radarSensorview = new osi3::RadarSensorView();
	
	auto config = radarSensorview->mutable_view_configuration();
	config->set_allocated_sensor_id(carla_osi::id_mapping::getOSIActorId(sensor).release());
	config->set_field_of_view_horizontal(hFov);
	config->set_field_of_view_vertical(vFov);
	config->set_max_number_of_interactions(1);
	config->set_emitter_frequency(senderHz);// Hz to Hz -> no conversion

	//TODO number of rays (horizontal/vertical)
	//TODO rx and tx antenna diagrams
	//TODO calculate sensor position in vehicle coordinates, that is relative to the vehicles rear

	//TODO calculate sensor position in vehicle coordinates
	config->set_allocated_mounting_position(Geometry::getInstance()->toOSI(sensor->GetTransform()).release());
	// not given in CARLA
	//config->set_allocated_mounting_position_rmse(rmse)

	std::unordered_set<float> horizontal, vertical;
	for (const auto& singlemeasure : *measurement)
	{
		auto* reflection = radarSensorview->add_reflection();
		reflection->set_source_horizontal_angle(singlemeasure.azimuth);//rad to rad -> no conversion
		reflection->set_source_vertical_angle(singlemeasure.altitude);//rad to rad -> no conversion
		double receiverHz = senderHz * (OSTAR_LIGHT_SPEED + singlemeasure.velocity) / (OSTAR_LIGHT_SPEED - singlemeasure.velocity);
		reflection->set_doppler_shift(receiverHz - senderHz);
		reflection->set_signal_strength(10 * range / singlemeasure.depth);//TODO check if signal strength estimation makes sense
		reflection->set_time_of_flight((2 * singlemeasure.depth) / OSTAR_LIGHT_SPEED);//double distance / lightspeed in m/s

		horizontal.insert(singlemeasure.azimuth);
		vertical.insert(singlemeasure.altitude);
	}

	config->set_number_of_rays_horizontal((uint32_t)horizontal.size());
	config->set_number_of_rays_vertical((uint32_t)vertical.size());

	return radarSensorview;
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

std::unique_ptr<osi3::Timestamp> CarlaUtility::parseTimestamp(const carla::client::Timestamp& carlaTime)
{
	std::unique_ptr<osi3::Timestamp> osiTime = std::make_unique<osi3::Timestamp>();
	double intPart;
	double fractional = std::modf(carlaTime.elapsed_seconds, &intPart);
	osiTime->set_seconds(google::protobuf::int64(intPart));
	osiTime->set_nanos(google::protobuf::uint32(fractional *1e9));
	return osiTime;
}

std::string CarlaUtility::findBestMatchingCarToSpawn(const osi3::Dimension3d& dimension,
	const std::vector<std::tuple<std::string, carla::geom::Vector3D>>& replayVehicleBoundingBoxes,
	double& weightLength_X, double& weightWidth_Y, double& weightHeight_Z){

	size_t minDiffVehicleIndex = 0;

	double minTotalDiff = DBL_MAX;
	double minTotalDiffLength = 0, minTotalDiffWidth = 0, minTotalDiffHeight = 0;

	for (int i = 0; i < replayVehicleBoundingBoxes.size(); i++) {
		auto& boundingBox = std::get<1>(replayVehicleBoundingBoxes[i]);

		double diffLength = dimension.length() - (2 * boundingBox.x);
		double diffWidth = dimension.width() - (2 * boundingBox.y);
		double diffHeight = dimension.height() - (2 * boundingBox.z);

		double sumDiff = weightLength_X * std::abs(diffLength);
		sumDiff += weightWidth_Y * std::abs(diffWidth);
		sumDiff += weightHeight_Z * std::abs(diffHeight);

		if (sumDiff < minTotalDiff) {
			minDiffVehicleIndex = i;
			minTotalDiff = sumDiff;

			minTotalDiffLength = diffLength;
			minTotalDiffWidth = diffWidth;
			minTotalDiffHeight = diffHeight;
		}
	}

	std::cout << "Search for vehicle with length: " << dimension.length() << ", width: " << dimension.width()
		<< ", height: " << dimension.height()
		<< " Spawn vehicle with length: " << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).x * 2
		<< ", width:" << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).y * 2
		<< ", height:" << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).z * 2 << std::endl;

	return std::get<0>(replayVehicleBoundingBoxes[minDiffVehicleIndex]);
}
