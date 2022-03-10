#include "CARLA_OSI_gRPC.h"

#include <limits.h>
#include "Utility.h"
#include "carla_osi/Identifiers.h"

void CARLA_OSI_client::StartServer(const bool nonBlocking)
{
	Logging.open("CARLA-OSI.log");
	Logging << "Start" << std::endl;
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	grpc::ServerBuilder builder;
	builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
	builder.RegisterService(static_cast<CoSiMa::rpc::BaseInterface::Service*>(this));
	builder.RegisterService(static_cast<CoSiMa::rpc::CARLAInterface::Service*>(this));
	builder.RegisterService(&trafficCommandReceiver);
	// try to use unlimited message size
	builder.SetMaxMessageSize(INT_MAX);
	server = builder.BuildAndStart();
	std::cout << "Server listening on " << server_address << std::endl;
	if (!nonBlocking) {
		server->Wait();
	}
	else {
		server_thread = std::make_unique<std::thread>(&grpc::Server::Wait, server);
	}
}

void CARLA_OSI_client::StopServer()
{
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	if (server_thread)
		server_thread->join();
	server = nullptr;
	std::cout << "Server stopped" << std::endl;
	Logging << "Stop" << std::endl;
	Logging.close();
}

grpc::Status CARLA_OSI_client::SetConfig(grpc::ServerContext* context, const CoSiMa::rpc::CarlaConfig* config, CoSiMa::rpc::Int32* response)
{
	for (auto& sensorViewExtra : config->sensor_view_extras()) {
		CoSiMa::rpc::SensorViewSensorMountingPosition mountingPosition;
		mountingPosition.CopyFrom(sensorViewExtra.sensor_mounting_position());
		sensorMountingPositionMap.insert({ sensorViewExtra.prefixed_fmu_variable_name(), mountingPosition });
	}
	response->set_value(
		carlaInterface.initialise(config->carla_host(), config->carla_port(), config->transaction_timeout(), config->delta_seconds(), debug));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response)
{
	if (scenarioRunnerDoesTick) {
		if (!initialDoStep) {
			initialDoStep = false;
			smphSignalSRToCosima.acquire();
		}
		else {
			//Cosima has computed timestep
			smphSignalCosimaToSR.release();
			//Wait for Scenario Runner
			smphSignalSRToCosima.acquire();
		}
		response->set_value(carlaInterface.getDeltaSeconds());
	}
	else {
		response->set_value(carlaInterface.doStep());
	}
	if (logHeartbeat != -1) {
		Logging << "Do step" << std::endl;
		logHeartbeatCounter++;
		if (logHeartbeatCounter >= logHeartbeat) {
			logHeartbeatCounter = 0;
			logEnabled = true;
		}
		else {
			logEnabled = false;
		}
	}
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::Bytes* response)
{
	std::string message = getAndSerialize(request->value());
	response->set_value(message);
	if (logEnabled) {
		Logging << "Out:" << request->value() << ":" << message << std::endl;
	}
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::SetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::NamedBytes* request, CoSiMa::rpc::Int32* response)
{
	if (logEnabled) {
		Logging << "In:" << request->name() << ":" << request->value() << std::endl;
	}
	response->set_value(deserializeAndSet(request->name(), request->value()));
	return grpc::Status::OK;
}

float CARLA_OSI_client::saveTrafficCommand(const osi3::TrafficCommand & command)
{

	trafficCommandForEgoVehicle = std::make_shared<osi3::TrafficCommand>(command);
	if (debug) {
		std::cout << "Set TrafficCommand" << std::endl;
	}

	//Cosima can compute
	smphSignalSRToCosima.release();
	//Cosima has computed timestep
	smphSignalCosimaToSR.acquire();

	return carlaInterface.getDeltaSeconds();
}

std::string_view CARLA_OSI_client::getPrefix(std::string_view name)
{
	// a prefix is surrounded by '#'
	if (2 < name.size() && '#' == name.front()) {
		std::string_view prefix = name.substr(1, name.find('#', 1) - 1);
		return prefix;
	}
	return std::string_view();
}

uint32_t CARLA_OSI_client::getIndex(const std::string_view osmp_name)
{
	if (']' == osmp_name.back()) {
		size_t index = osmp_name.rfind('[');
		//if brackets sourround something, try to parse as index
		if (std::string_view::npos != index && index < osmp_name.size() - 2) {
			auto chars = osmp_name.substr(index + 1, osmp_name.size() - 1);
			uint32_t value = 0;
			// parse as uint32_t
			auto result = std::from_chars(chars.data(), chars.data() + chars.size(), value);
			if (result.ec != std::errc::invalid_argument && result.ec != std::errc::result_out_of_range) {
				return value;
			}
		}
	}
	return 0;
}

int CARLA_OSI_client::deserializeAndSet(const std::string& base_name, const std::string& message) {
	auto prefix = getPrefix(base_name);
	if (0 < prefix.length() && 2 + prefix.length() == base_name.length()) {
		// variable has only a prefix and no name
		std::cerr << __FUNCTION__ << ": Tried to set a variable that has a prefix, but no name (name='" << base_name << "')." << std::endl;
		//TODO do we desire variables that have only a prefix and no name?
		return -2;
	}

	auto varName = std::string_view(&base_name.at(prefix.length() + 2));

	if (std::string::npos != varName.find("TrafficUpdate")) {
		// parse as TrafficUpdate and apply
		osi3::TrafficUpdate trafficUpdate;
		if (!trafficUpdate.ParseFromString(message)) {
			std::cerr << "CARLA2OSIInterface::setStringValue: Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -322;
		}

		carlaInterface.receiveTrafficUpdate(trafficUpdate);
	}
	else {
		//Cache unmapped messages so they can be retrieved as input
		//TODO how to map base_name for retrieval as input?
		varName2MessageMap[base_name] = message;
	}


	//TODO implement
	return 0;
}

std::string CARLA_OSI_client::getAndSerialize(const std::string& base_name) {
	auto prefix = getPrefix(base_name);
	auto varName = std::string_view(&base_name.at(prefix.length() + 2));
	std::shared_ptr<const grpc::protobuf::Message> message;

	if (0 < prefix.length() && 2 + prefix.length() == base_name.length()) {
		// variable has only a prefix and no name
		std::cerr << __FUNCTION__ << ": Tried to get a variable that has a prefix, but no name (name='" << base_name << "')." << std::endl;
		//TODO do we desire variables that have only a prefix and no name?
		//TODO return value or throw?
		return "-2";
	}

	//Test for a specific message type by name and try to retrieve it using the CARLA OSI interface
	if (std::string::npos != varName.rfind("OSMPSensorViewGroundTruth", 0)) {
		//OSMPSensorViewGroundTruth is not a OSMP variable prefix but used as a special name to retrieve a ground truth message as part of sensor view
		message = getSensorViewGroundTruth(base_name);
	}
	else if (std::string::npos != varName.rfind("OSMPSensorView", 0)) {
		// OSMPSensorViewIn
		message = carlaInterface.getSensorView(base_name);
	}
	else if (std::string::npos != varName.rfind("OSMPGroundTruth", 0)) {
		// OSMPGroundTruthInit
		message = carlaInterface.getLatestGroundTruth();
	}
	else if (std::string::npos != varName.rfind("OSMPTrafficCommand", 0)) {
		// OSMPTrafficCommand
		//set hero ID in traffic command message
		trafficCommandForEgoVehicle->mutable_traffic_participant_id()->set_value(carlaInterface.getHeroId());
		message = trafficCommandForEgoVehicle;
	}

	// if the CARLA OSI interface did provide a message, return its string serialization;
	if (message) {
		return message->SerializeAsString();
	}

	// Try lookup in variable cache, else return empty string
	auto iter = varName2MessageMap.find(base_name);
	if (iter != varName2MessageMap.end()) {
		return iter->second;
	}
	else {
		std::cerr << __FUNCTION__ << ": Could not find a variable named " << base_name << " in Carla." << std::endl;
		return "";
	}
}

std::shared_ptr<osi3::SensorView> CARLA_OSI_client::getSensorViewGroundTruth(const std::string& varName) {
	// create empty sensor view
	auto sensorView = std::make_shared<osi3::SensorView>();
	// create empty ground truth as part of sensor view
	auto groundTruth = sensorView->mutable_global_ground_truth();
	// copy latest ground truth into previously created ground truth
	groundTruth->MergeFrom(*carlaInterface.getLatestGroundTruth());

	//host_vehicle_id
	if (groundTruth->has_host_vehicle_id()) {
		sensorView->mutable_host_vehicle_id()->set_value(groundTruth->host_vehicle_id().value());
	}

	// if defined, set sensor mounting positions
	auto iter = sensorMountingPositionMap.find(varName);
	if (sensorMountingPositionMap.end() != iter) {
		if (debug)
		{
			std::cout << "Searched successfully for sensor " << varName << " Copy mounting position to sensorview message." << std::endl;
		}
		copyMountingPositions(iter->second, sensorView);
	}
	else if (debug)
	{
		std::cout << "No sensor found with name: " << varName << " Can not set mounting position.\n";
		if (sensorMountingPositionMap.size() != 0) {
			std::cout << "Available sensors are: ";
			for (auto& positions : sensorMountingPositionMap) {
				std::cout << positions.first << " ";
			}
			std::cout << std::endl;
		}
		else {
			std::cout << "No sensor positions are configured!" << std::endl;
		}
	}

	// find or generate id for the named sensor
	auto id = sensorIds.find(varName);
	if (sensorIds.end() != id) {
		sensorView->mutable_sensor_id()->set_value(id->second);
	}
	else {
		carla_osi::id_mapping::IDUnion sensorId;
		//TODO make sure the type is not defiend in CarlaUtility::CarlaUniqueID_e
		sensorId.type = 100;
		sensorId.id = sensorIds.size() + 1;
		sensorIds[varName] = sensorId.value;
		sensorView->mutable_sensor_id()->set_value(sensorId.value);
	}

	if (debug) {
		printSensorViewMessage(sensorView);
	}
	return sensorView;
}

void CARLA_OSI_client::copyMountingPositions(const CoSiMa::rpc::SensorViewSensorMountingPosition& from, std::shared_ptr<osi3::SensorView> to)
{
	//TODO
	//The virtual mounting position as well as rmse is not set: https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1SensorView.html
	//Is the virtual mounting position needed? Yes!

	if (from.generic_sensor_mounting_position_size()) {
		to->mutable_mounting_position()->CopyFrom(from.generic_sensor_mounting_position(0));
	}
	else if (from.radar_sensor_mounting_position_size()) {
		to->mutable_mounting_position()->CopyFrom(from.radar_sensor_mounting_position(0));
	}
	else if (from.lidar_sensor_mounting_position_size()) {
		to->mutable_mounting_position()->CopyFrom(from.lidar_sensor_mounting_position(0));
	}
	else if (from.camera_sensor_mounting_position_size()) {
		to->mutable_mounting_position()->CopyFrom(from.camera_sensor_mounting_position(0));
	}
	else if (from.ultrasonic_sensor_mounting_position_size()) {
		to->mutable_mounting_position()->CopyFrom(from.ultrasonic_sensor_mounting_position(0));
	}

	/*osi3::Vector3d bb_center_to_rear;//needed if virtual sensor mounting position is measured from bounding box center instead of rear axle
	if (to->has_host_vehicle_id()) {
		for (const auto &moving_object : to->global_ground_truth().moving_object()) {
			if (moving_object.has_id() && moving_object.id().value() == to->host_vehicle_id().value()) {
				if (moving_object.has_vehicle_attributes() && moving_object.vehicle_attributes().has_bbcenter_to_rear()) {
					bb_center_to_rear.CopyFrom(moving_object.vehicle_attributes().bbcenter_to_rear());
				}
				break;
			}
		}
	}
	to->mutable_mounting_position()->mutable_position()->set_x(to->mutable_mounting_position()->mutable_position()->x() - bb_center_to_rear.x());
	to->mutable_mounting_position()->mutable_position()->set_y(to->mutable_mounting_position()->mutable_position()->y() - bb_center_to_rear.y());
	to->mutable_mounting_position()->mutable_position()->set_z(to->mutable_mounting_position()->mutable_position()->z() - bb_center_to_rear.z());
	*/

	//to->mutable_mounting_position_rmse

	//physical sensor mounting position is defined by model itself
	/*for (int i = 0; i < from.generic_sensor_mounting_position_size(); i++) {
		to->add_generic_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.generic_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.radar_sensor_mounting_position_size(); i++) {
		to->add_radar_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.radar_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.lidar_sensor_mounting_position_size(); i++) {
		to->add_lidar_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.lidar_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.camera_sensor_mounting_position_size(); i++) {
		to->add_camera_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.camera_sensor_mounting_position(i));
	}
	for (int i = 0; i < from.ultrasonic_sensor_mounting_position_size(); i++) {
		to->add_ultrasonic_sensor_view()->mutable_view_configuration()->mutable_mounting_position()->CopyFrom(from.ultrasonic_sensor_mounting_position(i));
	}*/
}

void CARLA_OSI_client::printSensorViewMessage(std::shared_ptr<osi3::SensorView> sensorView) {
	std::cout << std::endl << "SensorViewMessage:\n";
	if (sensorView->has_version()) {
		std::cout << "Version: " << sensorView->version().version_major()
			<< "." << sensorView->version().version_minor() << "."
			<< sensorView->version().version_patch() << "\n";
	}
	//timestamp
	//sensor_id
	if (sensorView->has_mounting_position()) {
		std::cout << "   Mounting Position:\n      ";
		printOsiVector(sensorView->mounting_position().position());
		std::cout << "      ";
		printOsiOrientation3d(sensorView->mounting_position().orientation());
	}
	//mountin_position_rmse
	//host_vehicle_data
	//global_ground_truth
	if (sensorView->has_global_ground_truth()) {
		auto groundTruth = sensorView->global_ground_truth();
		//version
		if (groundTruth.has_timestamp()) {
			std::cout << "   Timestamp: " << groundTruth.timestamp().seconds()
				<< "," << groundTruth.timestamp().nanos() << "\n";//Not fully correct
		}
		if (groundTruth.has_host_vehicle_id()) {
			std::cout << "   HostVehicleID: " << groundTruth.host_vehicle_id().value() << "\n";
		}
		if (groundTruth.stationary_object_size()) {
			std::cout << "STATIONARY OBJECTS NOT YET IMPLEMENTED! There are " << groundTruth.stationary_object_size() << " objects.\n";
		}
		if (groundTruth.moving_object_size()) {
			std::cout << "   MovingObjects:\n";
			for (const osi3::MovingObject &movingObject : groundTruth.moving_object()) {
				if (movingObject.has_id()) {
					std::cout << "      ID: " << movingObject.id().value() << "\n";
				}
				if (movingObject.has_base()) {
					auto base = movingObject.base();
					if (base.has_dimension()) {
						std::cout << "         Dimension:\n"
							<< "            Length:" << base.dimension().length()
							<< " Width: " << base.dimension().width()
							<< " Height: " << base.dimension().height() << "\n";
					}
					if (base.has_position()) {
						std::cout << "         Position:\n            ";
						printOsiVector(base.position());
					}
					if (base.has_orientation()) {
						std::cout << "         Orientation:\n            ";
						printOsiOrientation3d(base.orientation());
					}
					if (base.has_velocity()) {
						std::cout << "         Velocity:\n            ";
						printOsiVector(base.velocity());
					}
					if (base.has_acceleration()) {
						std::cout << "         Acceleration:\n            ";
						printOsiVector(base.acceleration());
					}
					if (base.has_orientation_rate()) {
						std::cout << "         Orientation rate:\n            ";
						printOsiOrientation3d(base.orientation_rate());
					}
					if (base.has_orientation_acceleration()) {
						std::cout << "         Orientation Acceleration:\n            ";
						printOsiOrientation3d(base.orientation_acceleration());
					}
					if (base.base_polygon_size()) {
						std::cout << "         Base Polygon Size: " << base.base_polygon_size() << "\n";
					}
				}
				if (movingObject.has_type()) {
					std::cout << "      Type: " << movingObject.type() << "\n";
				}
				//assigned_lane_id
				//vehicle_attributes
				if (movingObject.has_vehicle_attributes()) {
					std::cout << "      Vehicle Attributes:\n";
					if (movingObject.vehicle_attributes().has_driver_id()) {
						std::cout << "         Driver ID: " << movingObject.vehicle_attributes().driver_id().value() << "\n";
					}
					//radius_wheel
					//number_wheels
					if (movingObject.vehicle_attributes().has_bbcenter_to_rear()) {
						std::cout << "         bbcenter_to_rear:\n            ";
						printOsiVector(movingObject.vehicle_attributes().bbcenter_to_rear());
					}
					if (movingObject.vehicle_attributes().has_bbcenter_to_front()) {
						std::cout << "         bbcenter_to_front:\n            ";
						printOsiVector(movingObject.vehicle_attributes().bbcenter_to_front());
					}
					if (movingObject.vehicle_attributes().has_ground_clearance()) {
						std::cout << "         Ground Clearance: " << movingObject.vehicle_attributes().ground_clearance() << "\n";
					}
					//wheel_data
					//steering_wheel_angle
				}
				if (movingObject.has_vehicle_classification()) {
					std::cout << "      Vehicle Classification:\n";
					if (movingObject.vehicle_classification().has_type()) {
						std::cout << "         Type: " << movingObject.vehicle_classification().type() << "\n";
					}
					//light_state
					if (movingObject.vehicle_classification().has_has_trailer()) {
						std::cout << "         Has Trailer: " << movingObject.vehicle_classification().has_trailer() << "\n";
					}
					if (movingObject.vehicle_classification().has_trailer_id()) {
						std::cout << "         TrailerID: " << movingObject.vehicle_classification().trailer_id().value() << "\n";
					}
				}
				//model_reference
				//future_trajetory
				//moving_object_classification
				//source_reference
				//color_description
			}
		}
		if (groundTruth.traffic_sign_size()) {
			std::cout << "   TrafficSign:\n";
			for (const osi3::TrafficSign &trafficSign : groundTruth.traffic_sign()) {
				if (trafficSign.has_id()) {
					std::cout << "      ID: " << trafficSign.id().value() << "\n";
				}
				if (trafficSign.has_main_sign()) {
					std::cout << "      Main Sign:\n";
					if (trafficSign.main_sign().has_base()) {
						std::cout << "         Base:\n";
						if (trafficSign.main_sign().base().has_dimension()) {
							std::cout << "            Dimension:\n"
								<< "               Length:" << trafficSign.main_sign().base().dimension().length()
								<< " Width: " << trafficSign.main_sign().base().dimension().width()
								<< " Height: " << trafficSign.main_sign().base().dimension().height() << "\n";
						}
						if (trafficSign.main_sign().base().has_position()) {
							std::cout << "            Position:\n               ";
							printOsiVector(trafficSign.main_sign().base().position());
						}
						if (trafficSign.main_sign().base().has_orientation()) {
							std::cout << "            Orientation:\n               ";
							printOsiOrientation3d(trafficSign.main_sign().base().orientation());
						}
						if (trafficSign.main_sign().base().base_polygon_size()) {
							std::cout << "            Base Polygon Size: " << trafficSign.main_sign().base().base_polygon_size();
						}
					}
					if (trafficSign.main_sign().has_classification()) {
						std::cout << "         Classification:\n";
						if (trafficSign.main_sign().classification().has_variability()) {
							std::cout << "               Variability: " << trafficSign.main_sign().classification().variability() << "\n";
						}
						if (trafficSign.main_sign().classification().has_type()) {
							std::cout << "               Type: " << trafficSign.main_sign().classification().type() << "\n";
						}
						if (trafficSign.main_sign().classification().has_value()) {
							std::cout << "               Value:\n";
							if (trafficSign.main_sign().classification().value().has_value()) {
								std::cout << "                  Value: " << trafficSign.main_sign().classification().value().value() << "\n";
							}
							if (trafficSign.main_sign().classification().value().has_value_unit()) {
								std::cout << "                  Value Unit: " << trafficSign.main_sign().classification().value().value_unit() << "\n";
							}
							if (trafficSign.main_sign().classification().value().has_text()) {
								std::cout << "                  Text: " << trafficSign.main_sign().classification().value().text() << "\n";
							}
						}
						if (trafficSign.main_sign().classification().has_direction_scope()) {
							std::cout << "               Direction Scope: " << trafficSign.main_sign().classification().direction_scope() << "\n";
						}
						//assigned_lane_id
						//vertically_mirrored
						//is_out_of_service
						//country
						//country_revicion
						//code
						//sub_code
					}
					//model_reference
				}
				//supplementary_sign
				//source_reference
			}
		}
		//traffic_light
		//road_marking
		//lane_boundary
		//lane
		//occupant
		//environmental_conditions
		//country_code
		//proj_string
		//map_reference
		//model_reference
	}
	//generic_sensor_view
	//radar_sensor_view
	//lidar_sensor_view
	//camera_sensor_view
	//ultrasonic_sensor_view
	std::cout << std::endl;
}

void CARLA_OSI_client::printOsiVector(osi3::Vector3d vector3d) {
	std::cout << "X:" << vector3d.x() << " Y:" << vector3d.y() << " Z:" << vector3d.z() << "\n";
}
void CARLA_OSI_client::printOsiOrientation3d(osi3::Orientation3d orientation3d) {
	std::cout << "Roll:" << orientation3d.roll() << " Pitch:" << orientation3d.pitch() << " Yaw:" << orientation3d.yaw() << "\n";
}
