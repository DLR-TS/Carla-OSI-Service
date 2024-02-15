#include "CARLA_OSI_gRPC.h"

void CARLA_OSI_client::StartServer(const bool nonBlocking)
{
	if (server)
		server->Shutdown(std::chrono::system_clock::now() + transaction_timeout);
	grpc::ServerBuilder builder;
	grpc::reflection::InitProtoReflectionServerBuilderPlugin();
	builder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
	builder.RegisterService(static_cast<CoSiMa::rpc::BaseInterface::Service*>(this));
	builder.RegisterService(static_cast<CoSiMa::rpc::CARLAInterface::Service*>(this));
	builder.RegisterService(&trafficCommandReceiver);
	// try to use unlimited message size
	builder.SetMaxMessageSize(INT_MAX);
	server = builder.BuildAndStart();
	std::cout << "Server listening on " << serverAddress << std::endl;
	if (!nonBlocking) {
		server->Wait();
	}
	else {
		server_thread = std::make_unique<std::thread>(&grpc::Server::Wait, server);
	}
}

void CARLA_OSI_client::watchdog(CARLA_OSI_client* client) {
	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(client->runtimeParameter.resumeCarlaAsyncSeconds));
		if (client->watchdogInitDone && !client->watchdogDoStepCalled) {
			std::cout << "Reset Carla mode by watchdog because of no activity." << std::endl;
			client->trafficUpdater->deleteSpawnedVehicles();
			client->carla->resetWorldSettings();
			exit(0);
		}
		else {
			if (client->watchdogInitDone) {
				client->watchdogDoStepCalled = false;
			}
		}
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
}

grpc::Status CARLA_OSI_client::SetConfig(grpc::ServerContext* context, const CoSiMa::rpc::CarlaConfig* config, CoSiMa::rpc::Int32* response)
{
	//parse configuration
	bool cityObjectLabelFilterSet = false;

	for (int i = 0; i < config->runtimeparameter_size(); i++) {
		std::string parameter = config->runtimeparameter(i);

		if (parameter == "-v" || parameter == "--verbose") {
			runtimeParameter.verbose = true;
			std::cout << "Running with verbose prints.\n";
		}
		else if (parameter == "-sr" || parameter == "--scenariorunner") {
			runtimeParameter.scenarioRunnerDoesTick = true;
			std::cout << "Wait for scenario runner connection.\n";
		}
		else if (parameter == "-a" || parameter == "--async") {
			runtimeParameter.sync = false;
			std::cout << "Running in asynchronous mode.\n";
		}
		else if (parameter == "-l" || parameter == "--logfile") {
			runtimeParameter.log = true;
			runtimeParameter.logFileName = config->runtimeparameter(++i);
			std::cout << "Log to std::cout and " << runtimeParameter.logFileName << "\n";
		}
		else if (parameter == "-ego") {
			runtimeParameter.ego = config->runtimeparameter(++i);
			std::cout << "Ego: " << runtimeParameter.ego << std::endl;
		}
		else if (parameter == "-replay") {
			runtimeParameter.replay.enabled = true;
			std::cout << "Replay mode active. If not further defined by -replayWeights or -replayMapOffsets default values are used." << std::endl;
		}
		else if (parameter == "-replayWeights") {
			runtimeParameter.replay.enabled = true;
			runtimeParameter.replay.weightLength_X = std::stod(config->runtimeparameter(++i));
			runtimeParameter.replay.weightWidth_Y = std::stod(config->runtimeparameter(++i));
			runtimeParameter.replay.weightHeight_Z = std::stod(config->runtimeparameter(++i));
			std::cout << "Replay mode active. Similarity weights are: " << runtimeParameter.replay.weightLength_X << ", "
				 << runtimeParameter.replay.weightWidth_Y << ", " << runtimeParameter.replay.weightHeight_Z << std::endl;
		}
		else if (parameter == "-replaySpawnCarByName") {
			runtimeParameter.replay.enabled = true;
			runtimeParameter.replay.spawnCarByName = config->runtimeparameter(++i);
			std::cout << "Replay mode active. Spawn all cars with model name: " << runtimeParameter.replay.spawnCarByName << std::endl;
		}
		else if (parameter == "-replayMapOffsets") {
			runtimeParameter.replay.enabled = true;
			runtimeParameter.replay.mapOffset.X = std::stod(config->runtimeparameter(++i));
			runtimeParameter.replay.mapOffset.Y = std::stod(config->runtimeparameter(++i));
			Geometry::getInstance()->setOffset(runtimeParameter.replay.mapOffset);
			std::cout << "Replay mode active. Map offsets are: " << runtimeParameter.replay.mapOffset.X << ", "
				 << runtimeParameter.replay.mapOffset.Y << std::endl;
		}
		else if (parameter == "-replayOutputUTM") {
			runtimeParameter.replay.enabled = true;
			runtimeParameter.replay.UTMOutput = true;
			Geometry::getInstance()->setOSI_UTM(runtimeParameter.replay.UTMOutput);
			std::cout << "Replay mode active. UTM Output set to true." << std::endl;
		}
		else if (parameter == "-replaySpawnHeight") {
			runtimeParameter.replay.enabled = true;
			runtimeParameter.replay.spawnHeight_Z = std::stof(config->runtimeparameter(++i));
			std::cout << "Replay mode active. Spawn height is: " << runtimeParameter.replay.spawnHeight_Z << std::endl;
		}
		else if (parameter == "--filterbyname") {
			runtimeParameter.filter = true;
			runtimeParameter.filterString = config->runtimeparameter(++i);
			std::cout << "Filterbyname for static objects active. Use: " << runtimeParameter.filterString << "\n";
		}
		else if (parameter == "--resumeafter" || parameter == "--maxresponseinterval") {
			runtimeParameter.resumeCarlaAsyncSeconds = std::stoi(config->runtimeparameter(++i));
			std::cout << "Max response interval for carla (anti - freeze) after seconds: " << runtimeParameter.resumeCarlaAsyncSeconds << "\n";
		}
		else if (parameter == "--camera") {
			runtimeParameter.carlaSensors = true;
			runtimeParameter.carlasensortypes.emplace(CAMERA);
			std::cout << "Use camera listeners on sensors spawned in CARLA.\n";
		}
		else if (parameter == "--lidar") {
			runtimeParameter.carlaSensors = true;
			runtimeParameter.carlasensortypes.emplace(LIDAR);
			std::cout << "Use lidar listeners on sensors spawned in CARLA.\n";
		}
		else if (parameter == "--radar") {
			runtimeParameter.carlaSensors = true;
			runtimeParameter.carlasensortypes.emplace(RADAR);
			std::cout << "Use radar listeners on sensors spawned in CARLA.\n";
		}
		else if (parameter == "--ultrasonic") {
			runtimeParameter.carlaSensors = true;
			runtimeParameter.carlasensortypes.emplace(ULTRASONIC);
			std::cout << "Use ultrasonic listeners on sensors spawned in CARLA.\n";
		}
		else if (parameter == "--cityobjectlabel") {
			cityObjectLabelFilterSet = true;
			std::string cityObjectLabelFilter = config->runtimeparameter(++i);
			if (cityObjectLabelFilter.find("None") != std::string::npos) runtimeParameter.options.None = true;
			if (cityObjectLabelFilter.find("Buildings") != std::string::npos) runtimeParameter.options.Buildings = true;
			if (cityObjectLabelFilter.find("Fences") != std::string::npos) runtimeParameter.options.Fences = true;
			if (cityObjectLabelFilter.find("Other") != std::string::npos) runtimeParameter.options.Other = true;
			if (cityObjectLabelFilter.find("Poles") != std::string::npos) runtimeParameter.options.Poles = true;
			if (cityObjectLabelFilter.find("RoadLines") != std::string::npos) runtimeParameter.options.RoadLines = true;
			if (cityObjectLabelFilter.find("Roads") != std::string::npos) runtimeParameter.options.Roads = true;
			if (cityObjectLabelFilter.find("Sidewalks") != std::string::npos) runtimeParameter.options.Sidewalks = true;
			if (cityObjectLabelFilter.find("TrafficSigns") != std::string::npos) runtimeParameter.options.TrafficSigns = true;
			if (cityObjectLabelFilter.find("Vegetation") != std::string::npos) runtimeParameter.options.Vegetation = true;
			if (cityObjectLabelFilter.find("Walls") != std::string::npos) runtimeParameter.options.Walls = true;
			if (cityObjectLabelFilter.find("Ground") != std::string::npos) runtimeParameter.options.Ground = true;
			if (cityObjectLabelFilter.find("Bridge") != std::string::npos) runtimeParameter.options.Bridge = true;
			if (cityObjectLabelFilter.find("RailTrack") != std::string::npos) runtimeParameter.options.RailTrack = true;
			if (cityObjectLabelFilter.find("GuardRail") != std::string::npos) runtimeParameter.options.GuardRail = true;
			if (cityObjectLabelFilter.find("TrafficLight") != std::string::npos) runtimeParameter.options.TrafficLight = true;
			if (cityObjectLabelFilter.find("Static") != std::string::npos) runtimeParameter.options.Static = true;
			if (cityObjectLabelFilter.find("Water") != std::string::npos) runtimeParameter.options.Water = true;
			if (cityObjectLabelFilter.find("Terrain") != std::string::npos) runtimeParameter.options.Terrain = true;
			if (cityObjectLabelFilter.find("Any") != std::string::npos) runtimeParameter.options.Any = true;
		}
		else if (parameter == "--mapnetwork") {
			runtimeParameter.mapNetworkInGroundTruth = true;
			std::cout << "Run with map network in ground truth messages.\n";
		}
		else if (parameter == "--carlahost") {
			runtimeParameter.carlaHost = config->runtimeparameter(++i);
			std::cout << "Carla host: " << runtimeParameter.carlaHost  << "\n";
		}
		else if (parameter == "--carlaport") {
			runtimeParameter.carlaPort = std::stoi(config->runtimeparameter(++i));
			std::cout << "Carla port: " << runtimeParameter.carlaPort << "\n";
		}
		else if (parameter == "--transactiontimeout") {
			runtimeParameter.transactionTimeout = std::stof(config->runtimeparameter(++i));
			std::cout << "Transaction timeout: " << runtimeParameter.transactionTimeout << "\n";
		}
		else if (parameter == "--deltaseconds") {
			runtimeParameter.deltaSeconds = std::stof(config->runtimeparameter(++i));
			std::cout << "Delta seconds: " << runtimeParameter.deltaSeconds << "\n";
		}
		else {
			std::cout << "Unkown parameter: " << parameter << "\n";
		}
	}

	//set options to any if no filter is set by user
	if (cityObjectLabelFilterSet) {
		std::cout << "CityObjectLabel Filter: Any\n";
		runtimeParameter.options.Any = true;
	}
	std::cout << std::endl;

	//set configuration
	if (runtimeParameter.resumeCarlaAsyncSeconds != 0) {//option is active
		watchdog_thread = std::make_unique<std::thread>(&CARLA_OSI_client::watchdog, this);
	}
	//check for sensors configured by cosima configuration
	for (auto& sensorViewExtra : config->sensor_view_extras()) {
		sensorViewConfiger->sensorsByUser.push_back(toSensorDescriptionInternal(sensorViewExtra));
	}

	response->set_value(carla->initialise(runtimeParameter));
	trafficUpdater->initialise(runtimeParameter, carla);
	sensorViewer->initialise(runtimeParameter, carla);
	sensorViewConfiger->initialise(runtimeParameter, carla);
	logger->initialise(runtimeParameter, carla);

	sensorViewConfiger->trySpawnSensors(sensorViewer);
	sensorViewer->groundTruthCreator->parseStationaryMapObjects();

	carla->doStep();

	if (runtimeParameter.scenarioRunnerDoesTick) {
		std::cout << "Waiting for connetion of scenario runner." << std::endl;
		smphSignalSRToCosima.acquire();
		//data could be changed by a new map loaded by the scenario runner
		carla->loadWorld();
		sensorViewer->groundTruthCreator->parseStationaryMapObjects();
	}

	return grpc::Status::OK;
}

//BEGIN OF CoSiMa::rpc::BaseInterface::Service

grpc::Status CARLA_OSI_client::DoStep(grpc::ServerContext* context, const CoSiMa::rpc::Empty* request, CoSiMa::rpc::Double* response)
{
	watchdogInitDone = true;
	watchdogDoStepCalled = true;

	if (runtimeParameter.log) {
		logger->writeLog(sensorViewer->groundTruthCreator->getLatestGroundTruth());
	}

	sensorViewConfiger->trySpawnSensors(sensorViewer);

	if (runtimeParameter.scenarioRunnerDoesTick) {
		//Cosima has computed timestep
		smphSignalCosimaToSR.release();
		//wait for Scenario Runner
		smphSignalSRToCosima.acquire();

		//update changes in carla
		sensorViewer->fetchActorsFromCarla();
		response->set_value(runtimeParameter.deltaSeconds);
	}
	else
	{
		//independent mode without scenario runner
		auto timestep = carla->doStep();
		sensorViewer->groundTruthCreator->invalidateLatestGroundTruth();
		//update changes in carla
		sensorViewer->fetchActorsFromCarla();
		response->set_value(timestep);
	}

	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::GetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::String* request, CoSiMa::rpc::Bytes* response)
{
	response->set_value(getAndSerialize(request->value()));
	return grpc::Status::OK;
}

grpc::Status CARLA_OSI_client::SetStringValue(grpc::ServerContext* context, const CoSiMa::rpc::NamedBytes* request, CoSiMa::rpc::Int32* response)
{
	response->set_value(deserializeAndSet(request->name(), request->value()));
	return grpc::Status::OK;
}

std::string CARLA_OSI_client::getAndSerialize(const std::string& base_name) {
	if (runtimeParameter.verbose)
	{
		std::cout << "Get " << base_name << std::endl;
	}

	std::shared_ptr<const grpc::protobuf::Message> message;

	//Test for a specific message type by name and try to retrieve it using the CARLA OSI interface
	if (std::string::npos != base_name.rfind("OSMPSensorViewGroundTruth", 0)) {
		//OSMPSensorViewGroundTruth is not a OSMP variable prefix but used as a special name to retrieve a ground truth message as part of sensor view
		message = sensorViewer->getSensorViewGroundTruth(base_name);
	}
	else if (std::string::npos != base_name.rfind("OSMPSensorViewConfiguration", 0)) {
		message = sensorViewConfiger->getLastSensorViewConfiguration();
	}
	else if (std::string::npos != base_name.rfind("OSMPGroundTruth", 0)) {
		// OSMPGroundTruthInit
		message = sensorViewer->groundTruthCreator->getLatestGroundTruth();
	}
	else if (std::string::npos != base_name.rfind("OSMPTrafficCommand", 0)) {
		// OSMPTrafficCommand
		//set hero ID in traffic command message
		if (trafficCommandForEgoVehicle == nullptr) {
			std::cout << "No OSMPTrafficCommand available. Use -sr parameter to enable scenario runner listener." << std::endl;
		} else {
			message = trafficCommandForEgoVehicle;
		}
	} else {
		// OSMPSensorView by spawned sensor
		message = sensorViewer->getSensorView(base_name);
	}

	// if the CARLA OSI interface did provide a message, return its string serialization;
	if (message) {
		return message->SerializeAsString();
	}

	// Try lookup in variable cache, else return empty string
	// Variables from other fmus are saved and exchanged here!
	if (runtimeParameter.verbose)
	{
		std::cout << "Look up message in map." << std::endl;
	}
	auto iter = varName2MessageMap.find(base_name);
	if (iter != varName2MessageMap.end()) {
		return iter->second;
	}
	else {
		std::cerr << __FUNCTION__ << ": Could not find a variable named " << base_name << " in Carla." << std::endl;
		return "";
	}
}

int CARLA_OSI_client::deserializeAndSet(const std::string& base_name, const std::string& message) {
	if (runtimeParameter.verbose)
	{
		std::cout << "Set " << base_name << std::endl;
	}
	if (std::string::npos != base_name.find("TrafficUpdate")) {
		osi3::TrafficUpdate trafficUpdate;
		if (!trafficUpdate.ParseFromString(message)) {
			std::cerr << "Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -1;
		}
		trafficUpdater->receiveTrafficUpdate(trafficUpdate);
		return 0;
	}
	else if (std::string::npos != base_name.find("OSMPSensorViewConfigurationRequest")) {
		osi3::SensorViewConfiguration sensorViewConfiguration;
		if (!sensorViewConfiguration.ParseFromString(message)) {
			std::cerr << "Variable name'" << base_name << "' indicates this is a SensorViewConfiguration, but parsing failed." << std::endl;
			return -1;
		}
		//return value will be added to request of SensorView
		sensorViewConfiger->sensorsByFMU.push_back(toSensorDescriptionInternal(sensorViewConfiguration));
		return 0;
	}
	else {
		//Cache unmapped messages so they can be retrieved from CoSiMa as input for other fmus.
		varName2MessageMap[base_name] = message;
		return 0;
	}
}

//END OF CoSiMa::rpc::BaseInterface::Service

//BEGIN OF ::srunner::osi::client::OSIVehicleController::Service

float CARLA_OSI_client::saveTrafficCommand(const osi3::TrafficCommand & command)
{

	if (runtimeParameter.verbose) {
		std::cout << __FUNCTION__ << std::endl;
	}
	trafficCommandForEgoVehicle = std::make_shared<osi3::TrafficCommand>(command);
	trafficCommandForEgoVehicle->mutable_traffic_participant_id()->set_value(trafficCommander->getHeroId());

	//Cosima can compute
	smphSignalSRToCosima.release();
	//Cosima has computed timestep
	smphSignalCosimaToSR.acquire();

	if (runtimeParameter.verbose) {
		std::cout << "Send delta to scenario runner: " << runtimeParameter.deltaSeconds << std::endl;
	}

	//control is given back to the scenario runner.
	//The state of the simulation can change.
	//The cached ground truth is now invalid.
	sensorViewer->groundTruthCreator->invalidateLatestGroundTruth();
	return runtimeParameter.deltaSeconds;
}

//END OF ::srunner::osi::client::OSIVehicleController::Service

Sensor CARLA_OSI_client::toSensorDescriptionInternal(osi3::SensorViewConfiguration& sensorViewConfiguration) {
	runtimeParameter.carlaSensors = true;

	Sensor sensor;
	sensor.sensorViewConfiguration.CopyFrom(sensorViewConfiguration);
	sensor.id = sensorViewConfiguration.sensor_id().value();

	//save all mounting bositions in base. Only one sensor possible
	if (sensorViewConfiguration.generic_sensor_view_configuration_size()) {
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.generic_sensor_view_configuration()[0].mounting_position());
		runtimeParameter.carlasensortypes.emplace(GENERIC);
		sensor.type = GENERIC;
	}
	else if (sensorViewConfiguration.radar_sensor_view_configuration_size()) {
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.radar_sensor_view_configuration()[0].mounting_position());
		runtimeParameter.carlasensortypes.emplace(RADAR);
		sensor.type = RADAR;
	}
	else if (sensorViewConfiguration.lidar_sensor_view_configuration_size()) {
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.lidar_sensor_view_configuration()[0].mounting_position());
		runtimeParameter.carlasensortypes.emplace(LIDAR);
		sensor.type = LIDAR;
	}
	else if (sensorViewConfiguration.camera_sensor_view_configuration_size()) {
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.camera_sensor_view_configuration()[0].mounting_position());
		runtimeParameter.carlasensortypes.emplace(CAMERA);
		sensor.type = CAMERA;
	}
	else if (sensorViewConfiguration.ultrasonic_sensor_view_configuration_size()) {
		sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.ultrasonic_sensor_view_configuration()[0].mounting_position());
		runtimeParameter.carlasensortypes.emplace(ULTRASONIC);
		sensor.type = ULTRASONIC;
	}
	return sensor;
}

Sensor CARLA_OSI_client::toSensorDescriptionInternal(const CoSiMa::rpc::OSISensorViewExtras& sensorViewConfiguration) {
	runtimeParameter.carlaSensors = true;

	Sensor sensor;
	sensor.prefixed_fmu_variable_name = sensorViewConfiguration.prefixed_fmu_variable_name();
	sensor.sensorViewConfiguration.mutable_mounting_position()->CopyFrom(sensorViewConfiguration.sensor_mounting_position());

	if (sensorViewConfiguration.sensor_type() == "generic") {
		sensor.type = GENERIC;
		runtimeParameter.carlasensortypes.emplace(GENERIC);
		sensor.sensorViewConfiguration.add_generic_sensor_view_configuration();
	}
	else if (sensorViewConfiguration.sensor_type() == "radar") {
		sensor.type = RADAR;
		runtimeParameter.carlasensortypes.emplace(RADAR);
		auto* radar = sensor.sensorViewConfiguration.add_radar_sensor_view_configuration();
		radar->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
		radar->set_field_of_view_vertical(sensorViewConfiguration.field_of_view_vertical());
		radar->set_emitter_frequency(sensorViewConfiguration.emitter_frequency());
	}
	else if (sensorViewConfiguration.sensor_type() == "lidar") {
		sensor.type = LIDAR;
		runtimeParameter.carlasensortypes.emplace(LIDAR);
		auto* lidar = sensor.sensorViewConfiguration.add_lidar_sensor_view_configuration();
		lidar->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
		lidar->set_field_of_view_vertical(sensorViewConfiguration.field_of_view_vertical());
		lidar->set_emitter_frequency(sensorViewConfiguration.emitter_frequency());
	}
	else if (sensorViewConfiguration.sensor_type() == "camera") {
		sensor.type = CAMERA;
		runtimeParameter.carlasensortypes.emplace(CAMERA);
		auto* camera = sensor.sensorViewConfiguration.add_camera_sensor_view_configuration();
		camera->set_field_of_view_horizontal(sensorViewConfiguration.field_of_view_horizontal());
		camera->set_number_of_pixels_horizontal(sensorViewConfiguration.number_of_pixels_horizontal());
		camera->set_number_of_pixels_vertical(sensorViewConfiguration.number_of_pixels_vertical());
	}
	else if (sensorViewConfiguration.sensor_type() == "ultrasonic") {
		sensor.type = ULTRASONIC;
		runtimeParameter.carlasensortypes.emplace(ULTRASONIC);
		sensor.sensorViewConfiguration.add_ultrasonic_sensor_view_configuration();
	}
	return sensor;
}
