#include "CARLA2OSIInterface.h"

int CARLA2OSIInterface::readConfiguration(CARLA2OSIInterfaceConfig& config) {
	this->host = config.host;
	this->port = config.port;
	this->transactionTimeout = std::chrono::duration<double>(config.transactionTimeout);
	this->deltaSeconds = config.deltaSeconds;

	return 0;
}

int CARLA2OSIInterface::initialise() {
	//connect
	this->client = std::make_unique<carla::client::Client>(host, port);
	this->client->SetTimeout(transactionTimeout);
	this->world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

	//assure server is in synchronous mode
	auto settings = world->GetSettings();
	settings.fixed_delta_seconds = this->deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings);

	//performing a tick does the same
	////get all actors and their role (if set)
	//auto actors = world->GetActors();
	//std::cout << "Actor count at init: " << actors->size() << std::endl;
	//for each (auto actor in *actors)
	//{
	//	activeActors.insert(actor->GetId());
	//	auto attributes = actor->GetAttributes();
	//	for each (auto attribute in attributes) {
	//		if ("role_name" == attribute.GetId() && !attribute.GetValue().empty()) {
	//			actorRole2IDMap.try_emplace(attribute.GetValue(), actor->GetId());
	//		}
	//	}

	parseStationaryMapObjects();

	// perform a tick to fill actor and message lists
	doStep();

	return 0;
}

double CARLA2OSIInterface::doStep() {
	if (!world) {
		throw std::exception("No world");
	}

	// track actors added/removed by simulation interfaces
	std::set<carla::ActorId> worldActorIDs, addedActors, removedActors;
	auto worldActors = world->GetActors();
	// compare actor ids, not actors
	for each (auto actor in *worldActors)
	{
		worldActorIDs.insert(actor->GetId());
	}
	CarlaUtility::twoWayDifference(
		activeActors.begin(), activeActors.end(),
		worldActorIDs.begin(), worldActorIDs.end(),
		std::inserter(addedActors, addedActors.begin()),
		std::inserter(removedActors, removedActors.begin())
	);

	world->Tick(this->transactionTimeout);
	//world->WaitForTick(this->transactionTimeout);

	// track actors added/removed by Carla
	addedActors.clear();
	removedActors.clear();
	worldActorIDs.clear();
	worldActors = world->GetActors();
	// compare actor ids, not actors
	for each (auto actor in *worldActors)
	{
		worldActorIDs.insert(actor->GetId());
	}
	CarlaUtility::twoWayDifference(
		activeActors.begin(), activeActors.end(),
		worldActorIDs.begin(), worldActorIDs.end(),
		std::inserter(addedActors, addedActors.end()),
		std::inserter(removedActors, removedActors.end()));

	//TODO implement reactions
	for (auto removedActor : removedActors) {
		if (actorRole2IDMap.right.count(removedActor)) {
			actorRole2IDMap.right.erase(removedActor);
		}
		activeActors.erase(removedActor);
	}
	for (auto addedActor : addedActors) {
		activeActors.insert(addedActor);
		auto actor = world->GetActor(addedActor);
		auto attributes = actor->GetAttributes();
		for (auto attribute : attributes) {
			if ("role_name" == attribute.GetId()) {
				if (!std::empty(attribute.GetValue())) {
					auto value = boost::bimap<std::string, carla::ActorId>::value_type(attribute.GetValue(), addedActor);
					actorRole2IDMap.insert(value);

					// if actor is of type sensor, add sensor update listener to receive latest sensor data
					if (0 == actor->GetTypeId().rfind("sensor.", 0)) {
						auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
						sensor->Listen([this, sensor](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData); });
					}
				}
				break;
			}
		}
	}

	return this->deltaSeconds;
}

int CARLA2OSIInterface::getIntValue(std::string base_name) {
	return 0;
};

bool CARLA2OSIInterface::getBoolValue(std::string base_name) {
	return true;
};

float CARLA2OSIInterface::getFloatValue(std::string base_name) {
	return 0.0;
};

double CARLA2OSIInterface::getDoubleValue(std::string base_name) {
	return 0.0;
};

std::string CARLA2OSIInterface::getStringValue(std::string base_name) {
	if (varName2MessageMap.count(base_name)) {
		return varName2MessageMap[base_name];
	}

	return "";
};

int CARLA2OSIInterface::setIntValue(std::string base_name, int value) {
	return 0;
};

int CARLA2OSIInterface::setBoolValue(std::string base_name, bool value) {
	return 0;
};

int CARLA2OSIInterface::setFloatValue(std::string base_name, float value) {
	return 0;
};

int CARLA2OSIInterface::setDoubleValue(std::string base_name, double value) {
	return 0;
};

int CARLA2OSIInterface::setStringValue(std::string base_name, std::string value) {
	auto prefix = getPrefix(base_name);
	if (0 < prefix.length() && 2 + prefix.length() < base_name.length()) {
		// variable has only a prefix and no name
		std::cerr << "CARLA2OSIInterface::setStringValue: Tried to set a variable that has a prefix, but no name (name='" << base_name << "')." << std::endl;
		//TODO do we desire variables that have only a prefix and no name?
		return -2;
	}

	auto varName = std::string_view(&base_name.at(prefix.length() + 2));
	if (0 != varName.rfind("OSMP", 0)) {
		// OSMP variable names begin with 'OSMP', so this is not an OSMP variable
		std::cerr << "CARLA2OSIInterface::setStringValue: '" << base_name << "' is not an OSMP variable." << std::endl;
		return -4;
	}

	// find index, surrounded by [], at the variable name's back, if existent
	uint64_t index = 0;//indices in OSMP variable names start at 1 -> value 0 indicates variable has no index
	if (']' == varName.back()) {
		int i = varName.rfind('[');
		auto indexView = varName.substr(i + 1, varName.length() - (i + 2));
		auto[ptr, errorCode] = std::from_chars(indexView.data(), indexView.data() + indexView.size(), index);

		switch (errorCode)
		{
		case std::errc::invalid_argument:
			std::cerr << "CARLA2OSIInterface::setStringValue: '" << indexView << "' is not an valid OSMP variable index (name='" << base_name << "')." << std::endl;
			return -5;
		case std::errc::result_out_of_range:
			std::cerr << "CARLA2OSIInterface::setStringValue: '" << indexView << "' is out of range (name='" << base_name << "')." << std::endl;
			return -6;
		default:
			break;
		}
	}

	//find variable target/type
	if (actorRole2IDMap.left.count(base_name)) {
		auto actor = world->GetActor(actorRole2IDMap.left.at(base_name));

		//TODO update Carla actor and store merged serialized OSI message in varName2MessageMap
	}
	else if (std::string::npos != varName.find("MotionCommand")) {
		// parse as MotionCommand and apply to ego vehicle
		setlevel4to5::MotionCommand motionCommand;
		if (!motionCommand.ParseFromString(value)) {
			std::cerr << "CARLA2OSIInterface::setStringValue: Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -322;
		}

		receiveMotionCommand(motionCommand);
	}
	else if (std::string::npos != varName.find("TrafficUpdate")) {
		// parse as TrafficUpdate and apply
		osi3::TrafficUpdate trafficUpdate;
		if (!trafficUpdate.ParseFromString(value)) {
			std::cerr << "CARLA2OSIInterface::setStringValue: Variable name'" << base_name << "' indicates this is a TrafficUpdate, but parsing failed." << std::endl;
			return -322;
		}

		receiveTrafficUpdate(trafficUpdate);
	}
	else {
		//Cache unmapped messages so they can be retrieved as input
		//TODO how to map base_name for retrieval as input?
		varName2MessageMap[base_name] = value;
	}


	return 0;
}

std::string_view CARLA2OSIInterface::getPrefix(std::string_view name)
{
	// a prefix is surrounded by '#'
	if (2 < name.size() && '#' == name.front()) {
		std::string_view prefix = name.substr(1, name.find('#', 1));
		return prefix;
	}
	return std::string_view();
}


osi3::Timestamp* CARLA2OSIInterface::parseTimestamp()
{
	osi3::Timestamp* osiTime = new osi3::Timestamp();
	auto carlaTime = world->GetSnapshot().GetTimestamp();
	double intPart;
	double fractional = std::modf(carlaTime.elapsed_seconds, &intPart);
	osiTime->set_seconds(google::protobuf::int64(intPart));
	osiTime->set_nanos(google::protobuf::uint32(fractional *1e9));
	return osiTime;
}


void CARLA2OSIInterface::parseStationaryMapObjects()
{
	mapTruth = std::make_shared<osi3::GroundTruth>();

	carla::SharedPtr<carla::client::Map> map = world->GetMap();

	mapTruth->set_map_reference(map->GetName());

	//parse OpenDRIVE for retrieving information dropped in Carla
	auto result = xodr.load_string(map->GetOpenDrive().c_str());

	auto stationaryObjects = mapTruth->mutable_stationary_object();
	//TODO parse map parts that won't change during simulation

	// Static props apparently aren't part of the actor list, so this list is empty
	auto staticProps = world->GetActors()->Filter("static.prop.*");
	for each(auto prop in *staticProps) {
		// parse as StationaryObject
		stationaryObjects->AddAllocated(CarlaUtility::toOSIStationaryObject(prop));

		//DEBUG
		std::cout << "Got an Actor of type 'static.prop.*'" << prop->GetDisplayId() << std::endl;
	}
	//TODO maybe parse Road Objects Record of OpenDrive file, if present - corresponds to OSI's StationaryObject

	auto landmarks = map->GetAllLandmarks();
	for each(auto landmark in landmarks) {
		std::cout << landmark->GetName() << " " << landmark->GetId() << " " << landmark->GetRoadId() << " " << landmark->GetType() << std::endl;
	}

	auto traffic = world->GetActors()->Filter("traffic.*");
#ifdef WIN32
	//auto trafficLight = traffic->Filter("traffic.traffic_light");
	std::unique_ptr<std::vector<carla::SharedPtr<carla::client::Actor>>> trafficSigns =
		std::make_unique<std::vector<carla::SharedPtr<carla::client::Actor>>>();
	std::copy_if(traffic->begin(), traffic->end(), std::inserter(*trafficSigns, trafficSigns->begin()),
		[](carla::SharedPtr<carla::client::Actor> actor) {return 0 != actor->GetTypeId().rfind("traffic.traffic_light", 0); });
#else
	// fnmatch supports negation
	auto trafficSigns = traffic->Filter("!traffic.traffic_light");
#endif

	auto OSITrafficSigns = mapTruth->mutable_traffic_sign();
	for (auto trafficSign : *trafficSigns) {
		carla::SharedPtr<carla::client::TrafficSign> carlaTrafficSign = boost::dynamic_pointer_cast<carla::client::TrafficSign>(trafficSign);
		auto OSITrafficSign = CarlaUtility::toOSI(carlaTrafficSign, xodr);
		OSITrafficSigns->AddAllocated(OSITrafficSign);
	}

	//TODO might be able to get lanes using map->GetTopology() and next_until_lane_end and previous_until_lane_start
	auto lanes = mapTruth->mutable_lane();
	auto topology = map->GetTopology();
	//TODO Junctions need extra handling when converting to OSI (lane pairing holds more than two lanes)
	std::set<carla::road::JuncId> junctions;
	for (auto endpoints : topology) {
		if (endpoints.first->IsJunction() && endpoints.second->IsJunction()) {
			auto junction = endpoints.first->GetJunction();

			auto id = junction->GetId();
			//TODO OSI Junction have a different defintion, listing the lanes connected to the junction, but not the paths through the junction
			if (junctions.count(id)) {
				//This junction has already been parsed
				continue;
			}

			junctions.insert(id);
			auto lane = lanes->Add();
			lane->set_allocated_id(CarlaUtility::toOSI(id));

			auto classification = lane->mutable_classification();
			classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION);

			for (auto path : junction->GetWaypoints()) {
				auto pair = classification->add_lane_pairing();
				//TODO finish implementation
			}

		}
		else {
			// A lane that is not a junction. Waypoints of endpoint pair map to a OSI lane
			auto lane = lanes->Add();
			auto roadId = endpoints.first->GetRoadId();
			auto laneId = endpoints.first->GetLaneId();
			lane->set_allocated_id(CarlaUtility::toOSI(roadId, laneId));

			auto classification = lane->mutable_classification();

			if (carla::road::Lane::LaneType::Driving == endpoints.first->GetType()) {
				// centerline is only defined for lanes of type driving, except for junctions
				classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING);

				//get all waypoints until the lane's end, approximately 10cm apart. osi3::Lane::Classification expects its centerline have a deviation of at most 5cm when followed linearly
				//TODO reduce number of waypoints - especially straight segments offer a more sparse representation
				auto waypoints = endpoints.first->GetNextUntilLaneEnd(0.1f);
				if (waypoints.size()) {
					classification->set_centerline_is_driving_direction(true);
				}
				else {
					// endpoints.first was not the start point (?)
					waypoints = endpoints.second->GetNextUntilLaneEnd(0.1f);
					//DEBUG
					std::cout << "Encountered a lane defined in reversed direction" << std::endl;
					classification->set_centerline_is_driving_direction(false);

					//switch endpoints so we don't have to test the direction again
					auto tmp = endpoints.first;
					endpoints.first = endpoints.second;
					endpoints.second = tmp;
				}
				//translate waypoints to OSI centerline
				auto centerline = classification->mutable_centerline();
				for (auto waypoint : waypoints) {
					auto location = waypoint->GetTransform().location;
					centerline->AddAllocated(CarlaUtility::toOSI(location));
				}
			}
			else {
				// A NonDriving lane. Intersections and Driving have already been handled
				classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING);
			}

			// OSI lane_pairing needs an antecessor/successor pair
			//TODO a lookup in OpenDrive file might be more accurate and prevent doubled entries
			auto prev = endpoints.first->GetPrevious(0.1f);
			auto next = endpoints.second->GetNext(0.1f);
			for (size_t p = 0; p < prev.size(); p++)
				for (size_t n = 0; n < next.size(); n++) {
					auto pair = classification->add_lane_pairing();
					if (prev[p]->IsJunction()) {
						pair->set_allocated_antecessor_lane_id(CarlaUtility::toOSI(prev[p]->GetJunctionId()));
					}
					else {
						pair->set_allocated_antecessor_lane_id(CarlaUtility::toOSI(prev[p]->GetRoadId(), prev[p]->GetLaneId()));
					}
					if (next[n]->IsJunction()) {
						pair->set_allocated_successor_lane_id(CarlaUtility::toOSI(next[n]->GetJunctionId()));
					}
					else {
						pair->set_allocated_successor_lane_id(CarlaUtility::toOSI(next[n]->GetRoadId(), next[n]->GetLaneId()));
					}
				}


			//TODO add osi3::LaneBoundary, needs positions of lane boundary markings. Maybe needs a lookup in XODR file

		}
	}



	}

osi3::GroundTruth* CARLA2OSIInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use mapTruth as a base for every new groundTruth message that already contains unchanging fields
	osi3::GroundTruth* groundTruth = new osi3::GroundTruth();
	groundTruth->MergeFrom(*mapTruth);

	for each (auto actor in *world->GetActors()) {
		auto typeID = actor->GetTypeId();

		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth->add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);

			vehicle->set_type(osi3::MovingObject_Type_TYPE_VEHICLE);
			vehicle->set_allocated_base(CarlaUtility::toOSIBaseMoving(actor));

			auto classification = vehicle->mutable_vehicle_classification();
			classification->set_has_trailer(false);

			// Get closest waypoint to determine current lane
			auto waypoint = world->GetMap()->GetWaypoint(vehicleActor->GetLocation());
			//TODO vehicle might be on more than one lane
			auto laneIDs = vehicle->mutable_assigned_lane_id();
			if (waypoint->IsJunction()) {
				laneIDs->AddAllocated(CarlaUtility::toOSI(waypoint->GetJunctionId(), CarlaUtility::JuncID));
			}
			else {
				laneIDs->AddAllocated(CarlaUtility::toOSI(waypoint->GetRoadId(), waypoint->GetLaneId()));
			}

			auto attributes = vehicle->mutable_vehicle_attributes();

			for (auto attribute : vehicleActor->GetAttributes()) {
				//TODO verify/improve object type mapping
				if ("object_type" == attribute.GetId()) {
					auto value = attribute.GetValue();
					if (std::string::npos != value.find("bicycle")) {
						classification->set_type(osi3::MovingObject_VehicleClassification_Type_TYPE_BICYCLE);
					}
					else if (std::string::npos != value.find("motorbike")
						|| std::string::npos != value.find("moped")) {
						classification->set_type(osi3::MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE);
					}
					else {
						//TODO extend object type mapping
						classification->set_type(osi3::MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR);
					}
				}
				else if ("number_of_wheels" == attribute.GetId()) {
					attributes->set_number_wheels(attribute.As<int>());
				}
			}

			// parse vehicle lights
			classification->set_allocated_light_state(CarlaUtility::toOSI(vehicleActor->GetLightState()).release());

			// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in CarlaUtility::toOSI
			auto[dimension, location] = CarlaUtility::toOSI(vehicleActor->GetBoundingBox());
			vehicle->mutable_base()->set_allocated_dimension(dimension.release());

			//TODO Bounding box to rear/front offsets

			//TODO ground clearance
			//TODO wheel radius




		}
		else if (typeID.rfind("walker.pedestrian", 0) == 0) {
			auto pedestrian = groundTruth->add_moving_object();
			auto walkerActor = boost::static_pointer_cast<carla::client::Walker>(actor);

			pedestrian->set_type(osi3::MovingObject_Type_TYPE_PEDESTRIAN);
			pedestrian->set_allocated_base(CarlaUtility::toOSIBaseMoving(actor));

			// parse bounding box to dimension field of base - there is no generic way to retrieve an actor's bounding box in CarlaUtility::toOSI
			auto[dimension, location] = CarlaUtility::toOSI(walkerActor->GetBoundingBox());
			pedestrian->mutable_base()->set_allocated_dimension(dimension.release());

			//TODO How to determine a lane for pedestrians? Carla walkers don't care about lanes and walk on meshes with specific names (see https://carla.readthedocs.io/en/0.9.9/tuto_D_generate_pedestrian_navigation/):
			// Road_Sidewalk, Road_Crosswalk, Road_Grass, Road_Road, Road_Curb, Road_Gutter or Road_Marking 

			//TODO parse pedestrian as moving object


		}
		else if ("traffic.traffic_light" == typeID) {
			carla::SharedPtr<carla::client::TrafficLight> trafficLight = boost::dynamic_pointer_cast<carla::client::TrafficLight>(actor);
			//TODO parse carla::client::TrafficLight as a set of osi3::TrafficLight
			//a osi3::TrafficLight describes a single bulb of a traffic light


		}
	}



	return groundTruth;
}

void CARLA2OSIInterface::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData)
{

	std::unique_ptr<osi3::SensorView> sensorView = std::make_unique<osi3::SensorView>();

	auto typeID = sensor->GetTypeId();
	//substring of typeID
	std::string_view sensorType(&typeID[7]);

	if (0 == sensorType.rfind("camera.rgb", 0))
	{
		auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(sensorData);
		auto cameraSensorView = CarlaUtility::toOSICamera(sensor, image);
		sensorView->mutable_camera_sensor_view()->AddAllocated(cameraSensorView);
	}
	else if (0 == sensorType.rfind("lidar.ray_cast", 0))
	{
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(sensorData);
		auto lidarSensorView = CarlaUtility::toOSILidar(sensor, measurement);
		sensorView->mutable_lidar_sensor_view()->AddAllocated(lidarSensorView);
	}
	else if (0 == sensorType.rfind("other.radar", 0))
	{
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::RadarMeasurement>(sensorData);
		auto radarSensorView = CarlaUtility::toOSIRadar(sensor, measurement);
		sensorView->mutable_radar_sensor_view()->AddAllocated(radarSensorView);
	}
	else {
		std::cerr << "CARLA2OSIInterface.sensorEventAction called for unsupported sensor type" << std::endl;
	}

	auto varName = actorRole2IDMap.right.at(sensor->GetId());
	varName2MessageMap[varName] = sensorView->SerializeAsString();
}

void CARLA2OSIInterface::sendTrafficCommand(carla::ActorId ActorId) {
	std::unique_ptr<osi3::TrafficCommand> trafficCommand = std::make_unique<osi3::TrafficCommand>();
	auto actorid = CarlaUtility::toOSI(ActorId);

	trafficCommand->set_allocated_traffic_participant_id(actorid);
	osi3::Timestamp* timestamp = parseTimestamp();
	trafficCommand->set_allocated_timestamp(timestamp);
	auto trafficAction = trafficCommand->add_action();

	//do action accordingly
	int TrafficActionType = 0;//TODO Placeholder at the moment

	switch (TrafficActionType) {
	case 0:
	{
		//follow trajectory
		auto trajectoryAction = trafficAction->mutable_follow_trajectory_action();
		//trajectoryAction->set_allocated_action_header();
		//trajectoryAction->add_trajectory_point(); //repeated
		//trajectoryAction->set_constrain_orientation();
		//trajectoryAction->set_following_mode();
		break;
	}
	case 1:
	{
		//follow path
		auto pathAction = trafficAction->mutable_follow_path_action();
		//pathAction->set_allocated_action_header();
		//pathAction->add_path_point(); //repeated
		//pathAction->set_constrain_orientation();
		//pathAction->set_following_mode();
		break;
	}
	case 2:
	{
		//acquire global position action
		auto acquireGlobalPositionAction = trafficAction->mutable_acquire_global_position_action();
		//acquireGlobalPositionAction->set_allocated_action_header();
		//acquireGlobalPositionAction->set_allocated_position();
		//acquireGlobalPositionAction->set_allocated_orientation();
		break;
	}
	case 3:
	{
		//lane change action
		auto laneChangeAction = trafficAction->mutable_lane_change_action();
		//laneChangeAction->set_allocated_action_header();
		//laneChangeAction->set_relative_target_lane();
		//laneChangeAction->set_dynamics_shape();
		//laneChangeAction->set_duration();
		//laneChangeAction->set_distance();
		break;
	}
	case 4:
	{
		//speed action
		auto speedAction = trafficAction->mutable_speed_action();
		break;
	}
	default:
		std::cerr << "CARLA2OSIInterface.sendTrafficCommand called with undefined traffic action type" << std::endl;
	}

	auto varName = actorRole2IDMap.right.at(ActorId);
	varName2MessageMap[varName] = trafficCommand->SerializeAsString();

	delete timestamp;
}

int CARLA2OSIInterface::receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate) {
	//OSI documentation:
	//Only the id, base member (without dimension and base_polygon),
	//and the vehicle_classification.light_state members are considered in
	//updates, all other members can be left undefined, and will be
	//ignored by the receiver of this message.

	if (!trafficUpdate.has_update() && trafficUpdate.mutable_update()->has_id()) {
		std::cerr << "CARLA2OSIInterface.receiveTrafficUpdate read traffic with no id." << std::endl;
		return 3;
	}
	auto TrafficId = std::get<carla::ActorId>(CarlaUtility::toCarla(&trafficUpdate.mutable_update()->id()));
	auto actor = world->GetActor(TrafficId);
	if (TrafficId != actor->GetId()) {
		std::cerr << "CARLA2OSIInterface.receiveTrafficUpdate: No actor with id" << TrafficId << std::endl;
		return 2;
	}


	//BASE
	if (trafficUpdate.mutable_update()->mutable_base()->has_position()
		&& trafficUpdate.mutable_update()->mutable_base()->has_orientation()) {
		auto position = CarlaUtility::toCarla(&trafficUpdate.mutable_update()->mutable_base()->position());
		auto orientation = CarlaUtility::toCarla(&trafficUpdate.mutable_update()->mutable_base()->orientation());
		actor->SetTransform(carla::geom::Transform(position, orientation));
	}

	//Velocity
	if (trafficUpdate.mutable_update()->mutable_base()->has_velocity()) {
		actor->SetVelocity(CarlaUtility::toCarla(&trafficUpdate.mutable_update()->mutable_base()->velocity()));
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//if (trafficUpdate.mutable_update()->mutable_base()->has_acceleration()) {
		//auto acceleration = CarlaUtility::toCarla(&trafficUpdate.mutable_update()->mutable_base()->acceleration());
	//}

	//Orientation
	if (trafficUpdate.mutable_update()->mutable_base()->has_orientation_rate()) {
		const auto orientationRate = CarlaUtility::toCarla(trafficUpdate.mutable_update()->mutable_base()->mutable_orientation_rate());

		//TODO Check if conversion is correct: x should be forward, y should be up, z should be right
		actor->SetAngularVelocity({ orientationRate.GetForwardVector().Length(), orientationRate.GetUpVector().Length(), orientationRate.GetRightVector().Length() });
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//if (trafficUpdate.mutable_update()->mutable_base()->has_orientation_acceleration()){
		//const osi3::Orientation3d* accelerationRoll = trafficUpdate.mutable_update()->mutable_base()->mutable_orientation_acceleration();
	//}

	//LIGHTSTATE
	if (trafficUpdate.mutable_update()->mutable_vehicle_classification()->has_light_state()) {
		auto indicatorState = CarlaUtility::toCarla(trafficUpdate.mutable_update()->mutable_vehicle_classification()->mutable_light_state());
		auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);

		vehicleActor->SetLightState(indicatorState);
	}
	return 0;
}

int CARLA2OSIInterface::receiveMotionCommand(setlevel4to5::MotionCommand& motionCommand) {
	// MotionCommand describes a 2D trajectory to follow

	//TODO MotionCommand has no id field -> verify message applies to ego vehicle

	//TODO how to safely retrieve the ego vehicle? 
	// ego vehicle in Carla usually is named 'hero'
	if (0 == actorRole2IDMap.left.count("hero")) {
		return -100;
	}
	auto actorId = actorRole2IDMap.left.at("hero");
	auto actor = world->GetActor(actorId);
	//From OSI documentation:
	// The motion command comprises of the trajectory the vehicle should
	// follow on, as well as the current dynamic state which contains
	// the vehicle's localization and dynamic properties.

	//TODO add checks?
	motionCommand.version();
	motionCommand.timestamp();

	//From OSI documentation
	// The coordinate system is right handed, a heading of zero equalling
	// the object being heading in x-direction.
	// All coordinates and orientations are relative to the global ground
	// truth frame.
	//
	// Units are [m] for positions, [m/s] for velocities, [m/s^2] for
	// accelerations and [rad] for angles.

	//TODO check timestamp of current state?
	motionCommand.current_state().timestamp();

	//Position and Orientation
	osi3::Vector3d vector;
	vector.set_x(motionCommand.current_state().position_x());
	vector.set_y(motionCommand.current_state().position_y());

	osi3::Orientation3d orientation;
	orientation.set_yaw(motionCommand.current_state().heading_angle());

	//convert to CARLA
	carla::geom::Location location = CarlaUtility::toCarla(&vector);
	carla::geom::Rotation rotation = CarlaUtility::toCarla(&orientation);

	actor->SetTransform(carla::geom::Transform(location, rotation));

	//Velocity
	if (motionCommand.current_state().has_velocity()) {
		double velocityTotal = motionCommand.current_state().velocity();

		//extract angles from rotation
		double angle_x = cos(rotation.yaw * M_1_PI / 180.0);
		double angle_z = sin(rotation.yaw * M_1_PI / 180.0);

		//y is upward in CARLA
		carla::geom::Vector3D velocity(angle_x * velocityTotal, 0, angle_z * velocityTotal);
		actor->SetVelocity(velocity);
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//double acceleration = motionCommand.current_state().acceleration();
	//Curvature can not be set in CARLA since it is operated in synchronous mode
	//double curvature = motionCommand.current_state().curvature();

	//Trajectory
	//CARLA can not handle locations for future timestamps
	//TODO write TrajectoryHandler to handle vehicle trajectory in this client?
	return 0;
}
