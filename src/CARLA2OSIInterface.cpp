#include "CARLA2OSIInterface.h"

int CARLA2OSIInterface::initialise(std::string host, uint16_t port, double transactionTimeout, double deltaSeconds, RuntimeParameter& runtimeParams) {
	this->runtimeParameter = runtimeParams;
	this->deltaSeconds = deltaSeconds;

	try {
		//connect
		this->client = std::make_unique<carla::client::Client>(host, port);
		this->client->SetTimeout(std::chrono::duration<double>(transactionTimeout));
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
	loadWorld();
	applyWorldSettings();
	parseStationaryMapObjects();

	// perform a tick to fill actor and message lists
	doStep();

	return 0;
}

double CARLA2OSIInterface::doStep() {
	if (!world) {
		std::cerr << "No world" << std::endl;
		throw std::exception();
	}
	else if (this->world->GetId() != this->client->GetWorld().GetId()) {
		// change of world id indicates map reload or map change
		std::cerr << "World has changed" << std::endl;
		this->clearData();
	}

	if (runtimeParameter.dynamicTimestamps) {
		auto end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end - last_timestamp;
		last_timestamp = end;

		auto settings = world->GetSettings();
		settings.fixed_delta_seconds = elapsed_seconds.count();
		this->world->ApplySettings(settings, settingsDuration);
	}

	//tick not needed if in asynchronous mode
	if (runtimeParameter.sync) {
		world->Tick(client->GetTimeout());
	}
	//world->WaitForTick(this->transactionTimeout);
	validLatestGroundTruth = false;

	// only accurate if using fixed time step, as activated during initialise()
	return world->GetSnapshot().GetTimestamp().delta_seconds;
}

void CARLA2OSIInterface::fetchActorsFromCarla() {

	// track actors added/removed inside Carla
	std::set<carla::ActorId> worldActorIDs, addedActors, removedActors;
	auto worldActors = world->GetActors();
	// compare actor ids, not actors
	for (auto actor : *worldActors)
	{
		worldActorIDs.insert(actor->GetId());
	}
	CarlaUtility::twoWayDifference(
		activeActors.begin(), activeActors.end(),
		worldActorIDs.begin(), worldActorIDs.end(),
		std::inserter(addedActors, addedActors.end()),
		std::inserter(removedActors, removedActors.end()));

	//TODO implement reactions
	{//mutex scope
		// using a unique lock - access not limited to read
		std::unique_lock lock(actorRole2IDMap_mutex);
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
						if (runtimeParameter.carlaSensors && 0 == actor->GetTypeId().rfind("sensor.", 0)) {
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							sensor->Listen([this, sensor](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData); });
						}
					}
					break;
				}
			}
		}
	}
}

std::shared_ptr<const osi3::GroundTruth> CARLA2OSIInterface::getLatestGroundTruth()
{
	if (!validLatestGroundTruth) {
		latestGroundTruth = parseWorldToGroundTruth();
		validLatestGroundTruth = true;
	}
	return latestGroundTruth;
}

void CARLA2OSIInterface::loadWorld() {
	this->world = std::make_unique<carla::client::World>(std::move(this->client->GetWorld()));
	this->map = world->GetMap();
}

void CARLA2OSIInterface::applyWorldSettings() {
	auto settings = world->GetSettings();
	//set sync or async operational mode
	settings.synchronous_mode = runtimeParameter.sync;

	if (settings.fixed_delta_seconds.has_value() &&
		settings.fixed_delta_seconds.value() == deltaSeconds &&
		settings.synchronous_mode) {
		if (runtimeParameter.verbose) {
			std::cout << "Settings of Carla Server are already correct and do not need to be changed" << std::endl;
		}
		return;
	}
	settings.fixed_delta_seconds = deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings, settingsDuration);
}

void CARLA2OSIInterface::resetWorldSettings() {
	auto settings = world->GetSettings();
	settings.synchronous_mode = false;
	this->world->ApplySettings(settings, settingsDuration);
}

std::shared_ptr<const osi3::SensorView> CARLA2OSIInterface::getSensorView(std::string role)
{
	// mutex scope
	// using a shared lock - read only access
	std::shared_lock lock(varName2MessageMap_mutex);
	auto iter = varName2MessageMap.find(role);
	if (varName2MessageMap.end() == iter) {
		std::string functionName(__FUNCTION__);
		return nullptr;
	}
	auto sensorViewPtr = std::get_if<std::shared_ptr<osi3::SensorView>>(&iter->second);
	if (nullptr == sensorViewPtr || !*sensorViewPtr) {
		std::string functionName(__FUNCTION__);
		std::cerr << functionName + ": role '" + role + "' is not a SensorView" << std::endl;
		return nullptr;
	}
	return *sensorViewPtr;
}

std::string CARLA2OSIInterface::actorIdToRoleName(const osi3::Identifier& id)
{
	carla::ActorId actorId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&id));
	std::string role;
	try {//mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex);
		//look up from right to left -> retrieve role for given id
		role = actorRole2IDMap.right.at(actorId);
	}
	catch (std::out_of_range exception) {
		//empty by design
	}
	return role;
}

std::string_view CARLA2OSIInterface::getPrefix(std::string_view name)
{
	// a prefix is surrounded by '#'
	if (2 < name.size() && '#' == name.front()) {
		std::string_view prefix = name.substr(1, name.find('#', 1) - 1);
		return prefix;
	}
	return std::string_view();
}

osi3::Timestamp* CARLA2OSIInterface::parseTimestamp()
{
	osi3::Timestamp* osiTime = new osi3::Timestamp();
	carla::client::Timestamp carlaTime = world->GetSnapshot().GetTimestamp();
	double intPart;
	double fractional = std::modf(carlaTime.elapsed_seconds, &intPart);
	osiTime->set_seconds(google::protobuf::int64(intPart));
	osiTime->set_nanos(google::protobuf::uint32(fractional *1e9));
	return osiTime;
}

std::vector<carla::rpc::EnvironmentObject> CARLA2OSIInterface::filterEnvironmentObjects() {

	std::vector<carla::rpc::EnvironmentObject> props{};
	std::vector<carla::rpc::EnvironmentObject> filteredprops{};

	if (runtimeParameter.options.None) {
		return props;
	}
	if (runtimeParameter.options.Any) {
		props = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Any);
	}
	else {
		//specialized types
		//carla::rpc::CityObjectLabel::Pedestrians is not a static object
		//carla::rpc::CityObjectLabel::TrafficSigns is handled at an other point
		//carla::rpc::CityObjectLabel::Vehicle is not a static object
		//carla::rpc::CityObjectLabel::Sky is not a static object
		//carla::rpc::CityObjectLabel::TrafficLight is handled at an other point
		//carla::rpc::CityObjectLabel::Dynamic is not a static object

		if (runtimeParameter.options.Buildings)
		{
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Buildings);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Fences) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Fences);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Other) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Other);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Poles) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Poles);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.RoadLines) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::RoadLines);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Roads) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Roads);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Sidewalks) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Sidewalks);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Vegetation) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Vegetation);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Walls) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Walls);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Ground) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Ground);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Bridge) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Bridge);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.RailTrack) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::RailTrack);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.GuardRail) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::GuardRail);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Static) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Static);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Water) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Water);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Terrain) {
			auto buildings = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Water);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
	}//endif options.Any

	for (auto& prop : props) {
		//do not parse the CameraActor spawned by Carla and all actors containing Planes
		if (prop.name.find("CameraActor") == 0 || prop.name.find("Plane") != std::string::npos) {
			if (runtimeParameter.verbose)
				std::cout << "Not parsing " << prop.name << "\n";
			continue;
		}

		//do only parse actors in filter set by user per runtime parameter
		if (runtimeParameter.filter) {
			if (prop.name.find(runtimeParameter.filterString) == std::string::npos) {
				if (runtimeParameter.verbose)
					std::cout << "Not parsing " << prop.name << "\n";
				continue;
			}
		}
		filteredprops.push_back(prop);
	}
	return filteredprops;
}

void CARLA2OSIInterface::parseStationaryMapObjects()
{
	staticMapTruth = std::make_unique<osi3::GroundTruth>();
	staticMapTruth->set_map_reference(map->GetName());
	auto OSIStationaryObjects = staticMapTruth->mutable_stationary_object();
	auto filteredStationaryMapObjects = filterEnvironmentObjects();

	for (auto& mapObject : filteredStationaryMapObjects) {
		OSIStationaryObjects->AddAllocated(CarlaUtility::toOSI(mapObject, runtimeParameter.verbose));
	}
	auto OSITrafficSigns = staticMapTruth->mutable_traffic_sign();
	auto signs = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::TrafficSigns);
	for (auto& sign : signs) {
		auto trafficSign = world->GetActor(sign.id);
		carla::SharedPtr<carla::client::TrafficSign> carlaTrafficSign = boost::dynamic_pointer_cast<carla::client::TrafficSign>(trafficSign);
		auto OSITrafficSign = carla_osi::traffic_signals::getOSITrafficSign(carlaTrafficSign, sign.bounding_box);
		OSITrafficSigns->AddAllocated(OSITrafficSign.release());
	}

	if (!runtimeParameter.noMapNetworkInGroundTruth) {
		auto lanes = staticMapTruth->mutable_lane();
		auto laneBoundaries = staticMapTruth->mutable_lane_boundary();
		auto topology = map->GetTopology();
		lanes->Reserve(topology.size());
		std::cout << "Map topology consists of " << topology.size() << " endpoint pairs" << std::endl;

		// use a vector as replacement for a python-zip-like view, as boost::combine() (boost version 1.72) cannot
		// be used with execution policies and ranges-v3 (with similar ranges::views::zip) requires at least 
		// Visual Studio 2019 on Windows
		using zip_type = std::vector<std::tuple<carla::client::Map::TopologyList::value_type*, std::unique_ptr<osi3::Lane>, google::protobuf::RepeatedPtrField<osi3::LaneBoundary>>>;
		zip_type combined(topology.size());
		for (size_t i = 0; i < topology.size(); i++) {
			std::get<0>(combined[i]) = &topology[i];
			std::get<1>(combined[i]) = std::make_unique<osi3::Lane>();
		}

		const carla::road::Map& roadMap = map->GetMap();

		// execute in parallel to increase performance for large maps
		std::for_each(std::execution::par, combined.begin(), combined.end(), [&](zip_type::value_type& tuple) {
			auto&[endpoints, lane, boundaries] = tuple;
			auto&[laneStart, laneEnd] = *endpoints;
			if (laneStart->IsJunction() && laneEnd->IsJunction()) {
				auto junction = laneStart->GetJunction();

				// OSI Junction have a different defintion, listing the lanes connected to the junction, but not the paths through the junction
				// => store all lanes that form this junction
				// TODO find and parse all free lane boundaries for this junction

				lane->set_allocated_id(carla_osi::id_mapping::getOSIJunctionId(junction).release());

				auto classification = lane->mutable_classification();
				classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION);

				auto waypoints = junction->GetWaypoints();
				auto lanePairings = classification->mutable_lane_pairing();
				lanePairings->Reserve(waypoints.size());
				for (const auto&[inbound, outbound] : waypoints) {
					// OSI lane_pairing needs an antecessor/successor pair
					auto pairs = carla_osi::lanes::GetOSILanePairings(roadMap,
						inbound, outbound);
					lanePairings->MergeFrom(pairs);
				}

			}
			else {
				// A lane that is not a junction. Waypoints of endpoint pair map to a OSI lane
				//auto lane = lanes->Add();
				auto roadId = laneStart->GetRoadId();
				auto laneId = laneStart->GetLaneId();
				auto sectionId = laneStart->GetSectionId();
				lane->set_allocated_id(carla_osi::id_mapping::getOSIWaypointId(laneStart).release());

				auto classification = lane->mutable_classification();

				if (carla::road::Lane::LaneType::Driving == laneStart->GetType()) {
					// centerline is only defined for lanes of type driving, except for junctions
					classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING);

					//get all waypoints until the lane's end, approximately 10cm apart. osi3::Lane::Classification expects its centerline have a deviation of at most 5cm when followed linearly
					//TOCHECK Optimization reduce number of waypoints - especially straight segments offer a more sparse representation
					auto waypoints = laneStart->GetNextUntilLaneEnd(0.1f);
					if (waypoints.size()) {
						classification->set_centerline_is_driving_direction(true);
					}
					else {
						// laneStart was not the start point (?)
						waypoints = laneEnd->GetNextUntilLaneEnd(0.1f);
						//DEBUG
						std::cout << __FUNCTION__ << " DEBUG: Encountered a lane defined in reversed direction" << std::endl;
						classification->set_centerline_is_driving_direction(false);

						////switch endpoints so we don't have to test the direction again
						//auto tmp = laneStart;
						//laneStart = laneEnd;
						//laneEnd = tmp;
					}
					//translate waypoints to OSI centerline
					auto centerline = classification->mutable_centerline();
					centerline->Reserve(waypoints.size());
					for (const auto& waypoint : waypoints) {
						auto location = waypoint->GetTransform().location;
						centerline->AddAllocated(carla_osi::geometry::toOSI(location).release());
					}

					//add left neighbouring lane
					for (auto& neighbouringLane : { laneStart->GetLeft() }) {
						if (neighbouringLane) {
							classification->mutable_left_adjacent_lane_id()->AddAllocated(
								carla_osi::id_mapping::getOSIWaypointId(neighbouringLane).release());
						}
					}
					//add right neighbouring lane
					for (auto& neighbouringLane : { laneStart->GetRight() }) {
						if (neighbouringLane) {
							classification->mutable_right_adjacent_lane_id()->AddAllocated(
								carla_osi::id_mapping::getOSIWaypointId(neighbouringLane).release());
						}
					}
				}
				else {
					// A NonDriving lane. Intersections and Driving have already been handled
					classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING);
				}

				//add antecesseor/successor pairs
				classification->mutable_lane_pairing()->MergeFrom(carla_osi::lanes::GetOSILanePairings(roadMap, laneStart, laneEnd));

				// From OSI documentationon of osi3::LaneBoundary::Classification::Type:
				// There is no special representation for double lines, e.g. solid / solid or dashed / solid. In such 
				// cases, each lane will define its own side of the lane boundary.
				auto boundary = carla_osi::lanes::parseLaneBoundary(*endpoints);
				auto&[parsedBoundaries, left_lane_boundary_id, right_lane_boundary_id] = boundary;
				if (0 < left_lane_boundary_id) {
					classification->add_left_lane_boundary_id()->set_value(left_lane_boundary_id);
				}
				if (0 < right_lane_boundary_id) {
					classification->add_right_lane_boundary_id()->set_value(right_lane_boundary_id);
				}
				boundaries.MergeFrom(parsedBoundaries);
			}
			//TODO osi3::Lane::Classification::road_condition
		}
		);

		// OSI Junction have a different defintion, listing the lanes connected to the junction, but not the paths through the junction
		// => remove duplicates
		std::set<uint64_t> junctionIds;
		for (auto& zipped : combined) {
			auto&[_, lane, boundaries] = zipped;
			if (osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION == lane->classification().type()) {
				if (junctionIds.count(lane->id().value())) {
					continue;
				}
				junctionIds.insert(lane->id().value());
			}
			//lanes->Add(std::move(lane));
			lanes->AddAllocated(lane.release());
		}
		std::cout << "Finished parsing of topology" << std::endl;
	}

}

std::shared_ptr<osi3::GroundTruth> CARLA2OSIInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use staticMapTruth as a base for every new groundTruth message that already contains unchanging fields
	std::shared_ptr<osi3::GroundTruth> groundTruth = std::make_shared<osi3::GroundTruth>();
	//default shall be true
	if (runtimeParameter.staticObjectsInGroundTruthMessage) {
		groundTruth->MergeFrom(*staticMapTruth);
	}

	auto worldActors = world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();

		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth->add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);

			vehicle->set_model_reference(vehicleActor->GetTypeId());
			vehicle->set_allocated_id(carla_osi::id_mapping::getOSIActorId(vehicleActor).release());
			vehicle->set_type(osi3::MovingObject_Type_TYPE_VEHICLE);

			vehicle->set_allocated_base(CarlaUtility::toOSIBaseMoving(vehicleActor).release());

			auto classification = vehicle->mutable_vehicle_classification();
			classification->set_has_trailer(false);

			// Get closest waypoint to determine current lane
			auto waypoint = map->GetWaypoint(vehicleActor->GetLocation());
			//TODO vehicle might be on more than one lane
			auto laneIDs = vehicle->mutable_assigned_lane_id();
			laneIDs->AddAllocated(carla_osi::id_mapping::getOSIWaypointId(waypoint).release());

			auto attributes = vehicle->mutable_vehicle_attributes();
			auto frontAxle = attributes->mutable_bbcenter_to_front();
			auto rearAxle = attributes->mutable_bbcenter_to_rear();

			for (auto& attribute : vehicleActor->GetAttributes()) {
				//TODO verify/improve object type mapping
				if ("object_type" == attribute.GetId() || "osi_vehicle_type" == attribute.GetId()) {
					classification->set_type(CarlaUtility::ParseVehicleType(attribute.GetValue()));
				}
				else if ("number_of_wheels" == attribute.GetId()) {
					attributes->set_number_wheels(attribute.As<int>());
				}
				else if ("wheel_radius" == attribute.GetId()) {
					attributes->set_radius_wheel(attribute.As<float>());
				}
				else if ("ground_clearance" == attribute.GetId()) {
					attributes->set_ground_clearance(attribute.As<float>());
				}
				else if ("bbcenter_to_front_x" == attribute.GetId()) {
					frontAxle->set_x(attribute.As<float>());
				}
				else if ("bbcenter_to_front_y" == attribute.GetId()) {
					// y-axis has to be flipped
					frontAxle->set_y(-attribute.As<float>());
				}
				else if ("bbcenter_to_front_z" == attribute.GetId()) {
					frontAxle->set_z(attribute.As<float>());
				}
				else if ("bbcenter_to_rear_x" == attribute.GetId()) {
					rearAxle->set_x(attribute.As<float>());
				}
				else if ("bbcenter_to_rear_y" == attribute.GetId()) {
					// y-axis has to be flipped
					rearAxle->set_y(-attribute.As<float>());
				}
				else if ("bbcenter_to_rear_z" == attribute.GetId()) {
					rearAxle->set_z(attribute.As<float>());
				}
				else if ("role_name" == attribute.GetId()) {
					std::string role_name = attribute.GetValue();
					if (role_name == runtimeParameter.ego) {
						groundTruth->mutable_host_vehicle_id()->set_value(vehicle->id().value());
						//saved for trafficCommand message from scenario runner
						heroId = vehicle->id().value();
					}
				}
			}
			// parse vehicle lights
			classification->set_allocated_light_state(CarlaUtility::toOSI(vehicleActor->GetLightState()).release());

			if (runtimeParameter.verbose) {
				std::cout << "OSI-Dimensions: " << vehicle->base().dimension().length() << " " << vehicle->base().dimension().width() << " " << vehicle->base().dimension().height()
					<< " OSI-Position: " << vehicle->base().position().x() << " " << vehicle->base().position().y() << " " << vehicle->base().position().z()
					<< " OSI-Rotation: " << vehicle->base().orientation().roll() << " " << vehicle->base().orientation().pitch() << " " << vehicle->base().orientation().yaw()
					<< " Name: " << vehicleActor->GetId()
					<< std::endl;
			}

		}
		else if (typeID.rfind("walker.pedestrian", 0) == 0) {
			auto pedestrian = groundTruth->add_moving_object();
			auto walkerActor = boost::static_pointer_cast<const carla::client::Walker>(actor);

			pedestrian->set_model_reference(walkerActor->GetTypeId());
			pedestrian->set_allocated_id(carla_osi::id_mapping::getOSIActorId(walkerActor).release());
			pedestrian->set_type(osi3::MovingObject_Type_TYPE_PEDESTRIAN);
			pedestrian->set_allocated_base(CarlaUtility::toOSIBaseMoving(walkerActor).release());

			//TODO How to determine a lane for pedestrians? Carla walkers don't care about lanes and walk on meshes with specific names (see https://carla.readthedocs.io/en/0.9.9/tuto_D_generate_pedestrian_navigation/):
			// Road_Sidewalk, Road_Crosswalk, Road_Grass, Road_Road, Road_Curb, Road_Gutter or Road_Marking 

			//The following will only work if there is a matching waypoint:
			auto closestWaypoint = map->GetWaypoint(walkerActor->GetLocation(), true, (uint32_t)carla::road::Lane::LaneType::Any);
			if (closestWaypoint) {
				pedestrian->mutable_assigned_lane_id()->AddAllocated(
					carla_osi::id_mapping::getOSIWaypointId(closestWaypoint).release());
			}
		}
		/*else if ("traffic.traffic_light" == typeID) {
			carla::SharedPtr<const carla::client::TrafficLight> trafficLight = boost::dynamic_pointer_cast<carla::client::TrafficLight>(actor);
			//TODO parse carla::client::TrafficLight as a set of osi3::TrafficLight
			//a osi3::TrafficLight describes a single bulb of a traffic light

			//TODO retrieve traffic light heads and parse them to osi traffic lights instead of using a static guess.
			auto heads = world->GetTrafficLightHeads(trafficLight);
			auto bulbs = carla_osi::traffic_signals::getOSITrafficLight(trafficLight, heads);
			//add converted bulbs to ground truth
			auto trafficLights = groundTruth->mutable_traffic_light();
			for (auto& bulb : bulbs) {
				trafficLights->AddAllocated(bulb.release());
			}
		}*/
		else {
			if (runtimeParameter.verbose)
				std::cout << typeID << " not parsed to groundtruth." << std::endl;
		}
	}

	//Timestamp
	osi3::Timestamp* timestamp = parseTimestamp();
	groundTruth->set_allocated_timestamp(timestamp);

	return groundTruth;
}

void CARLA2OSIInterface::clearData()
{
	if (!world) {
		std::cerr << "No world" << std::endl;
		throw new std::exception();
	}
	{//mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
		actorRole2IDMap.clear();
		varName2MessageMap.clear();
	}
	staticMapTruth->Clear();
	parseStationaryMapObjects();
}

void CARLA2OSIInterface::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData)
{
	if (!world) {
		// Local world object has been destroyed and thus probably also the CARLA OSI interface, but client is still sending
		// Stop listening to this sensor
		sensor->Stop();
		return;
	}
	if (world->GetId() != sensor->GetWorld().GetId()) {
		std::cerr << __FUNCTION__ << ": received event for wrong world" << std::endl;
		return;
	}

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
	else if (runtimeParameter.verbose) {
		std::cerr << "CARLA2OSIInterface::sensorEventAction called for unsupported sensor type" << std::endl;
	}

	{//Mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
		auto iter = actorRole2IDMap.right.find(sensor->GetId());
		if (iter != actorRole2IDMap.right.end()) {
			std::string varName = iter->second;
			varName2MessageMap[varName] = std::move(sensorView);
		}
		else if (runtimeParameter.verbose) {
			std::cerr << __FUNCTION__ << ": received event for unknown sensor with id " << sensor->GetId() << std::endl;
		}
	}
}

void CARLA2OSIInterface::sendTrafficCommand(carla::ActorId ActorId) {
	std::unique_ptr<osi3::TrafficCommand> trafficCommand = std::make_unique<osi3::TrafficCommand>();
	trafficCommand->set_allocated_traffic_participant_id(carla_osi::id_mapping::toOSI(ActorId).release());
	osi3::Timestamp* timestamp = parseTimestamp();
	trafficCommand->set_allocated_timestamp(timestamp);
	auto trafficAction = trafficCommand->add_action();

	//do action accordingly
	int TrafficActionType = 0;//TODO Placeholder at the moment
	//hero has a routing action in OpenScenario
	//Routing Action has an Oriented Point

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

	{// mutex scope
		//using a scoped mutex - locking multiple mutexes could cause deadlock
		std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
		auto varName = actorRole2IDMap.right.at(ActorId);
		varName2MessageMap[varName] = std::move(trafficCommand);
	}

	delete timestamp;
}

int CARLA2OSIInterface::receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate) {
	//OSI documentation:
	//Only the id, base member (without dimension and base_polygon),
	//and the vehicle_classification.light_state members are considered in
	//updates, all other members can be left undefined, and wiudll be
	//ignored by the receiver of this message.

	if (trafficUpdate.update_size() == 0) {
		std::cerr << "CARLA2OSIInterface.receiveTrafficUpdate No update." << std::endl;
		return 3;
	}
	for (auto& update : trafficUpdate.update()) {
		auto TrafficId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
		auto actor = world->GetActor(TrafficId);
		if (actor == nullptr) {
			std::cout << "Actor not found! No position updates will be done!" << std::endl;
			return 0;
		}
		if (TrafficId != actor->GetId()) {
			std::cerr << "CARLA2OSIInterface.receiveTrafficUpdate: No actor with id" << TrafficId << std::endl;
			return 2;
		}


		//BASE
		if (update.base().has_position()
			&& update.base().has_orientation()) {
			auto position = carla_osi::geometry::toCarla(&update.base().position());
			auto orientation = carla_osi::geometry::toCarla(&update.base().orientation());
			//do not set height, pitch an roll of vehicles in asynchronous mode
			//these would break the visualization
			//Generally you should not set any positions in an asychronous simulation, since the physics will go crazy because of artificial high accelerations
			if (!runtimeParameter.sync) {
				position.z = actor->GetLocation().z;
				orientation.pitch = actor->GetTransform().rotation.pitch;
				orientation.roll = actor->GetTransform().rotation.roll;
			}
			actor->SetTransform(carla::geom::Transform(position, orientation));
		}

		//Velocity
		if (update.base().has_velocity()) {
			actor->SetTargetVelocity(carla_osi::geometry::toCarla(&update.base().velocity()));
		}

		//Acceleration can not be set in CARLA
		//GetAcceleration() calculates the acceleration with the actor's velocity
		//if (update.mutable_base()->has_acceleration()) {
			//auto acceleration = carla_osi::geometry::toCarla(&update.mutable_base()->acceleration());
		//}

		//Orientation
		if (update.base().has_orientation_rate()) {
			const auto orientationRate = carla_osi::geometry::toCarla(&update.base().orientation_rate());

			//TODO Check if conversion is correct: x should be forward, y should be up, z should be right
			actor->SetTargetAngularVelocity({ orientationRate.GetForwardVector().Length(), orientationRate.GetUpVector().Length(), orientationRate.GetRightVector().Length() });
		}

		//Acceleration can not be set in CARLA
		//GetAcceleration() calculates the acceleration with the actor's velocity
		//if (update.mutable_base()->has_orientation_acceleration()){
			//const osi3::Orientation3d* accelerationRoll = update.mutable_base()->mutable_orientation_acceleration();
		//}

		//LIGHTSTATE
		if (update.vehicle_classification().has_light_state()) {
			auto classification = update.vehicle_classification();
			auto light_state = classification.mutable_light_state();
			auto carla_light_state  = CarlaUtility::toCarla(light_state);
			//auto indicatorState = CarlaUtility::toCarla(update.vehicle_classification().light_state());
			auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);

			vehicleActor->SetLightState(carla_light_state);
		}
	}
	return 0;
}

void CARLA2OSIInterface::writeLog() {
	if (!logFile.is_open()) {
		logFile.open(runtimeParameter.logFileName);
		if (runtimeParameter.verbose) {
			std::cout << "Write to " << runtimeParameter.logFileName << std::endl;
		}
		std::string header = "Timestamp,Actor1_ID,Actor1_x,Actor1_y,Actor1_heading,Actor2_ID,Actor2_x,Actor2_y,Actor2_heading,Actor3_ID,Actor3_x,Actor3_y,Actor3_heading,Actor4_ID,Actor4_x,Actor4_y,Actor4_heading,Actor5_ID,Actor5_x,Actor5_y,Actor5_heading\n";
		logFile << header;
		std::cout << header;
	}

	//parseWorldToGroundTruth();//to get the data from the osi representation???

	if (!allXActorsSpawned) {
		//ego
		actors[0].id = std::to_string(latestGroundTruth->host_vehicle_id().value());

		//other actors
		for (const auto& movingObject : latestGroundTruth->moving_object()) {
			for (auto& logData : actors) {
				if (logData.id != "NaN") {
					if (logData.id == std::to_string(movingObject.id().value())) {
						//already inserted
						break;
					}
				}
				else {
					//insert id
					logData.id = std::to_string(movingObject.id().value());
					break;
				}
			}
		}
		if (latestGroundTruth->moving_object_size() == actors.size()) {
			allXActorsSpawned = true;
		}
	}

	//log all vehicles
	for (const auto& movingObject : latestGroundTruth->moving_object()) {
		for (auto& logData : actors) {
			if (logData.id == std::to_string(movingObject.id().value())) {
				logData.x = movingObject.base().position().x();
				logData.y = movingObject.base().position().y();
				logData.yaw = movingObject.base().orientation().yaw() * 180 / M_PI;
			}
		}
	}

	//write all data
	char separator = ',';
	double seconds = world->GetSnapshot().GetTimestamp().elapsed_seconds;
	logFile << seconds;//time as floating point
	std::cout << seconds;

	for (const auto& logData : actors) {
		logFile << separator<< logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw ;
		std::cout << separator << logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw;
	}
	//end line and flush data
	logFile << std::endl;
	std::cout << std::endl;
}
