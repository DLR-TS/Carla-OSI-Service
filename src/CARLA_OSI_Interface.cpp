#include "CARLA_OSI_Interface.h"

int CARLAOSIInterface::initialise(RuntimeParameter& runtimeParams) {
	this->runtimeParameter = runtimeParams;

	try {
		//connect
		this->client = std::make_unique<carla::client::Client>(runtimeParams.carlaHost, runtimeParameter.carlaPort);
		this->client->SetTimeout(std::chrono::duration<double>(runtimeParams.transactionTimeout));

		loadWorld();
		applyWorldSettings();
		parseStationaryMapObjects();

		if (runtimeParameter.replay.enabled) {
			fillBoundingBoxLookupTable();
		}
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		return -1;
	}

	// perform a tick to fill actor and message lists
	doStep();
	return 0;
}

double CARLAOSIInterface::doStep() {
	if (runtimeParameter.verbose) {
		std::cout << "Do Step" << std::endl;
	}
	if (!world) {
		std::cerr << "No world" << std::endl;
		throw std::exception();
	}
	else if (this->world->GetId() != this->client->GetWorld().GetId()) {
		// change of world id indicates map reload or map change
		std::cerr << "World has changed" << std::endl;
		this->clearData();
	}
	//tick not needed if in asynchronous mode
	if (runtimeParameter.sync) {
		//Length of simulationed tick is set in applyWorldSettings()
		world->Tick(client->GetTimeout());
	}
	//world->WaitForTick(this->transactionTimeout);
	validLatestGroundTruth = false;

	// only accurate if using fixed time step, as activated during initialise()
	return world->GetSnapshot().GetTimestamp().delta_seconds;
}

void CARLAOSIInterface::fillBoundingBoxLookupTable() {
	auto vehicleLibrary = world->GetBlueprintLibrary()->Filter("vehicle.*");
	carla::geom::Location location(0, 0, 0);
	carla::geom::Rotation rotation(0, 0, 0);
	carla::geom::Transform transform(location, rotation);
	for (auto vehicle : *vehicleLibrary) {
		auto temp_actor = world->SpawnActor(vehicle, transform);
		auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(temp_actor);
		auto bbox = vehicleActor->GetBoundingBox();
		replayVehicleBoundingBoxes.emplace_back(vehicle.GetId(), bbox.extent);
		world->Tick(client->GetTimeout());
		temp_actor->Destroy();
	}
	if (runtimeParameter.verbose) {
		std::cout << "Number of possible vehicle bounding boxes: " << replayVehicleBoundingBoxes.size() << std::endl;
	}
}

void CARLAOSIInterface::fetchActorsFromCarla() {

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
							std::cout << "add sensor "  << actor->GetTypeId() << std::endl;
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							int index = (int)sensorCache.size();
							sensorCache.emplace(index, nullptr);
							sensor->Listen([this, sensor, index](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData, index); });
						}
					}
					break;
				}
			}
		}
	}
}

std::shared_ptr<const osi3::GroundTruth> CARLAOSIInterface::getLatestGroundTruth()
{
	if (!validLatestGroundTruth) {
		latestGroundTruth = parseWorldToGroundTruth();
		validLatestGroundTruth = true;
	}
	return latestGroundTruth;
}

void CARLAOSIInterface::loadWorld() {
	this->world = std::make_unique<carla::client::World>(std::move(this->client->GetWorld()));
	this->map = world->GetMap();
}

void CARLAOSIInterface::applyWorldSettings() {
	auto settings = world->GetSettings();
	//set sync or async operational mode
	settings.synchronous_mode = runtimeParameter.sync;

	if (settings.fixed_delta_seconds.has_value() &&
		settings.fixed_delta_seconds.value() == runtimeParameter.deltaSeconds &&
		settings.synchronous_mode) {
		if (runtimeParameter.verbose) {
			std::cout << "Settings of Carla Server are already correct and do not need to be changed" << std::endl;
		}
		return;
	}
	settings.fixed_delta_seconds = runtimeParameter.deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings, settingsDuration);
}

void CARLAOSIInterface::resetWorldSettings() {
	auto settings = world->GetSettings();
	settings.synchronous_mode = false;
	this->world->ApplySettings(settings, settingsDuration);
	if (runtimeParameter.verbose) {
		std::cout << "Reset CARLA World Settings." << std::endl;
	}
}

std::shared_ptr<const osi3::SensorViewConfiguration> CARLAOSIInterface::getSensorViewConfiguration(const std::string& sensor)
{
	//string has format of: OSMPSensorViewConfigurationX
	std::string index_string(&sensor[27]);
	int index = std::stoi(index_string);
	//todo a
	return nullptr;
}

std::shared_ptr<const osi3::SensorView> CARLAOSIInterface::getSensorView(const std::string& sensor)
{
	//string has format of: OSMPSensorViewX
	std::string index_string(&sensor[14]);
	int index = std::stoi(index_string);
	{
		// mutex scope: using a shared lock - read only access
		std::shared_lock lock(sensorCache_mutex);
		auto iter = sensorCache.find(index);
		if (iter == sensorCache.end()) {
			return nullptr;
		}
		return iter->second;
	}
}

std::string CARLAOSIInterface::actorIdToRoleName(const osi3::Identifier& id)
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

std::vector<carla::rpc::EnvironmentObject> CARLAOSIInterface::filterEnvironmentObjects() {

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

void CARLAOSIInterface::parseStationaryMapObjects()
{
	staticMapTruth = std::make_unique<osi3::GroundTruth>();
	staticMapTruth->set_map_reference(map->GetName());
	auto OSIStationaryObjects = staticMapTruth->mutable_stationary_object();
	auto filteredStationaryMapObjects = filterEnvironmentObjects();
	for (auto& mapObject : filteredStationaryMapObjects) {
		OSIStationaryObjects->AddAllocated(
			CarlaUtility::toOSI(mapObject, runtimeParameter.verbose));
	}

	for (auto& mapObject : filteredStationaryMapObjects) {
		OSIStationaryObjects->AddAllocated(CarlaUtility::toOSI(mapObject, runtimeParameter.verbose));
	}
	auto OSITrafficSigns = staticMapTruth->mutable_traffic_sign();
	auto signs = world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::TrafficSigns);
	for (auto& sign : signs) {
		//auto trafficSign = world->GetActor((carla::ActorId)sign.id);
		//carla::SharedPtr<carla::client::TrafficSign> carlaTrafficSign = boost::dynamic_pointer_cast<carla::client::TrafficSign>(trafficSign);
		auto OSITrafficSign = carla_osi::traffic_signals::getOSITrafficSign(sign);
		OSITrafficSigns->AddAllocated(OSITrafficSign.release());
	}

	if (runtimeParameter.mapNetworkInGroundTruth) {
		auto lanes = staticMapTruth->mutable_lane();
		auto laneBoundaries = staticMapTruth->mutable_lane_boundary();
		auto topology = map->GetTopology();
		lanes->Reserve((int)topology.size());
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
				lanePairings->Reserve((int)waypoints.size());
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
					centerline->Reserve((int)waypoints.size());
					for (const auto& waypoint : waypoints) {
						auto location = waypoint->GetTransform().location;
						centerline->AddAllocated(Geometry::getInstance()->toOSI(location).release());
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

std::shared_ptr<osi3::GroundTruth> CARLAOSIInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use staticMapTruth as a base for every new groundTruth message that already contains unchanging fields
	std::shared_ptr<osi3::GroundTruth> groundTruth = std::make_shared<osi3::GroundTruth>();
	groundTruth->MergeFrom(*staticMapTruth);

	auto worldActors = world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();
		//std::cout << "Ground Truth: " << typeID  << "  " << actor->GetId() << std::endl;
		bool destroyed = false;
		for(auto& v: deletedVehicles) {
			if (v.second.idInCarla == actor->GetId())
			{
				destroyed = true;
				break;
			}
		}
		if (destroyed){ continue; }
		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth->add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);

			vehicle->set_model_reference(vehicleActor->GetTypeId());
			OSIVehicleID spawnedVehicleID = vehicleIsSpawned(vehicleActor);
			if (spawnedVehicleID) {
				vehicle->mutable_id()->set_value(spawnedVehicleID);
				if (runtimeParameter.ego == std::to_string(spawnedVehicleID)) {
					groundTruth->mutable_host_vehicle_id()->set_value(spawnedVehicleID);
					//saved for trafficCommand message from scenario runner
					trafficCommandMessageHeroId = spawnedVehicleID;
				}
			}
			else {
				//not a spawned vehicle, use carla id mapping system
				vehicle->set_allocated_id(carla_osi::id_mapping::getOSIActorId(vehicleActor).release());
			}
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
					if (runtimeParameter.ego == attribute.GetValue()) {
						groundTruth->mutable_host_vehicle_id()->set_value(vehicle->id().value());
						//saved for trafficCommand message from scenario runner
						trafficCommandMessageHeroId = vehicle->id().value();
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
	groundTruth->set_allocated_timestamp(CarlaUtility::parseTimestamp(world->GetSnapshot().GetTimestamp()).release());

	return groundTruth;
}

void CARLAOSIInterface::clearData()
{
	if (!world) {
		std::cerr << "No world" << std::endl;
		throw new std::exception();
	}
	{//mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex, sensorCache_mutex);
		actorRole2IDMap.clear();
		sensorCache.clear();
	}
	staticMapTruth->Clear();
	parseStationaryMapObjects();
}

void CARLAOSIInterface::sensorEventAction(carla::SharedPtr<carla::client::Sensor> sensor, carla::SharedPtr<carla::sensor::SensorData> sensorData, int index)
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
	//substring of typeID: sensor.camera.rgb -> camera.rgb
	std::string_view sensorType(&typeID[7]);

	if (0 == sensorType.rfind("camera.rgb", 0))
	{
		if (runtimeParameter.carlasensortypes.find(CAMERA) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(sensorData);
		auto cameraSensorView = CarlaUtility::toOSICamera(sensor, image);
		sensorView->mutable_camera_sensor_view()->AddAllocated(cameraSensorView);
	}
	else if (0 == sensorType.rfind("lidar.ray_cast", 0))
	{
		if (runtimeParameter.carlasensortypes.find(LIDAR) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(sensorData);
		auto lidarSensorView = CarlaUtility::toOSILidar(sensor, measurement);
		sensorView->mutable_lidar_sensor_view()->AddAllocated(lidarSensorView);
	}
	else if (0 == sensorType.rfind("other.radar", 0))
	{
		if (runtimeParameter.carlasensortypes.find(RADAR) == runtimeParameter.carlasensortypes.end()) {
			return;
		}
		auto measurement = boost::dynamic_pointer_cast<carla::sensor::data::RadarMeasurement>(sensorData);
		auto radarSensorView = CarlaUtility::toOSIRadar(sensor, measurement);
		sensorView->mutable_radar_sensor_view()->AddAllocated(radarSensorView);
	}
	else if (runtimeParameter.verbose) {
		std::cerr << "CARLAOSIInterface::sensorEventAction called for unsupported sensor type" << std::endl;
	}
	sensorCache[index] = std::move(sensorView);
	if (runtimeParameter.verbose) {
		std::cout << "Update " << sensorType << " with index " << index << "." << std::endl;
	}
}

int CARLAOSIInterface::receiveTrafficUpdate(osi3::TrafficUpdate& trafficUpdate) {
	//OSI documentation:
	//Only the id, base member (without dimension and base_polygon),
	//and the vehicle_classification.light_state members are considered in
	//updates, all other members can be left undefined, and will be
	//ignored by the receiver of this message.

	if (trafficUpdate.update_size() == 0) {
		std::cerr << __FUNCTION__ << " No update." << std::endl;
		return 3;
	}

	carla::ActorId actorId;

	if (runtimeParameter.replay.enabled) {
		replayTrafficUpdate(trafficUpdate, actorId);
		return 0;
	}

	for (auto& update : trafficUpdate.update()) {
		//original operation mode with update for exisiting cars

		actorId = std::get<carla::ActorId>(carla_osi::id_mapping::toCarla(&update.id()));
		auto actor = world->GetActor(actorId);
		if (actor == nullptr && !runtimeParameter.replay.enabled) {
			std::cout << "Actor not found! No position updates will be done!" << std::endl;
			return 0;
		}
		applyTrafficUpdate(update, actor);
	}
	return 0;
}

void CARLAOSIInterface::replayTrafficUpdate(const osi3::TrafficUpdate& trafficUpdate, carla::ActorId& actorID) {
	//check if spawned from Carla-OSI-Service
	
	for (auto& update : trafficUpdate.update()) {
		auto ActorID = spawnedVehicles.find(update.id().value());
		if (ActorID == spawnedVehicles.end()) {
			//not found --> find best matching representation
			const osi3::Dimension3d dimension = update.base().dimension();

			size_t minDiffVehicleIndex = 0;

			double minTotalDiff = DBL_MAX;
			double minTotalDiffLength = 0, minTotalDiffWidth = 0, minTotalDiffHeight = 0;

			for (int i = 0; i < replayVehicleBoundingBoxes.size(); i++) {
				auto& boundingBox = std::get<1>(replayVehicleBoundingBoxes[i]);

				double diffLength = dimension.length() - boundingBox.x;
				double diffWidth = dimension.width() - boundingBox.y;
				double diffHeight = dimension.height() - boundingBox.z;

				double sumDiff = runtimeParameter.replay.weightLength_X * std::abs(diffLength);
				sumDiff += runtimeParameter.replay.weightWidth_Y * std::abs(diffWidth);
				sumDiff += runtimeParameter.replay.weightHeight_Z * std::abs(diffHeight);

 				if (sumDiff < minTotalDiff) {
					minDiffVehicleIndex = i;
					minTotalDiff = sumDiff;

					minTotalDiffLength = diffLength;
					minTotalDiffWidth = diffWidth;
					minTotalDiffHeight = diffHeight;
				}
			}

			std::cout << "Search for vehicle with length: " << dimension.length() << ", width: " << dimension.width()
				<< ", height: " << dimension.height() <<
				" Spawn vehicle with length: " << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).x << ", width:" << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).y
				<< ", height:" << std::get<1>(replayVehicleBoundingBoxes[minDiffVehicleIndex]).z << std::endl;

			auto position = Geometry::getInstance()->toCarla(update.base().position());
			position.z = runtimeParameter.replay.spawnHeight_Z;

			auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
			carla::geom::Transform transform(position, orientation);

			//spawn actor
			auto blueprintlibrary = world->GetBlueprintLibrary();
			auto search = std::get<0>(replayVehicleBoundingBoxes[minDiffVehicleIndex]);
			std::cout << "Spawn vehicle: " << search << " Position: " << position.x << ", " << position.y  << ", " << position.z << std::endl;

			auto vehicleBlueprint = blueprintlibrary->Find(search);
			carla::SharedPtr<carla::client::Actor> actor;
			actor = world->TrySpawnActor(*vehicleBlueprint, transform);
			if (actor == nullptr) {
				std::cerr << "Spawn vehicle: " << search << " Position: " << position.x << ", " << position.y  << ", " << position.z << std::endl;
				std::cerr << "Could not spawn actor!" << std::endl;
				continue;
			}

			spawnedVehicle addedVehicle;
			addedVehicle.idInCarla = actor->GetId();
			spawnedVehicles.emplace(update.id().value(), addedVehicle);
			applyTrafficUpdate(update, actor);
		} else {
			applyTrafficUpdate(update, world->GetActor(ActorID->second.idInCarla));
		}
		//save the Update with current timestamp
		spawnedVehicles[update.id().value()].lastTimeUpdated = trafficUpdate.timestamp().seconds() * 1000000000u + trafficUpdate.timestamp().nanos();
	}

	//deconstruct actors with no update
	for (auto& vehicle : spawnedVehicles) {
		if (vehicle.second.lastTimeUpdated != trafficUpdate.timestamp().seconds() * 1000000000u + trafficUpdate.timestamp().nanos()) {
			std::cout << "No update for vehicle: " << vehicle.first << " Will stop the display of this vehicle." << std::endl;
			auto vehicleActor = world->GetActor(vehicle.second.idInCarla);
			deletedVehicles.emplace(vehicle);
			vehicleActor->Destroy();
		}
	}
	for (auto& deletedVehicleId : deletedVehicles) {
		spawnedVehicles.erase(deletedVehicleId.first);
	}
}

void CARLAOSIInterface::applyTrafficUpdate(const osi3::MovingObject& update, carla::SharedPtr<carla::client::Actor> actor)
{
	//BASE
	if (update.base().has_position() && update.base().has_orientation()) {
		auto position = Geometry::getInstance()->toCarla(update.base().position());
		auto orientation = Geometry::getInstance()->toCarla(update.base().orientation());
		if (runtimeParameter.replay.enabled){
			position.z = runtimeParameter.replay.spawnHeight_Z;
		}

		//heigth is controlled by terrain
		if (actor->GetLocation().z != 0) {
			position.z = actor->GetLocation().z;
		}
		else {
			//new height is spawnHeight
		}

		//do not set pitch an roll of vehicles in asynchronous mode
		//these would break the visualization
		//Generally you should not set any positions in an asychronous simulation, since the physics will go crazy because of artificial high accelerations
		if (!runtimeParameter.sync) {
			orientation.pitch = actor->GetTransform().rotation.pitch;
			orientation.roll = actor->GetTransform().rotation.roll;
		}
		if (runtimeParameter.verbose) {
			std::cout << "Apply position: " << position.x << ", " << position.y  << ", " << position.z << std::endl;
		}
		actor->SetTransform(carla::geom::Transform(position, orientation));
	}

	//Velocity
	if (update.base().has_velocity()) {
		actor->SetTargetVelocity(Geometry::getInstance()->toCarlaVelocity(update.base().velocity()));
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//if (update.mutable_base()->has_acceleration()) {
	//auto acceleration = carla_osi::geometry::toCarla(&update.mutable_base()->acceleration());
	//}

	//Orientation
	if (update.base().has_orientation_rate()) {
		const auto orientationRate = Geometry::getInstance()->toCarla(update.base().orientation_rate());

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
		auto carla_light_state = CarlaUtility::toCarla(light_state);
		//auto indicatorState = CarlaUtility::toCarla(update.vehicle_classification().light_state());
		auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);

		vehicleActor->SetLightState(carla_light_state);
	}
}

void CARLAOSIInterface::deleteSpawnedVehicles() {
	//deconstruct all spawned actors
	for (auto& vehicle : spawnedVehicles) {
		std::cout << "Remove: " << vehicle.first << std::endl;
		auto vehicleActor = world->GetActor(vehicle.second.idInCarla);
		deletedVehicles.emplace(vehicle);
		vehicleActor->Destroy();
	}
}

int CARLAOSIInterface::receiveSensorViewConfigurationRequest(osi3::SensorViewConfiguration& sensorViewConfiguration) {
	//todo a
	for (auto& cameraSensorConfiguration : sensorViewConfiguration.camera_sensor_view_configuration()) {
		// todo
	}
	for (auto& lidarSensorConfiguration : sensorViewConfiguration.lidar_sensor_view_configuration()) {
		// todo
	}
	for (auto& radarSensorConfiguration : sensorViewConfiguration.radar_sensor_view_configuration()) {
		// todo
	}
	for (auto& ultrasonicSensorConfiguration : sensorViewConfiguration.ultrasonic_sensor_view_configuration()) {
		// todo
	}
	for (auto& genericSensorConfiguration : sensorViewConfiguration.generic_sensor_view_configuration()) {
		// todo
	}
	//sensorViewConfiguration.field_of_view_horizontal
	//sensorViewConfiguration.field_of_view_vertical
	//sensorViewConfiguration.update_cycle_offset
	//sensorViewConfiguration.mounting_position
	//sensorViewConfiguration.mounting_position_rmse
	//sensorViewConfiguration.range
	//sensorViewConfiguration.update_cycle_time
	//sensorViewConfiguration.update_cycle_offset

	//spawn sensor and attach to vehicle, vehicle should have name: runtimeparameter.ego
	//add cache entry from fetchActorsFromCarla() and remove that function and its then useless subfunctions
	//save applied sensorviewconfiguration so that getSensorViewConfiguration() can retrieve the information
	return 0;
}

OSIVehicleID CARLAOSIInterface::vehicleIsSpawned(boost::shared_ptr<const carla::client::Vehicle> vehicle) {
	for (auto& spawnedVehicle: spawnedVehicles) {
		if (spawnedVehicle.second.idInCarla == vehicle->GetId()) {
			return spawnedVehicle.first;
		}
	}
	return 0;
}

void CARLAOSIInterface::writeLog() {

	std::string separator = ",";

	if (!logFile.is_open()) {
		logFile.open(runtimeParameter.logFileName);
		if (runtimeParameter.verbose) {
			std::cout << "Write to " << runtimeParameter.logFileName << std::endl;
		}
		std::string header = "Timestamp" + separator + "Actor_ID" + separator + 
			"Actor_x" + separator + "Actor_y" + separator + "Actor_heading\n";
		logFile << header;
		std::cout << header;
	}

	double seconds = world->GetSnapshot().GetTimestamp().elapsed_seconds;

	//log all vehicles
	logData logData;
	for (const auto& movingObject : latestGroundTruth->moving_object()) {
		logData.id = std::to_string(movingObject.id().value());
		logData.x = movingObject.base().position().x();
		logData.y = movingObject.base().position().y();
		logData.yaw = movingObject.base().orientation().yaw() * 180 / M_PI;

		logFile << seconds << separator
			<< logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw << "\n";
		std::cout << seconds << separator
			<< logData.id << separator
			<< logData.x << separator
			<< logData.y << separator
			<< logData.yaw << "\n";
	}
	//end line and flush data
	logFile << std::flush;
	std::cout << std::flush;
}
