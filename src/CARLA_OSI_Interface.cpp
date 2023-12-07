#include "CARLA_OSI_Interface.h"

int CARLAOSIInterface::initialise(RuntimeParameter& runtimeParams, std::shared_ptr<CARLAInterface> carla){
	this->runtimeParameter = runtimeParams;
	this->carla = carla;
	return 0;
}

double CARLAOSIInterface::doStep() {
	if (runtimeParameter.verbose) {
		std::cout << "Do Step" << std::endl;
	}
	if (!carla->world) {
		std::cerr << "No world" << std::endl;
		throw std::exception();
	}
	//tick not needed if in asynchronous mode
	if (runtimeParameter.sync) {
		//Length of simulationed tick is set in applyWorldSettings()
		carla->world->Tick(carla->client->GetTimeout());
	}
	//carla->world->WaitForTick(this->transactionTimeout);
	validLatestGroundTruth = false;

	// only accurate if using fixed time step, as activated during initialise()
	return carla->world->GetSnapshot().GetTimestamp().delta_seconds;
}

void CARLAOSIInterface::fetchActorsFromCarla() {

	// track actors added/removed inside Carla
	std::set<carla::ActorId> worldActorIDs, addedActors, removedActors;
	auto worldActors = carla->world->GetActors();
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
			auto actor = carla->world->GetActor(addedActor);
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
		props = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Any);
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
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Buildings);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Fences) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Fences);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Other) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Other);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Poles) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Poles);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.RoadLines) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::RoadLines);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Roads) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Roads);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Sidewalks) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Sidewalks);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Vegetation) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Vegetation);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Walls) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Walls);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Ground) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Ground);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Bridge) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Bridge);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.RailTrack) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::RailTrack);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.GuardRail) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::GuardRail);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Static) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Static);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Water) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Water);
			props.insert(props.end(), buildings.begin(), buildings.end());
		}
		if (runtimeParameter.options.Terrain) {
			auto buildings = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::Water);
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
	staticMapTruth->set_map_reference(carla->map->GetName());
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
	auto signs = carla->world->GetEnvironmentObjects((uint8_t)carla::rpc::CityObjectLabel::TrafficSigns);
	for (auto& sign : signs) {
		//auto trafficSign = carla->world->GetActor((carla::ActorId)sign.id);
		//carla::SharedPtr<carla::client::TrafficSign> carlaTrafficSign = boost::dynamic_pointer_cast<carla::client::TrafficSign>(trafficSign);
		auto OSITrafficSign = carla_osi::traffic_signals::getOSITrafficSign(sign);
		OSITrafficSigns->AddAllocated(OSITrafficSign.release());
	}

	if (runtimeParameter.mapNetworkInGroundTruth) {
		auto lanes = staticMapTruth->mutable_lane();
		auto laneBoundaries = staticMapTruth->mutable_lane_boundary();
		auto topology = carla->map->GetTopology();
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

		const carla::road::Map& roadMap = carla->map->GetMap();

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
	std::cout << "Finished parsing of stationary map objects." << std::endl;
}

std::shared_ptr<osi3::GroundTruth> CARLAOSIInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use staticMapTruth as a base for every new groundTruth message that already contains unchanging fields
	std::shared_ptr<osi3::GroundTruth> groundTruth = std::make_shared<osi3::GroundTruth>();
	groundTruth->MergeFrom(*staticMapTruth);

	auto worldActors = carla->world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();
		//std::cout << "Ground Truth: " << typeID  << "  " << actor->GetId() << std::endl;
		bool destroyed = false;
		for(auto& v: carla->deletedVehicles) {
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
			auto waypoint = carla->map->GetWaypoint(vehicleActor->GetLocation());
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
			auto closestWaypoint = carla->map->GetWaypoint(walkerActor->GetLocation(), true, (uint32_t)carla::road::Lane::LaneType::Any);
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
			auto heads = carla->world->GetTrafficLightHeads(trafficLight);
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
	groundTruth->set_allocated_timestamp(CarlaUtility::parseTimestamp(carla->world->GetSnapshot().GetTimestamp()).release());

	return groundTruth;
}

void CARLAOSIInterface::clearData()
{
	if (!carla->world) {
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
	if (!carla->world) {
		// Local world object has been destroyed and thus probably also the CARLA OSI interface, but client is still sending
		// Stop listening to this sensor
		sensor->Stop();
		return;
	}
	if (carla->world->GetId() != sensor->GetWorld().GetId()) {
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
	for (auto& spawnedVehicle : carla->spawnedVehicles) {
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

	double seconds = carla->world->GetSnapshot().GetTimestamp().elapsed_seconds;

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
