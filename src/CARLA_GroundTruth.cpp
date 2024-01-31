#include "CARLA_GroundTruth.h"

std::shared_ptr<const osi3::GroundTruth> GroundTruthCreator::getLatestGroundTruth()
{
	if (!validLatestGroundTruth) {
		latestGroundTruth = parseWorldToGroundTruth();
		validLatestGroundTruth = true;
	}
	return latestGroundTruth;
}

void GroundTruthCreator::parseStationaryMapObjects()
{
	staticMapTruth = std::make_unique<osi3::GroundTruth>();
	auto OSIStationaryObjects = staticMapTruth->mutable_stationary_object();
	auto filteredStationaryMapObjects = filterEnvironmentObjects();
	for (auto& mapObject : filteredStationaryMapObjects) {
		OSIStationaryObjects->AddAllocated(
			CarlaUtility::toOSI(mapObject, runtimeParameter.verbose));
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
#if defined(_WIN32) && (_MSC_VER >= 1910) || defined(__linux__) && __cplusplus >= 201703L
		std::for_each(std::execution::par, combined.begin(), combined.end(), [&](zip_type::value_type& tuple) {
			auto&[endpoints, lane, boundaries] = tuple;
			auto&[laneStart, laneEnd] = *endpoints;
#elif defined(_WIN32) && (_MSC_VER >= 1600) || defined(__linux__) && __cplusplus >= 201103L
		boost::range::for_each(combined, [&](zip_type::value_type& tuple) {
			auto& endpoints = std::get<0>(tuple);
			auto& lane = std::get<1>(tuple);
			auto& boundaries = std::get<2>(tuple);
			auto& laneStart = std::get<0>(*endpoints);
			auto& laneEnd = std::get<1>(*endpoints);
#endif
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
				for (const auto& waypoint : waypoints) {
					const auto& inbound = std::get<0>(waypoint);
					const auto& outbound = std::get<1>(waypoint);
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
#if defined(_WIN32) && (_MSC_VER >= 1910) || defined(__linux__) && __cplusplus >= 201703L
				auto&[parsedBoundaries, left_lane_boundary_id, right_lane_boundary_id] = boundary;
#elif defined(_WIN32) && (_MSC_VER >= 1600) || defined(__linux__) && __cplusplus >= 201103L
				auto& parsedBoundaries = std::get<0>(boundary);
				auto& left_lane_boundary_id = std::get<1>(boundary);
				auto& right_lane_boundary_id = std::get<2>(boundary);
#endif
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
			//get<0> not used
			auto& lane = std::get<1>(zipped);
			auto& boundaries = std::get<2>(zipped);
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

std::shared_ptr<osi3::GroundTruth> GroundTruthCreator::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use staticMapTruth as a base for every new groundTruth message that already contains unchanging fields
	std::shared_ptr<osi3::GroundTruth> groundTruth = std::make_shared<osi3::GroundTruth>();
	groundTruth->MergeFrom(*staticMapTruth);

	//carla::SharedPtr<carla::client::ActorList>
	auto worldActors = carla->world->GetActors();
	for (auto actor : *worldActors) {
		auto typeID = actor->GetTypeId();
		//std::cout << "Ground Truth: " << typeID  << "  " << actor->GetId() << std::endl;
		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth->add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);

			vehicle->set_model_reference(vehicleActor->GetTypeId());
			//if a vehicle is spawned by TrafficUpdate, then the ID from that OSI message shall be used, not the ID given by Carla when spawned.
			OSIVehicleID spawnedVehicleID = vehicleIsSpawned(vehicleActor);
			if (spawnedVehicleID) {
				vehicle->mutable_id()->set_value(spawnedVehicleID);
				if (runtimeParameter.ego == std::to_string(spawnedVehicleID)) {
					groundTruth->mutable_host_vehicle_id()->set_value(spawnedVehicleID);
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

std::vector<carla::rpc::EnvironmentObject> GroundTruthCreator::filterEnvironmentObjects() {

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
