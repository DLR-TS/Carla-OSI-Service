#include "CARLA2OSIInterface.h"

int CARLA2OSIInterface::initialise(std::string host, uint16_t port, double transactionTimeout, double deltaSeconds) {
	//connect
	this->client = std::make_unique<carla::client::Client>(host, port);
	this->client->SetTimeout(std::chrono::duration<double>(transactionTimeout));
	this->world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

	//assure server is in synchronous mode
	auto settings = world->GetSettings();
	settings.fixed_delta_seconds = deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings);

	parseStationaryMapObjects();

	// perform a tick to fill actor and message lists
	doStep();

	return 0;
}

double CARLA2OSIInterface::doStep() {
	if (!world) {
		throw std::exception("No world");
	}

	auto preStepTimestamp = world->GetSnapshot().GetTimestamp();

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

	world->Tick(client->GetTimeout());
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
						if (0 == actor->GetTypeId().rfind("sensor.", 0)) {
							auto sensor = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
							sensor->Listen([this, sensor](carla::SharedPtr<carla::sensor::SensorData> sensorData) {sensorEventAction(sensor, sensorData); });
						}
					}
					break;
				}
			}
		}
	}

	latestGroundTruth = parseWorldToGroundTruth();

	// only accurate if using fixed time step, as activated during initialise()
	return world->GetSnapshot().GetTimestamp().delta_seconds;
}

std::shared_ptr<const osi3::GroundTruth> CARLA2OSIInterface::getLatestGroundTruth()
{
	return latestGroundTruth;
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
	auto carlaTime = world->GetSnapshot().GetTimestamp();
	double intPart;
	double fractional = std::modf(carlaTime.elapsed_seconds, &intPart);
	osiTime->set_seconds(google::protobuf::int64(intPart));
	osiTime->set_nanos(google::protobuf::uint32(fractional *1e9));
	return osiTime;
}


void CARLA2OSIInterface::parseStationaryMapObjects()
{
	staticMapTruth = std::make_unique<osi3::GroundTruth>();

	carla::SharedPtr<carla::client::Map> map = world->GetMap();

	staticMapTruth->set_map_reference(map->GetName());

	// parse OpenDRIVE for retrieving information dropped in Carla
	auto result = xodr.load_string(map->GetOpenDrive().c_str());

	auto stationaryObjects = staticMapTruth->mutable_stationary_object();
	//TODO parse map parts that won't change during simulation

	// Static props apparently aren't part of the actor list, so this list is empty
	auto staticProps = world->GetActors()->Filter("static.prop.*");
	for each(auto prop in *staticProps) {
		// class Actor has no generic way of retrieving its bounding box -> custom api
		auto bbox = world->GetActorBoundingBox(prop->GetId());
		// parse as StationaryObject
		stationaryObjects->AddAllocated(CarlaUtility::toOSI(prop, bbox));
	}
	//TODO maybe parse Road Objects Record of OpenDrive file, if present - corresponds to OSI's StationaryObject

	std::vector<carla::rpc::StationaryMapObject> roadMarkings;

	auto stationaryMapObjects = world->GetStationaryMapObjects();
	for (auto& mapObject : stationaryMapObjects) {

		//TODO don't parse RoadMarkings as stationary object but add them to their lane
		if (mapObject.semantic_tag == carla::rpc::CityObjectLabel::RoadLines) {
			roadMarkings.push_back(std::move(mapObject));
			continue;
		}
		//TODO Skip meshes of roads and sidewalks, but not curbs

		auto stationaryObject = staticMapTruth->add_stationary_object();
		//TODO maybe keep a mapping of really unique actor FName to generated id
		// id of stationaryMapObject is generated per call of world->GetStationaryMapObjects and is always equal to the array index. Thus it is not really an identifier and cannot be mapped back to Unreal/Carla
		stationaryObject->set_allocated_id(CarlaUtility::toOSI(mapObject.id, CarlaUtility::CarlaUniqueID_e::StationaryMapObject));

		auto base = stationaryObject->mutable_base();
		auto[dimension, position] = CarlaUtility::toOSI(mapObject.bounding_box);
		base->set_allocated_dimension(dimension.release());
		base->set_allocated_position(CarlaUtility::toOSI(mapObject.transform.location));
		base->set_allocated_orientation(CarlaUtility::toOSI(mapObject.transform.rotation));

		//TODO base_polygon

		//TODO Carla doesn't seem to offer much information needed for osi3::StationaryObject::Classification.
		auto classification = stationaryObject->mutable_classification();//creates default instance as side-effect

		// Use the first tag that is not categorized as none or other

		carla::rpc::CityObjectLabel label = mapObject.semantic_tag;

		switch (mapObject.semantic_tag)
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
			//std::cerr << "Encountered an unmappable stationary map object of value " << (int)mapObject.semantic_tag << std::endl;
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

		//TODO mapObject.name is not the model name
		stationaryObject->set_model_reference(mapObject.name);
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

	auto OSITrafficSigns = staticMapTruth->mutable_traffic_sign();
	for (auto trafficSign : *trafficSigns) {
		carla::SharedPtr<carla::client::TrafficSign> carlaTrafficSign = boost::dynamic_pointer_cast<carla::client::TrafficSign>(trafficSign);
		auto OSITrafficSign = CarlaUtility::toOSI(carlaTrafficSign, xodr);
		OSITrafficSigns->AddAllocated(OSITrafficSign);
	}

	auto lanes = staticMapTruth->mutable_lane();
	auto laneboundarys = staticMapTruth->mutable_lane_boundary();
	auto topology = map->GetTopology();
	std::cout << "Map topology consists of " << topology.size() << " endpoint pairs" << std::endl;

	std::set<carla::road::JuncId> junctions;
	for (auto endpoints : topology) {
		////DEBUG
		//std::cout << "Current endpoint pair:" << std::endl <<
		//	"RoadId: " << endpoints.first->GetRoadId() << " LaneId: " << endpoints.first->GetLaneId() << " SectionId: " << endpoints.first->GetSectionId()
		//	<< std::endl <<
		//	"RoadId: " << endpoints.second->GetRoadId() << " LaneId: " << endpoints.second->GetLaneId() << " SectionId: " << endpoints.second->GetSectionId()
		//	<< std::endl;
		if (endpoints.first->IsJunction() && endpoints.second->IsJunction()) {
			auto junction = endpoints.first->GetJunction();

			auto id = junction->GetId();

			//TOCHECK simplification of data is ok
			//OSI Junction have a different defintion, listing the lanes connected to the junction, but not the paths through the junction

			if (junctions.count(id)) {
				//This junction has already been parsed
				continue;
			}

			junctions.insert(id);
			auto lane = lanes->Add();
			lane->set_allocated_id(CarlaUtility::toOSI(id, CarlaUtility::CarlaUniqueID_e::RoadIDLaneID));

			auto classification = lane->mutable_classification();
			classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION);

			for (auto path : junction->GetWaypoints()) {
				// OSI lane_pairing needs an antecessor/successor pair
				auto pair = classification->add_lane_pairing();
				auto inBound = path.first;
				auto outBound = path.second;
				pair->set_allocated_antecessor_lane_id(CarlaUtility::toOSI(inBound->GetRoadId(), inBound->GetLaneId()));
				pair->set_allocated_successor_lane_id(CarlaUtility::toOSI(outBound->GetRoadId(), outBound->GetLaneId()));
			}

		}
		else {
			// A lane that is not a junction. Waypoints of endpoint pair map to a OSI lane
			auto lane = lanes->Add();
			auto roadId = endpoints.first->GetRoadId();
			auto laneId = endpoints.first->GetLaneId();
			lane->set_allocated_id(CarlaUtility::toOSI(roadId, laneId, CarlaUtility::CarlaUniqueID_e::RoadIDLaneID));

			auto classification = lane->mutable_classification();

			if (carla::road::Lane::LaneType::Driving == endpoints.first->GetType()) {
				// centerline is only defined for lanes of type driving, except for junctions
				classification->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING);

				//get all waypoints until the lane's end, approximately 10cm apart. osi3::Lane::Classification expects its centerline have a deviation of at most 5cm when followed linearly
				//TOCHECK Optimization reduce number of waypoints - especially straight segments offer a more sparse representation
				auto waypoints = endpoints.first->GetNextUntilLaneEnd(0.1f);
				if (waypoints.size()) {
					classification->set_centerline_is_driving_direction(true);
				}
				else {
					// endpoints.first was not the start point (?)
					waypoints = endpoints.second->GetNextUntilLaneEnd(0.1f);
					//DEBUG
					std::cout << __FUNCTION__ << " DEBUG: Encountered a lane defined in reversed direction" << std::endl;
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

			auto leftLaneMarking = endpoints.first->GetLeftLaneMarking();
			if (leftLaneMarking) {
				auto leftLaneBoundary = CarlaUtility::parseLaneBoundary(leftLaneMarking.get());
				//connect laneboundary with lane via id
				classification->add_left_lane_boundary_id()->set_value(leftLaneBoundary->id().value());
				laneboundarys->AddAllocated(leftLaneBoundary.release());
			}

			auto rightLaneMarking = endpoints.first->GetRightLaneMarking();
			if (rightLaneMarking) {
				auto rightLaneBoundary = CarlaUtility::parseLaneBoundary(rightLaneMarking.get());
				//connect laneboundary with lane via id
				classification->add_right_lane_boundary_id()->set_value(rightLaneBoundary->id().value());
				laneboundarys->AddAllocated(rightLaneBoundary.release());
			}
		}
	}
	std::cout << "Finished parsing of topology" << std::endl;
}

std::shared_ptr<osi3::GroundTruth> CARLA2OSIInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use staticMapTruth as a base for every new groundTruth message that already contains unchanging fields
	std::shared_ptr<osi3::GroundTruth> groundTruth = std::make_shared<osi3::GroundTruth>();
	groundTruth->MergeFrom(*staticMapTruth);

	auto map = world->GetMap();
	auto worldActors = world->GetActors();
	for each (auto actor in *worldActors) {
		auto typeID = actor->GetTypeId();

		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth->add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<const carla::client::Vehicle>(actor);

			vehicle->set_allocated_id(CarlaUtility::toOSI(vehicleActor->GetId()));
			vehicle->set_type(osi3::MovingObject_Type_TYPE_VEHICLE);
			vehicle->set_allocated_base(CarlaUtility::toOSIBaseMoving(vehicleActor).release());

			auto classification = vehicle->mutable_vehicle_classification();
			classification->set_has_trailer(false);

			// Get closest waypoint to determine current lane
			auto waypoint = map->GetWaypoint(vehicleActor->GetLocation());
			//TODO vehicle might be on more than one lane
			auto laneIDs = vehicle->mutable_assigned_lane_id();
			if (waypoint->IsJunction()) {
				laneIDs->AddAllocated(CarlaUtility::toOSI(waypoint->GetJunctionId(), CarlaUtility::JuncID));
			}
			else {
				laneIDs->AddAllocated(CarlaUtility::toOSI(waypoint->GetRoadId(), waypoint->GetLaneId()));
			}

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
			}
			//TODO ground clearance

			// parse vehicle lights
			classification->set_allocated_light_state(CarlaUtility::toOSI(vehicleActor->GetLightState()).release());

			//





		}
		else if (typeID.rfind("walker.pedestrian", 0) == 0) {
			auto pedestrian = groundTruth->add_moving_object();
			auto walkerActor = boost::static_pointer_cast<const carla::client::Walker>(actor);

			pedestrian->set_allocated_id(CarlaUtility::toOSI(walkerActor->GetId()));
			pedestrian->set_type(osi3::MovingObject_Type_TYPE_PEDESTRIAN);
			pedestrian->set_allocated_base(CarlaUtility::toOSIBaseMoving(actor).release());

			//TODO How to determine a lane for pedestrians? Carla walkers don't care about lanes and walk on meshes with specific names (see https://carla.readthedocs.io/en/0.9.9/tuto_D_generate_pedestrian_navigation/):
			// Road_Sidewalk, Road_Crosswalk, Road_Grass, Road_Road, Road_Curb, Road_Gutter or Road_Marking 


		}
		else if ("traffic.traffic_light" == typeID) {
			carla::SharedPtr<const carla::client::TrafficLight> trafficLight = boost::dynamic_pointer_cast<carla::client::TrafficLight>(actor);
			//TODO parse carla::client::TrafficLight as a set of osi3::TrafficLight
			//a osi3::TrafficLight describes a single bulb of a traffic light

			auto bulbs = CarlaUtility::toOSI(trafficLight, xodr);
			//add converted bulbs to ground truth
			auto trafficLights = groundTruth->mutable_traffic_light();
			for (auto* bulb : bulbs) {
				trafficLights->AddAllocated(bulb);
			}
		}
	}

	return groundTruth;
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
	else {
		std::cerr << "CARLA2OSIInterface::sensorEventAction called for unsupported sensor type" << std::endl;
	}

	{//Mutex scope
		std::scoped_lock lock(actorRole2IDMap_mutex, varName2MessageMap_mutex);
		auto iter = actorRole2IDMap.right.find(sensor->GetId());
		if (iter != actorRole2IDMap.right.end()) {
			std::string varName = iter->second;
			varName2MessageMap[varName] = std::move(sensorView);
		}
		else {
			std::cerr << __FUNCTION__ << ": received event for unknown sensor with id " << sensor->GetId() << std::endl;
		}
	}
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
		actor->SetTargetVelocity(CarlaUtility::toCarla(&trafficUpdate.mutable_update()->mutable_base()->velocity()));
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
		actor->SetTargetAngularVelocity({ orientationRate.GetForwardVector().Length(), orientationRate.GetUpVector().Length(), orientationRate.GetRightVector().Length() });
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

	carla::SharedPtr<carla::client::Actor> actor;
	{// mutex scope
		// using shared lock - read only access
		std::shared_lock lock(actorRole2IDMap_mutex);
		//TODO how to safely retrieve the ego vehicle? 
		// ego vehicle in Carla usually is named 'hero'
		if (0 == actorRole2IDMap.left.count("hero")) {
			return -100;
		}
		auto actorId = actorRole2IDMap.left.at("hero");
		actor = world->GetActor(actorId);
	}
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
		actor->SetTargetVelocity(velocity);
	}

	//Acceleration can not be set in CARLA
	//GetAcceleration() calculates the acceleration with the actor's velocity
	//double acceleration = motionCommand.current_state().acceleration();
	//Curvature can not be set in CARLA since it is operated in synchronous mode
	//double curvature = motionCommand.current_state().curvature();

	//Trajectory
	//CARLA can not handle locations for future timestamps
	//TODO write TrajectoryHandler to handle vehicle trajectory in this client?
	//MotionCommand is not specified as a Sl4to5 OSMP Message

	return 0;
}
