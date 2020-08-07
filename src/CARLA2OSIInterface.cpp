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

	//TODO inform Carla of added/removed actors
	//TODO inform Carla of traffic updates

	world->Tick(this->transactionTimeout);
	//world->WaitForTick(this->transactionTimeout);

	// track actors added/removed by Carla
	addedActors.clear();
	removedActors.clear();
	worldActorIDs.clear();
	worldActors = world->GetActors();
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
	std::string mergedMessage;
	if (actorRole2IDMap.left.count(base_name)) {
		auto actor = world->GetActor(actorRole2IDMap.left.at(base_name));

		//TODO update Carla actor and store merged serialized OSI message in varName2MessageMap
	}
	else {
		varName2MessageMap[base_name] = value;
	}


	return 0;
}
std::string_view CARLA2OSIInterface::getPrefix(std::string_view name)
{
	if (2 < name.size() && '#' == name[0]) {
		std::string_view prefix = std::string_view(&name.at(1), name.find('#', 1));
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
			//TODO parse vehicle as moving object




			//TODO should walkers be parsed as moving objects? They are not StationaryObject
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
