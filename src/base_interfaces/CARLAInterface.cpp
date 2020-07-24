#include "base_interfaces\CARLAInterface.h"

int CARLAInterface::readConfiguration(baseConfigVariants_t variant) {
	CARLAInterfaceConfig* config = std::get_if<CARLAInterfaceConfig>(&variant);
	if (nullptr == config) {
		std::cerr << "Called with wrong configuration variant!" << std::endl;
		return 1;
	}

	this->host = config->host;
	this->port = config->port;
	this->transactionTimeout = std::chrono::duration<double>(config->transactionTimeout);
	this->deltaSeconds = config->deltaSeconds;

	return 0;
}

int CARLAInterface::initialise() {
	//connect
	this->client = std::make_unique<carla::client::Client>(host, port);
	this->client->SetTimeout(transactionTimeout);
	this->world = std::make_unique<carla::client::World>(std::move(client->GetWorld()));

	//assure server is in synchronous mode
	auto settings = world->GetSettings();
	settings.fixed_delta_seconds = this->deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings);

	//get all actors and their role (if set)
	auto actors = world->GetActors();
	std::cout << "Actor count at init: " << actors->size() << std::endl;
	for each (auto actor in *actors)
	{
		activeActors.insert(actor->GetId());
		auto attributes = actor->GetAttributes();
		for each (auto attribute in attributes) {
			if ("role_name" == attribute.GetId() && !attribute.GetValue().empty()) {
				actorRole2IDMap.try_emplace(attribute.GetValue(), actor->GetId());
			}
		}

		std::cout << actor->GetDisplayId() << std::endl;
	}

	parseStationaryMapObjects();

	return 0;
}

double CARLAInterface::doStep() {
	if (!world) {
		throw std::exception("No world");
	}

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

	world->Tick(this->transactionTimeout);
	//world->WaitForTick(this->transactionTimeout);

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

	return this->deltaSeconds;
}

int CARLAInterface::getIntValue(std::string base_name) {
	return 0;
};

bool CARLAInterface::getBoolValue(std::string base_name) {
	return true;
};

float CARLAInterface::getFloatValue(std::string base_name) {
	return 0.0;
};

double CARLAInterface::getDoubleValue(std::string base_name) {
	return 0.0;
};

std::string CARLAInterface::getStringValue(std::string base_name) {




	return "";
};

int CARLAInterface::setIntValue(std::string base_name, int value) {
	return 0;
};

int CARLAInterface::setBoolValue(std::string base_name, bool value) {
	return 0;
};

int CARLAInterface::setFloatValue(std::string base_name, float value) {
	return 0;
};

int CARLAInterface::setDoubleValue(std::string base_name, double value) {
	return 0;
};

int CARLAInterface::setStringValue(std::string base_name, std::string value) {
	return 0;
}
std::string_view CARLAInterface::getPrefix(std::string_view name)
{
	if (2 < name.size() && '#' == name[0]) {
		std::string_view prefix = std::string_view(&name.at(1), name.find('#', 1));
		return prefix;
	}
	return std::string_view();
}


osi3::Timestamp CARLAInterface::parseTimestamp()
{
	osi3::Timestamp osiTime;
	auto carlaTime = world->GetSnapshot().GetTimestamp();
	double intPart;
	double fractional = std::modf(carlaTime.elapsed_seconds, &intPart);
	osiTime.set_seconds(google::protobuf::int64(intPart));
	osiTime.set_nanos(google::protobuf::uint32(fractional *1e9));
	return osiTime;
}


void CARLAInterface::parseStationaryMapObjects()
{
	mapTruth = std::make_shared<osi3::GroundTruth>();

	carla::SharedPtr<carla::client::Map> map = world->GetMap();

	mapTruth->set_map_reference(map->GetName());

	auto stationaryObjects = mapTruth->mutable_stationary_object();
	//TODO parse map parts that won't change during simulation

	// Static props apparently aren't part of the actor list, so this list is empty
	auto staticProps = world->GetActors()->Filter("static.prop.*");
	for each(auto prop in *staticProps) {
		// parse as StationaryObject
		stationaryObjects->AddAllocated(&CarlaUtility::toOSIStationaryObject(prop));

		//DEBUG
		std::cout << "Got an Actor of type 'static.prop.*'" << prop->GetDisplayId() << std::endl;
	}

	//DEBUG
	auto landmarks = map->GetAllLandmarks();
	std::cout << "Landmarks: " << landmarks.size() << std::endl;
	for each(auto landmark in landmarks) {
		std::cout << landmark->GetName() << " " << landmark->GetId() << " " << landmark->GetRoadId() << " " << landmark->GetType() << std::endl;
	}

	//DEBUG
	auto traffic = world->GetActors()->Filter("traffic.*");
	std::cout << "traffic Actors: " << traffic->size() << std::endl;
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

	std::cout << "traffic signs: " << trafficSigns->size() << std::endl;
	for each(auto trafficSign in *trafficSigns) {
		std::cout << trafficSign->GetDisplayId() << std::endl;
		auto attributes = trafficSign->GetAttributes();
		std::cout << " attributes: " << attributes.size() << std::endl;
		for each (auto attribute in attributes) {
			std::cout << "  " << attribute.GetId() << ": " << attribute.GetValue() << std::endl;
		}

	}

	//TODO might be able to get lanes using map->GetTopology() and next_until_lane_end and previous_until_lane_start

}

osi3::GroundTruth CARLAInterface::parseWorldToGroundTruth()
{

	// lanes and lane boundaries are part of the map, which shouldn't change during simulation and can be preparsed during init
	// use mapTruth as a base for every new groundTruth message that already contains unchanging fields
	osi3::GroundTruth groundTruth(*mapTruth);
	//groundTruth.MergeFrom(mapTruth);

	for each (auto actor in *world->GetActors()) {
		auto typeID = actor->GetTypeId();

		//based on blueprint vehicle.*
		if (typeID.rfind("vehicle", 0) == 0) {
			auto vehicle = groundTruth.add_moving_object();
			auto vehicleActor = boost::static_pointer_cast<carla::client::Vehicle>(actor);
			//TODO parse vehicle as moving object




			//TODO should walkers be parsed as moving objects? They are not StationaryObject
		}
	}



	return groundTruth;
}

std::vector<osi3::SensorView> CARLAInterface::parseSensorActors()
{
	std::vector<osi3::SensorView> sensorViews;
	for each (auto actor in *world->GetActors()) {
		auto typeID = actor->GetTypeId();



		//based on blueprint sensor.*
		if (0 == typeID.rfind("sensor", 0))
		{
			osi3::SensorView sensorView;
			auto sensor = boost::static_pointer_cast<carla::client::Sensor>(actor);

			//substring of typeID
			std::string_view sensorType(&typeID[7]);

			if (0 == sensorType.rfind("camera.rgb", 0))
			{

			}
			else if (0 == sensorType.rfind("lidar.ray_cast", 0))
			{

			}
			else if (0 == sensorType.rfind("other.radar", 0))
			{

			}




			//TODO parse sensor view depending on typeID
		}
	}
	return sensorViews;
}
