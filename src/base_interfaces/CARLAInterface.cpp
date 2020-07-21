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
	for each (auto actor in *world->GetActors())
	{
		activeActors.insert(actor->GetId());
		auto attributes = actor->GetAttributes();
		for each (auto attribute in attributes) {
			if ("role_name" == attribute.GetId() && !attribute.GetValue().empty()) {
				actorRole2IDMap.try_emplace(attribute.GetValue(), actor->GetId());
			}
		}
	}

	parseStationaryMapObjects();

	return 0;
}

double CARLAInterface::doStep() {
	if (!world) {
		throw std::exception("No world");
	}


	world->Tick(this->transactionTimeout);
	//world->WaitForTick(this->transactionTimeout);

	std::vector<carla::ActorId> removedActors;
	auto worldActors = world->GetActors();
	//std::sort(worldActors->begin(), worldActors->end());
	//std::set_difference(activeActors.begin(), activeActors.end(), worldActors->begin(), worldActors->end(), removedActors.begin());
	//std::sort(activeActors.begin(), activeActors.end());

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
	if (mapTruth) {
		mapTruth->Clear();
	}
	else {
		mapTruth = std::make_shared<osi3::GroundTruth>();
	}

	carla::SharedPtr<carla::client::Map> map = world->GetMap();
	//TODO parse map parts that won't change during simulation

	mapTruth->set_map_reference(map->GetName());


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




			//TODO should walkers be parsed as moving objects?
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
