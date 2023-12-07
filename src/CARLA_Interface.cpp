#include "CARLA_Interface.h"

int CARLAInterface::initialise(RuntimeParameter& runtimeParams) {
	this->runtimeParameter = runtimeParams;

	try {
		//connect
		this->client = std::make_unique<carla::client::Client>(runtimeParams.carlaHost, runtimeParams.carlaPort);
		this->client->SetTimeout(std::chrono::duration<double>(runtimeParams.transactionTimeout));

		loadWorld();
		applyWorldSettings();
        //TODO December
		//parseStationaryMapObjects();

		//if (runtimeParameter.replay.enabled) {
		//	fillBoundingBoxLookupTable();
		//}
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    //TODO December
	// perform a tick to fill actor and message lists
	//doStep();
	return 0;
}

void CARLAInterface::loadWorld() {
	this->world = std::make_unique<carla::client::World>(std::move(this->client->GetWorld()));
	this->map = world->GetMap();
}

void CARLAInterface::applyWorldSettings() {
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

void CARLAInterface::resetWorldSettings() {
	auto settings = world->GetSettings();
	settings.synchronous_mode = false;
	this->world->ApplySettings(settings, settingsDuration);
	if (runtimeParameter.verbose) {
		std::cout << "Reset CARLA World Settings." << std::endl;
	}
}
