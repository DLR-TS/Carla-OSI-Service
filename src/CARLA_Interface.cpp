#include "CARLA_Interface.h"

int CARLAInterface::initialise(std::shared_ptr<RuntimeParameter> runtimeParams) {
	this->runtimeParameter = runtimeParams;

	try {
		//connect
		this->client = std::make_unique<carla::client::Client>(runtimeParams->carlaHost, runtimeParams->carlaPort);
		this->client->SetTimeout(std::chrono::duration<double>(runtimeParams->transactionTimeout));

		loadWorld();
		applyWorldSettings();
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
	return 0;
}

void CARLAInterface::loadWorld() {
	this->world = std::make_unique<carla::client::World>(std::move(this->client->GetWorld()));
	this->map = world->GetMap();
}

void CARLAInterface::applyWorldSettings() {
	if (runtimeParameter->scenarioRunner.doesTick) {
		std::cout << "No settings applied because scenario runner should be active." << std::endl;
		runtimeParameter->deltaSeconds = (float)world->GetSettings().fixed_delta_seconds.value();
		return;
	}
	auto settings = world->GetSettings();
	//set sync or async operational mode
	settings.synchronous_mode = runtimeParameter->sync;

	if (settings.fixed_delta_seconds.has_value() &&
		settings.fixed_delta_seconds.value() == runtimeParameter->deltaSeconds &&
		settings.synchronous_mode) {
		if (runtimeParameter->verbose) {
			std::cout << "Settings of Carla Server are already correct and do not need to be changed" << std::endl;
		}
		return;
	}
	settings.fixed_delta_seconds = runtimeParameter->deltaSeconds;
	settings.synchronous_mode = true;
	this->world->ApplySettings(settings, settingsDuration);
}

void CARLAInterface::resetWorldSettings() {
	if (runtimeParameter->scenarioRunner.doesTick) {
		std::cout << "No settings resetted because scenario runner should be active." << std::endl;
		return;
	}
	auto settings = world->GetSettings();
	settings.synchronous_mode = false;
	this->world->ApplySettings(settings, settingsDuration);
	if (runtimeParameter->verbose) {
		std::cout << "Reset CARLA World Settings." << std::endl;
	}
}

double CARLAInterface::doStep() {
	if (runtimeParameter->verbose) {
		std::cout << "Do Step" << std::endl;
	}
	if (!world) {
		std::cerr << "No world" << std::endl;
		throw std::exception();
	}
	//tick not needed if in asynchronous mode
	if (runtimeParameter->sync) {
		//Length of simulationed tick is set in applyWorldSettings()
		world->Tick(client->GetTimeout());
	}
	//carla->world->WaitForTick(this->transactionTimeout);

	// only accurate if using fixed time step, as activated during initialise()
	return world->GetSnapshot().GetTimestamp().delta_seconds;
}
