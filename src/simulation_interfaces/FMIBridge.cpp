#include <string>
#include "simulation_interfaces/FMIBridge.h"

int FMIBridge::init(std::string scenario, float starttime, int mode) {

	//TODO cannot set up the stepFinished callback in fmi2CallbackFunctions because it is not available in FMI4cpp => have to fallback to polling the fmu preStepState
	////TODO move this block. While setting up fmi2Callbackfunctions is required before instanciating the model, it is not required to this every time
	//if (coSimFMU->get_model_description()->can_run_asynchronuously) {
	//	//TODO set up stepFinished callback
	//}

	//Instance name cannot be set with FMU4cpp. The model identifier is used automatically instead
	coSimSlave = coSimFMU->new_instance();

	//TODO where to get the stop time and tolerance (both optional)?
	coSimSlave->setup_experiment((fmi2Real)starttime);
	//TODO set variables with initial==exact or initial==approx
	coSimSlave->enter_initialization_mode();
	//TODO set independent and continuous-time variables with initial==exact
	//TODO set continuous- and discrete-time inputs and optionally also the derivatives of the former
	//TODO get (calculated or dependent) values and derivatives, if needed
	coSimSlave->exit_initialization_mode();
	return 0;
}

int FMIBridge::connect(std::string info) {
	return 0;
}

int FMIBridge::disconnect() {
	return 0;
}

int FMIBridge::writeToInternalState() {

	auto const model_description = coSimFMU->get_model_description();
	//iterate over unknowns declared as output
	for (auto const& unknown : model_description->model_structure->outputs) {
		// use index to translate unknown into scalar_variable. FMU ScalarVariable index begins at 1
		auto const& outputVar = (*model_description->model_variables.get())[unknown.index - 1];

		// cannot switch on outputVar.value_type because fmi4cpp uses strings and 'real' has no match in Mapper::getType
		// Map output value into internal preStepState
		if (outputVar.is_boolean()) {
			fmi2Boolean boolean;
			coSimSlave->read_boolean(outputVar.value_reference, boolean);
			mapper->mapToInternalState((bool)boolean, outputVar.name, eDataType::BOOLCOSIMA);
		}
		else if (outputVar.is_enumeration() || outputVar.is_integer()) {
			fmi2Integer integer;
			coSimSlave->read_integer(outputVar.value_reference, integer);
			mapper->mapToInternalState(integer, outputVar.name, eDataType::INTEGERCOSIMA);
		}
		else if (outputVar.is_real()) {
			fmi2Real real;
			coSimSlave->read_real(outputVar.value_reference, real);
			const values_t value = real;
			//TODO skip this test and always use double? default fmi2TypesPlatform.h uses double
			if (nullptr != std::get_if<double>(&value)) {
				mapper->mapToInternalState(real, outputVar.name, eDataType::DOUBLECOSIMA);
			}
			else {
				mapper->mapToInternalState(real, outputVar.name, eDataType::FLOATCOSIMA);
			}
		}
		else {
			fmi2String string;
			coSimSlave->read_string(outputVar.value_reference, string);
			mapper->mapToInternalState(string, outputVar.name, eDataType::STRINGCOSIMA);
		}
		//}
	}
	return 0;
}

int FMIBridge::doStep(double stepSize) {
	//TODO set independent tunable parameters
	//TODO set continuous- and discrete-time inputs and optionally also the derivatives of the former

	//TODO support rollback in case step is incomplete?
	auto preStepState = FMUSlaveStateWrapper::tryGetStateOf(coSimSlave);

	//TODO step by stepSize
	if (!coSimSlave->step(stepSize)) {
		while (fmi4cpp::status::Pending == coSimSlave->last_status()) {
			//wait for asynchronous fmu to finish
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}

		switch (coSimSlave->last_status()) {
		case fmi4cpp::status::Fatal:
			return -1;//TODO decide on common error return values
		case fmi4cpp::status::Error:
			return -2;//TODO decide on common error return values
		case fmi4cpp::status::Discard:
			//If a pre step state could be captured and the slave supports step size variation, try performing smaller substeps instead of one stepSize step
			if (!preStepState || !coSimSlave->get_model_description()->can_handle_variable_communication_step_size) {
				return 2;
			}
			//restore state before failed step
			coSimSlave->set_fmu_state(preStepState.value().state);
			// perform some smaller substeps
			int substeps = 2;
			for (int i = 0; i < substeps; i++) {
				int err = doStep(stepSize / 2);
				if (0 != err) {
					return 2;
				}
			}
		}
	}

	return 0;
}

int FMIBridge::readFromInternalState() {
	auto const modelDescription = coSimSlave->get_model_description();
	// The following would have to loop twice, because it cannot filter for multiple causalities at once
	//std::vector<fmi4cpp::fmi2::scalar_variable> inputs;
	//modelDescription->model_variables->getByCausality(fmi4cpp::fmi2::causality::input, inputs);
	//modelDescription->model_variables->getByCausality(fmi4cpp::fmi2::causality::parameter, inputs);
	// Explicit loop to filter is faster:
	for (auto const& variable : *(modelDescription->model_variables)) {

		if (fmi4cpp::fmi2::causality::input == variable.causality
			|| fmi4cpp::fmi2::causality::parameter == variable.causality) {

			if (variable.is_integer())
			{
				const values_t value = mapper->mapFromInternalState(variable.name, eDataType::INTEGERCOSIMA);
				const fmi2Integer integer = std::get<fmi2Integer>(value);
				coSimSlave->write_integer(variable.value_reference, integer);
			}
			//FMI specifies only 'Real'. No discrimination of precision
			else if (variable.is_real())
			{
				const values_t value = mapper->mapFromInternalState(variable.name,
					typeid(fmi2Real) == typeid(double) ? eDataType::DOUBLECOSIMA : eDataType::FLOATCOSIMA);
				const fmi2Real real = std::get<fmi2Real>(value);
				coSimSlave->write_real(variable.value_reference, real);
			}
			else if (variable.is_boolean())
			{
				const values_t value = mapper->mapFromInternalState(variable.name, eDataType::BOOLCOSIMA);
				const fmi2Boolean boolean = std::get<bool>(value);
				coSimSlave->write_boolean(variable.value_reference, boolean);
			}
			else if (variable.is_string())
			{
				const values_t value = mapper->mapFromInternalState(variable.name, eDataType::STRINGCOSIMA);
				const fmi2String str = std::get<std::string>(value).c_str();
				coSimSlave->write_string(variable.value_reference, str);
			}
			else {
				//this would indicate and error in FMI4cpp...
				std::cerr << "FMIBridge::mapTo encountered unsupported type " << variable.type_name() << std::endl;
				return -1;
			}
		}
	}
	return 0;
}

inline FMIBridge::FMUSlaveStateWrapper::FMUSlaveStateWrapper(std::shared_ptr<fmi4cpp::fmi2::cs_slave> slave) {
	slave->get_fmu_state(state);
	coSimSlave = slave;
}

inline FMIBridge::FMUSlaveStateWrapper::~FMUSlaveStateWrapper() {
	coSimSlave->free_fmu_state(state);
}

std::optional<FMIBridge::FMUSlaveStateWrapper> FMIBridge::FMUSlaveStateWrapper::tryGetStateOf(std::shared_ptr<fmi4cpp::fmi2::cs_slave> slave) {
	if (slave->get_model_description()->can_get_and_set_fmu_state) {
		return FMIBridge::FMUSlaveStateWrapper(slave);
	}
	return std::nullopt;
}