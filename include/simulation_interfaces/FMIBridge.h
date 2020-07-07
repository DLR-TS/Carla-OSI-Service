#ifndef FMIBRIDGE_H
#define FMIBRIDGE_H

#include "iSimulationData.h"
#include "../Mapper/Mapper.h"
#include "../Mapper/FMIMapper.h"
//windef.h compatibility
#define NOMINMAX
#include "fmi4cpp/fmi4cpp.hpp"
//sleep to reduce poll interval for async FMUs
#include <thread>
#include <chrono>


class FMIBridge : public iSimulationData
{
	friend class FMIMapper;
protected:
	std::unique_ptr<fmi4cpp::fmi2::cs_fmu> coSimFMU;
	std::shared_ptr<fmi4cpp::fmi2::cs_slave> coSimSlave;


	class FMUSlaveStateWrapper {
	private:
		//cs_slave creating this fmu state is needed later for freeing the memory again
		std::shared_ptr<fmi4cpp::fmi2::cs_slave> coSimSlave;

		FMUSlaveStateWrapper(std::shared_ptr < fmi4cpp::fmi2::cs_slave> slave);

	public:
		~FMUSlaveStateWrapper();

		fmi4cpp::fmi4cppFMUstate state;
		static std::optional<FMUSlaveStateWrapper> tryGetStateOf(std::shared_ptr<fmi4cpp::fmi2::cs_slave> slave);
	};

public:
	FMIBridge(std::shared_ptr<Mapper> mapper) : iSimulationData(mapper){};

	int init(std::string scenario, float starttime, int mode) override;
	int connect(std::string info) override;
	int disconnect() override;

	int writeToInternalState() override;
	int readFromInternalState() override;
	int doStep(double stepSize = 1) override;

};

#endif // !FMIBRIDGE_H