#ifndef ISIMULATIONDATA_H
#define ISIMULATIONDATA_H

#include <vector>
#include <string>
#include <memory>
#include "base_interfaces/BaseSystemInterface.h"
#include "mapper/Mapper.h"
#include "internalState.h"

//forward declarations
/**
* Basic value types
*/
typedef std::variant<int, float, double, bool, std::string> values_t;
enum eDataType;
class BaseSystemInterface;
class Mapper;

/**
* Enum containing all supported interfaces and error for parsing failures.
*/
enum eSimulatorName
{
	FMI, //needs to be first
	VTD,
	ROS,
	UNREAL,
	OSI,
	SUMO,

	SIMULATORNAME_ERROR //needs to be last
};

/**
Abstract class for all simulation interfaces.
*/
class iSimulationData
{
public:
	/**
	Constructor of iSimulationData.
	\param mapper Mapper to be set.
	*/
	iSimulationData(std::shared_ptr<Mapper> mapper)
	{
		this->mapper = mapper;

	}

protected:
	/**
	Holds a copy of the simulator interface variables.
	*/
	std::shared_ptr<internalState> state;
	/**
	Specific mapper of this interface.
	*/
	std::shared_ptr<Mapper> mapper;

public:
	/**
	Initialize the interface.
	\param scenario Scenario identification.
	\param starttime Time of start.
	\param mode The mode to start in.
	\return Success status.
	*/
	virtual int init(std::string scenario, float starttime, int mode) = 0;
	/**
	Connect with followed information.
	\param info Information to connect with simulator.
	\return Success status.
	*/
	virtual int connect(std::string info) = 0;
	/**
	Disconnect from interface.
	\return Success status.
	*/
	virtual int disconnect() = 0;
	/**
	Search and map needed information of this interface from all other interfaces.
	\param baseInterface base interface
	\return Success status.
	*/
	int mapToInterfaceSystem(std::shared_ptr<BaseSystemInterface> baseInterface);
	/**
	Write output data of interface to base system
	\return Success status.
	*/
	int mapFromInterfaceSystem(std::shared_ptr<BaseSystemInterface> baseInterface);
	/**
	Do simulation step.
	\return Success status.
	*/
	virtual int doStep(double stepSize = 1) = 0;
	/**
	update outputs of the interface in the internal state
	uses the Mapper::mapToInternalState method to write outputs
	*/
	virtual int writeToInternalState() = 0;
	/**
	\return Mapper of this interface.
	*/
	std::shared_ptr<Mapper> getMapper();

	/**
	Reads the internal state into the simulation interface.
	*/
	virtual int readFromInternalState() = 0;

};
#endif // !ISIMULATIONDATA_H