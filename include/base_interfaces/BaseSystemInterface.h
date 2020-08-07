#ifndef BASESYSTEMINTERFACE_H
#define BASESYSTEMINTERFACE_H

#include "mapper/Mapper.h"
#include <string>

class BaseSystemInterface
{
public:
	virtual int getIntValue(std::string base_name) = 0;
	virtual bool getBoolValue(std::string base_name) = 0;
	virtual float getFloatValue(std::string base_name) = 0;
	virtual double getDoubleValue(std::string base_name) = 0;
	virtual std::string getStringValue(std::string base_name) = 0;

	virtual int setIntValue(std::string base_name, int value) = 0;
	virtual int setBoolValue(std::string base_name, bool value) = 0;
	virtual int setFloatValue(std::string base_name, float value) = 0;
	virtual int setDoubleValue(std::string base_name, double value) = 0;
	virtual int setStringValue(std::string base_name, std::string value) = 0;
};

#endif // !BASESYSTEMINTERFACE_H