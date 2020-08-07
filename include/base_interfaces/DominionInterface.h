#ifndef DOMINIONINTERFACE_H
#define DOMINIONINTERFACE_H

#include <string>
#include "base_interfaces/BaseSystemInterface.h"

class DominionInterface : BaseSystemInterface
{
	virtual int getIntValue(std::string base_name) override;
	virtual bool getBoolValue(std::string base_name) override;
	virtual float getFloatValue(std::string base_name) override;
	virtual double getDoubleValue(std::string base_name) override;
	virtual std::string getStringValue(std::string base_name) override;

	virtual int setIntValue(std::string base_name, int value) override;
	virtual int setBoolValue(std::string base_name, bool value) override;
	virtual int setFloatValue(std::string base_name, float value) override;
	virtual int setDoubleValue(std::string base_name, double value) override;
	virtual int setStringValue(std::string base_name, std::string value) override;
};

#endif // !DOMINIONINTERFACE_H