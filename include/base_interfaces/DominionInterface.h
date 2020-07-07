#ifndef DOMINIONINTERFACE_H
#define DOMINIONINTERFACE_H

#include <string>
#include "base_interfaces/BaseSystemInterface.h"
#include "configreader/BaseConfigVariants.h"

class DominionInterface : public BaseSystemInterface
{
	virtual int readConfiguration(baseConfigVariants_t config) override;
	virtual int connect() override;
	virtual int disconnect() override;

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