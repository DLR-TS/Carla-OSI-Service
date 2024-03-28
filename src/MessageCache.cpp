#include "MessageCache.h"

void MessageCache::setMessage(const std::string& key, const std::string& message, bool verbose) {
	if (verbose)
	{
		std::cout << __FUNCTION__ << key << " Message size: " << message.size() << std::endl;
	}
	varName2MessageMap[key] = message;
}

std::string MessageCache::getMessage(const std::string& key, bool verbose) {
	auto iter = varName2MessageMap.find(key);
	if (iter != varName2MessageMap.end()) {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << " Message size: " << iter->second.size() << std::endl;
		}
		return iter->second;
	}
	else {
		if (verbose)
		{
			std::cout << __FUNCTION__ << key << "No message found." << std::endl;
		}
		return "";
	}
}
