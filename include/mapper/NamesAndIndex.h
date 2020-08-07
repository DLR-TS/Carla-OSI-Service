#ifndef NAMESANDINDEX_H
#define NAMESANDINDEX_H

/**
* Contains information about variable interface name and its position in vector.
* \var std::string baseName
* holds the base name of variable
* \var std::string interfaceName
* holds the interface name of variable
* \var int index
* holds the index where the variable with the name is located
*/
class NamesAndIndex {
public:
	NamesAndIndex(std::string baseName, std::string interfaceName, int index) : baseName(baseName), interfaceName(interfaceName), index(index) {};
	const std::string baseName;
	const std::string interfaceName;
	const int index;
};

#endif // !NAMESANDINDEX_H
