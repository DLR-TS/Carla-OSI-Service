#include "simulation_interfaces/OSMPBridge.h"

int OSMPBridge::readOSMP(int hi, int lo, int size, eOSIMessage messageType) {
	address address;
	address.addr.base.hi = hi;
	address.addr.base.lo = lo;
	address.size = size;
	writeToInternalState(address, messageType);
	return 0;
}

int OSMPBridge::writeOSMP(int& hi, int& lo, int& size, eOSIMessage messageType) {
	address address;
	readFromInternalState(address, messageType);
	hi = address.addr.base.hi;
	lo = address.addr.base.lo;
	size = address.size;
	return 0;
}
