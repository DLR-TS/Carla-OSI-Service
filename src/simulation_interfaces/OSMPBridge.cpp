#include "simulation_interfaces/OSMPBridge.h"

int OSMPBridge::readOSMP(int hi, int lo, int size, eOSIMessage messageType) {
	address address;
	address.addr.base.hi = hi;
	address.addr.base.lo = lo;
	address.size = size;
	writeToInternalState(address, messageType);
	return 0;
}
