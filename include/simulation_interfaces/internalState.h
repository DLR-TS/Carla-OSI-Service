#ifndef INTERNALSTATE_H
#define INTERNALSTATE_H

#include <vector>
#include <string>

/**
Holds the state of an interface.
\var integers Vector of integer values of interface.
\var floats Vector of float values of interface.
\var doubles Vector of double values of interface.
\var bools Vector of bool values of interface.
\var strings Vector of string values of interface.
*/
class internalState {
public:
	std::vector<int> integers;
	std::vector<float> floats;
	std::vector<double> doubles;
	std::vector<bool> bools;
	std::vector<std::string> strings;
};

#endif // !INTERNALSTATE_H
