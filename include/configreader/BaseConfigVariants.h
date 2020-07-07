#ifndef BASECONFIGVARIANTS_H
#define BASECONFIGVARIANTS_H

#include <variant>
#include <string>
#include "configreader/StandardYAMLConfig.h"

typedef std::variant<CARLAInterfaceConfig> baseConfigVariants_t;

#endif BASECONFIGVARIANTS_H