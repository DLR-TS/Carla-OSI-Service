#ifndef COSIMA_H
#define COSIMA_H

#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include "CARLA2OSIInterface.h"
#include "grpc_proto_files/base_interface/BaseInterface.grpc.bp.h"
#include "grpc_proto_files/base_interface/BaseInterface.pb.h"

void mainLoop();

#endif // !COSIMA_H
