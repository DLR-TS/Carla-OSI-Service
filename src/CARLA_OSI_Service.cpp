#include "CARLA_OSI_gRPC.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client\n" <<
		"Compiled for Carla Version 0.9.13\n" << std::endl;

	std::cout << "Current directory: " << fs::current_path() << "\n" << std::endl;
	
	//Server address deliberately chosen to accept any connection
	std::string serverAddress = "0.0.0.0:51425";

	CARLA_OSI_client client(serverAddress);
	client.StartServer();

	return 0;
}
