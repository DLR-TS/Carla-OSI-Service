#include "CARLA-OSI.h"

int main(int argc, char *argv[])
{
	std::cout << "Welcome to CARLA-OSI client" << std::endl << std::endl;

	std::cout << std::filesystem::current_path() << std::endl << std::endl;



	mainLoop();

	return 0;
}

void mainLoop() {
	//TODO wait for CoSiMa message, handle message, reply
}