# CoSimulationManager for SETLevel4to5

## Installation Guide

preparations:
install (conan.io)[conan.io]
add conan.exe to PATH environment variable
check out submodules (git submodule update --init --recursive) to get FMI4cpp or use GIT_SUBMODULE CMake option to do so automatically during build
	- FMI4cpp is not available as conan package (might change in the future?)
	- its dependencies are installed using conan, invoked from cmake when building CoSimulationManagerLib

in root folder:
mkdir build && cd build
conan install ..


use cmake for project generation


## Used Libraries

cmake-conan 0.15