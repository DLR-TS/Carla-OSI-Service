# CARLA OSI client for SETLevel4to5

A client for CARLA simulator. Translates World and sensor outputs into OSI messages

## Installation Guide

preparations:
install (conan.io)[conan.io]
add conan.exe to PATH environment variable
check out submodules (git submodule update --init --recursive) to get FMI4cpp or use GIT_SUBMODULE CMake option to do so automatically during build (enabled by default).

use cmake for project generation

# CMake FetchContent Overrides
Some dependencies are retrieved using CMake's FetchContent Module. To override their source in your local repository, create a file named 'CMake_FetchContent_Overrides.cmake' and use the FetchContent_declare() to declare your desired replacement. The will be included by the root CmakeLists.txt file. CMake_FetchContent_Overrides.cmake is ignored and thus will not be added to the global repository

## Used Libraries

carla 0.9.9.4
catch2
cmake-conan 0.15
open-simulation-interface

## Fix for current osi problems

Either don't build protobuffer as shared library or copy libprotobufd.dll and libprotocd.dll in cache folder from bin to lib\open-simulation-interface since the generation of pb.cc and pb.h files is not working right now.
ToDo: Fix this issue