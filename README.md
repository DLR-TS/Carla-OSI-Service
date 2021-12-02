# CARLA OSI Service for SETLevel4to5

A client service for CARLA simulator. Translates World and sensor outputs into OSI messages

# manual build (on Linux)
in root folder:
```sh
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release
 cmake --build . --target CARLA_OSI_Service 
 cmake --install . # not yet defined
```

## Installation Guide

preparations:
install (conan.io)[conan.io]
add conan.exe to PATH environment variable

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

##Fix for Waypoint.cpp error

Change Lines in libarla Wapoint.cpp
16: size_t seed = 0u;
21: return (WaypointHash::result_type) seed;
