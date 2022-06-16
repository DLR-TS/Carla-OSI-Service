# CARLA OSI Service

A client service for CARLA simulator.\
Translates world into Ground Truth OSI message and updates traffic participants by TrafficUpdate OSI Message.\
Synchronizes with Carla Scenario Runner (SETLevel version).\
Is used by CoSiMa as a base simulator.\
Many different configurations are available through runtime parameters. Use -h or --help for details.

# Installation Guide

preparations:
install (conan.io)[conan.io]

add conan.exe to PATH environment variable

## Linux

in root folder:
```sh
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release
 cmake --build . --target CARLA_OSI_Service
```

## Docker

Create personal access token (PAT) for gitlab.setlevel.de.
Create .TOKEN file in project root.

Paste PAT in file: \<username\>:\<accesstoken\>

```sh
 docker build -t setlevel:carlaosiservice .
```

## Windows with MSVC 2017

Open the folder in Visual Studio and use the cmake integration.

## CMake FetchContent Overrides

Some dependencies are retrieved using CMake's FetchContent Module.
To override their source in your local repository, create a file named 'CMake_FetchContent_Overrides.cmake' and use the FetchContent_declare() to declare your desired replacement.
They will be included by the root CmakeLists.txt file.
CMake_FetchContent_Overrides.cmake is ignored and thus will not be added to the global repository.

# Used Libraries

carla 0.9.10\
catch2\
cmake-conan 0.15\
open-simulation-interface

## Fix for Waypoint.cpp error

Change Lines in libarla Wapoint.cpp
16: size_t seed = 0u;
21: return (WaypointHash::result_type) seed;
