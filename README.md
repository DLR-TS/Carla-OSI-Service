# CARLA OSI Service

CARLA OSI Sercive is a client service for [CARLA](https://github.com/carla-simulator/carla).
It communicates with [ASAM OSI](https://www.asam.net/standards/detail/osi/) messages:

 - Ground Truth 
 - Sensor View for generic sensors and [CARLA sensors](https://carla.readthedocs.io/en/0.9.13/ref_sensors/)
 - Sensor View Configuration
 - Traffic Command
 - Traffic Update

Carla OSI Service is used as a base simulator in [CoSiMa](https://github.com/DLR-TS/CoSiMa) and can synchronize with Carla and Carla Scenario Runner with OSI controller [OSTAR Version](https://github.com/DLR-TS/scenario_runner/tree/ostar).

Many different configurations are available through runtime parameters.
The complete list of inputs and outputs can be found [here](https://github.com/DLR-TS/Carla-OSI-Service/blob/master/Configuration.md).

# Installation Guide

## Linux

Install [conan](https://conan.io/) 1.x. Conan 2 not yet supported!

in root folder:
```sh
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release
 cmake --build . --target CARLA_OSI_Service
```

## Docker

```sh
 docker build -t setlevel:carlaosiservice .
```

## Windows with MSVC 2017

Install [conan 1.x](https://conan.io/) \
Add conan.exe to PATH environment variable \
Open the folder in Visual Studio and use the cmake integration.

## CMake FetchContent Overrides

Some dependencies are retrieved using CMake's FetchContent Module.
To override their source in your local repository, create a file named 'CMake_FetchContent_Overrides.cmake' and use the FetchContent_declare() to declare your desired replacement. \
They will be included by the root CMakeLists.txt file. \
CMake_FetchContent_Overrides.cmake is ignored and thus will not be added to the global repository.

# Contacts

bjoern.bahn@dlr.de

This software was originally developed as part of [SetLevel](https://setlevel.de/).
