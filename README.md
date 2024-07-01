# OSTAR CARLA-OSI-Service

CARLA-OSI-Sercive is a part of [OSTAR](https://github.com/DLR-TS/OSTAR-Quickstart).
It connects OSTAR with [CARLA](https://github.com/carla-simulator/carla) and communicates via [ASAM OSI](https://www.asam.net/standards/detail/osi/) messages.

# Installation Guide

## Linux

```sh
 # Preparation
 pip install conan==1.59.0
 # Checkout
 git clone https://github.com/DLR-TS/Carla-OSI-Service.git
 cd Carla-OSI-Service
 # Build
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release
 cmake --build . --target CARLA_OSI_Service
 # Run
 cd bin
 ./CARLA_OSI_Service
```

## Docker

```sh
 git clone https://github.com/DLR-TS/Carla-OSI-Service.git
 cd Carla-OSI-Service
 docker build -t setlevel:carlaosiservice .
```

# Run

| Imporant runtime parameter | Description |
| ------ | ------ |
| \<port\> | open port |
| \<ip\>:\<port\> | open port and allow only connections from ip |

Many configurations are available through CoSiMa Configuration.
The complete list can be found [here](https://github.com/DLR-TS/Carla-OSI-Service/blob/master/Configuration.md).

# Windows

Install [conan 1.x](https://conan.io/) \
Add conan.exe to PATH environment variable \
Open the directory in Visual Studio and use the cmake integration.

# Further information

Check out the OSTAR documentation at [OSTAR Quickstart](https://github.com/DLR-TS/OSTAR-Quickstart/tree/main/docu).


# Contacts

bjoern.bahn@dlr.de frank.baumgarten@dlr.de opensource-ts@dlr.de

This software was originally developed as part of [SetLevel](https://setlevel.de/).
