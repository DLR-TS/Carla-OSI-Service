FROM ubuntu:20.04 as carla_osi_service_builder
MAINTAINER frank.baumgarten@dlr.de

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y cmake build-essential pip git libtbb2 libboost-filesystem-dev && rm -rf /var/lib/apt/lists/*
RUN pip install conan

RUN mkdir carlaosiservice && mkdir carlaosiservice/build
WORKDIR /carlaosiservice/build
COPY . /carlaosiservice/

RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . --target CARLA_OSI_Service -j 8

WORKDIR /
RUN mkdir logs
CMD /carlaosiservice/build/bin/CARLA_OSI_Service -log /logs/log.csv
