FROM ubuntu as carla_osi_service_builder
MAINTAINER frank.baumgarten@dlr.de

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y cmake build-essential pip git libtbb2 #&& rm -rf /var/lib/apt/lists/*
RUN pip install conan

RUN mkdir carlaosiservice && mkdir carlaosiservice/build
WORKDIR /carlaosiservice/build
COPY . /carlaosiservice/

RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN rm /carlaosiservice/.TOKEN
RUN cmake --build . --target CARLA_OSI_Service -j 8

FROM ubuntu
COPY --from=carla_osi_service_builder /carlaosiservice/build/bin/CARLA_OSI_Service .
RUN mkdir logs
CMD ./CARLA_OSI_Service -d -sr -log /logs/log.csv
