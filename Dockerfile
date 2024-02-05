FROM ubuntu:20.04 as carla_osi_service_builder
MAINTAINER frank.baumgarten@dlr.de

ENV DEBIAN_FRONTEND=noninteractive MAKEFLAGS="-j$(nproc)"
RUN apt-get update && apt-get install -y cmake build-essential pip git && rm -rf /var/lib/apt/lists/*
RUN pip install conan==1.59.0

RUN mkdir carlaosiservice && mkdir carlaosiservice/build
WORKDIR /carlaosiservice/build
COPY . /carlaosiservice/

RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . --target CARLA_OSI_Service -j 8

FROM ubuntu:20.04
COPY --from=carla_osi_service_builder /carlaosiservice/build/bin/CARLA_OSI_Service .
COPY --from=carla_osi_service_builder /root/.conan/data/boost/1.80.0/_/_/package/bac44d9899fc87bf2bb8aaf11c7c5655e9dc6bd8/lib/libboost_filesystem.so.1.80.0 /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.80.0
COPY --from=carla_osi_service_builder /root/.conan/data/tbb/2020.3/_/_/package/97172bab7554b947975f35cab343b2a755de9955/lib/libtbb.so.2 /usr/lib/x86_64-linux-gnu/libtbb.so.2

WORKDIR /
RUN mkdir logs
CMD /CARLA_OSI_Service -log /logs/log.csv
