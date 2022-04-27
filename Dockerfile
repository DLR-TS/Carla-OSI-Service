FROM ubuntu
MAINTAINER frank.baumgarten@dlr.de

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y libtbb2 && rm -rf /var/lib/apt/lists/*

RUN mkdir setlevel
WORKDIR /setlevel
RUN mkdir logs

COPY CARLA_OSI_Service /setlevel/
CMD ./CARLA_OSI_Service -d -sr -log log.csv
