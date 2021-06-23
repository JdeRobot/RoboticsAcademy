#!/usr/bin/env bash

docker build -f Dockerfile-foxy -t foxy-radi . && \
docker run -it \
      --rm \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --name foxy_radi_container \
      -v $PWD/../:/RoboticsAcademy \
      -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 \
      foxy-radi ./start.sh