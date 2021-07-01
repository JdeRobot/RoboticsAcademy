#!/bin/bash

docker build -f Dockerfile-foxy-minimal -t foxy-minimal-radi . && \
docker run -it \
      --rm \
      --net=host \
      --privileged \
      --gpus all \
      -e DISPLAY \
      -e XAUTHORITY=/tmp/.Xauthority \
      -v ${XAUTHORITY}:/tmp/.Xauthority \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --name foxy_radi_container \
      foxy-minimal-radi bash