#!/usr/bin/env bash

docker build -f Dockerfile-foxy.base -t jderobot/base:4.0.0 . && \
docker build --no-cache=true -f Dockerfile-foxy -t jderobot/robotics-academy:4.0.0 .
