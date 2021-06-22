#!/bin/bash

docker build -f Dockerfile-2.4.base -t jderobot/base:2.4 .
docker build --no-cache=true -f Dockerfile-2.4 -t jderobot/robotics-academy:$1 .