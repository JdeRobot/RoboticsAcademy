#!/bin/bash

docker build -f Dockerfile.base -t jderobot/base .
docker build --no-cache=true -t jderobot/robotics-academy:$1 .