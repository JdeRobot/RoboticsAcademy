#!/bin/bash

docker build -f Dockerfile.base -t jderobot/base:3.2 .
docker build --no-cache=true -t jderobot/robotics-academy:$1 .