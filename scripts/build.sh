#!/bin/bash

docker build -f Dockerfile.base -t jderobot/base .
docker build -t jderobot/robotics-academy:$1