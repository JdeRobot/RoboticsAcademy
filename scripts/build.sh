#!/usr/bin/env bash

docker build -f Dockerfile-foxy.base -t foxy-radi-base . && \
docker build --no-cache=true -f Dockerfile-foxy -t foxy-radi .
