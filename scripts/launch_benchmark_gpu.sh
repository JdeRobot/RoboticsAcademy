#!/bin/bash

# Define the name of the script to be executed inside the Docker container
SCRIPT_FNAME="benchmark_gpu.sh"

# Define the Docker image to use
DOCKER_IMAGE="jderobot/robotics-backend:latest"

# Check if nvidia-smi is available and set the GPUS_ARG variable accordingly
# GPUS_ARG will be "--gpus all" if nvidia-smi works, otherwise it will be empty
GPUS_ARG=$(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "")

# Allow local X11 connections
xhost +local:

# Run the Docker container with the specified options
docker run $GPUS_ARG \
  --rm \
  -e DISPLAY=$DISPLAY \
  --device /dev/dri:/dev/dri \
  --net host \
  -v ./$SCRIPT_FNAME:/$SCRIPT_FNAME \
  --entrypoint sh \
  -it $DOCKER_IMAGE ./$SCRIPT_FNAME

# Revoke the permission for local X11 connections
xhost -local: