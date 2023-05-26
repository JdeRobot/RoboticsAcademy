#!/bin/bash

# Default branch if not specified
ROBOTICS_ACADEMY=${1:-master}
ROBOTICS_INFRASTRUCTURE=${2:-noetic-devel}
RAM=${3:-main}
ROS_DISTRO=${4:-noetic}
IMAGE_TAG=${5:-test}

# Determine Dockerfile based on ROS_DISTRO
if [[ $ROS_DISTRO == "noetic" ]]; then
    DOCKERFILE="Dockerfile.mini_noetic"
    echo "Building using $DOCKERFILE for ROS Noetic"
elif [[ $ROS_DISTRO == "humble" ]]; then
    DOCKERFILE="Dockerfile.mini_humble"
    echo "Building using $DOCKERFILE for ROS Humble"
else
    echo "Error: Unknown ROS_DISTRO ($ROS_DISTRO). Please set it to 'noetic' or 'humble'."
    exit 1
fi

# Build the Docker image
docker build --no-cache -f $DOCKERFILE \
  --build-arg ROBOTICS_ACADEMY=$ROBOTICS_ACADEMY \
  --build-arg ROBOTICS_INFRASTRUCTURE=$ROBOTICS_INFRASTRUCTURE \
  --build-arg RAM=$RAM \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  -t jderobot/robotics-academy:$IMAGE_TAG .
