#!/bin/bash

# Default branch if not specified
ROBOTICS_ACADEMY="master"
ROBOTICS_INFRASTRUCTURE="noetic-devel"
RAM="main"
ROS_DISTRO="noetic"
IMAGE_TAG="test"
FORCE_BUILD=false

Help()
{
   # Display Help
   echo "Syntax: build.sh [options]"
   echo "Options:"
   echo "  -h                        Print this Help."
   echo "  -f                        Force creation of the base image. If omitted, the base image is created only if "
   echo "                            it doesn't exist."
   echo "  -a, --academy    <value>  Branch of RoboticsAcademy.               Default: master"
   echo "  -i, --infra      <value>  Branch of RoboticsInfrastructure.        Default: noetic-devel"
   echo "  -m, --ram        <value>  Branch of RoboticsApplicationManager.    Default: main"
   echo "  -r, --ros        <value>  ROS Distro (humble or noetic).           Default: noetic"
   echo "  -t, --tag        <value>  Tag name of the image.                   Default: test"
   echo
   echo "Example:"
   echo "   ./build.sh -t my_image"
   echo "   ./build.sh -f -a master -i noetic-devel -m main -r noetic -t my_image" 
   echo "   ./build.sh -f --academy master --infra noetic-devel --ram main --ros noetic --tag my_image" 
   echo
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -a | --academy) 
            ROBOTICS_ACADEMY="$2"
            shift 2
            ;;
        -i | --infra)
            ROBOTICS_INFRASTRUCTURE="$2"
            shift 2
            ;;
        -m | --ram)
            RAM="$2"
            shift 2
            ;;
        -r | --ros)
            ROS_DISTRO="$2"
            shift 2
            ;;
        -t | --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        -f | --force)
            FORCE_BUILD=true
            shift
            ;;
        -h | --help) # display Help
            echo "Generates RoboticsAcademy RADI image"
            echo
            Help
            exit 0
            ;;
        *)
            echo "Invalid Option: $1"
            Help
            exit 1
            ;;
   esac
done

echo "ROBOTICS_ACADEMY:-------------:$ROBOTICS_ACADEMY"
echo "ROBOTICS_INFRASTRUCTURE:------:$ROBOTICS_INFRASTRUCTURE"
echo "RAM:--------------------------:$RAM"
echo "ROS_DISTRO:-------------------:$ROS_DISTRO"
echo "IMAGE_TAG:--------------------:$IMAGE_TAG"
echo

# Determine Dockerfile based on ROS_DISTRO
if [[ $ROS_DISTRO == "noetic" ]]; then
    DOCKERFILE_BASE="Dockerfile.mini_noetic_base"
    DOCKERFILE="Dockerfile.mini_noetic"
elif [[ $ROS_DISTRO == "humble" ]]; then
    DOCKERFILE_BASE="Dockerfile.mini_humble_base"
    DOCKERFILE="Dockerfile.mini_humble"
else
    echo "Error: Unknown ROS_DISTRO ($ROS_DISTRO). Please set it to 'noetic' or 'humble'."
    exit 1
fi

# Build the Docker Base image
if $FORCE_BUILD || [[ "$(docker images -q jderobot/robotics-applications:mini_$ROS_DISTRO-base 2> /dev/null)" == "" ]]; then
  echo "===================== BUILDING $ROS_DISTRO BASE IMAGE ====================="
  echo "Building base using $DOCKERFILE_BASE for ROS $ROS_DISTRO"
  docker build -f $DOCKERFILE_BASE -t jderobot/robotics-applications:mini_$ROS_DISTRO-base .
fi

# Build the Docker image
echo "===================== BUILDING $ROS_DISTRO RADI ====================="
echo "Building RADI using $DOCKERFILE for ROS $ROS_DISTRO"

docker build --no-cache -f $DOCKERFILE \
  --build-arg ROBOTICS_ACADEMY=$ROBOTICS_ACADEMY \
  --build-arg ROBOTICS_INFRASTRUCTURE=$ROBOTICS_INFRASTRUCTURE \
  --build-arg RAM=$RAM \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  -t jderobot/robotics-academy:$IMAGE_TAG .
