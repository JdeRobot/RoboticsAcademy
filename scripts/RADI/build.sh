#!/bin/bash

# Default branch if not specified
ROBOTICS_ACADEMY="humble-devel"
ROBOTICS_INFRASTRUCTURE="humble-devel"
RAM="humble-devel"
ROS_DISTRO="humble"
IMAGE_TAG="test"
FORCE_BUILD=false
FORCE_BUILD_NO_CACHE=false

# Default GitHub owners
ACADEMY_OWNER="JdeRobot"
INFRA_OWNER="JdeRobot"

Help()
{
   echo "Syntax: build.sh [options]"
   echo "Options:"
   echo "  -h                           Print this Help."
   echo "  -f                           Force creation of the base image. If omitted, the base image is created only if it doesn't exist."
   echo "  -F                           Force creation of the base image without using docker cache."
   echo "  -a, --academy       <value>  Branch of RoboticsAcademy.               Default: humble-devel"
   echo "  -i, --infra         <value>  Branch of RoboticsInfrastructure.        Default: humble-devel"
   echo "  -m, --ram           <value>  Branch of RoboticsApplicationManager.    Default: humble-devel"
   echo "  -r, --ros           <value>  ROS Distro (humble).                     Default: humble"
   echo "  -t, --tag           <value>  Tag name of the image.                   Default: test"
   echo "  --academy-owner     <value>  GitHub owner for RoboticsAcademy.        Default: JdeRobot"
   echo "  --infra-owner       <value>  GitHub owner for RoboticsInfrastructure. Default: JdeRobot"
   echo
   echo "Example:"
   echo "   ./build.sh -t my_image"
   echo "   ./build.sh --academy-owner aquintan4 -a issue-3476 -t my_image"
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
        --academy-owner)
            ACADEMY_OWNER="$2"
            shift 2
            ;;
        --infra-owner)
            INFRA_OWNER="$2"
            shift 2
            ;;
        -f | --force)
            FORCE_BUILD=true
            shift
            ;;
        -F | --force-no-cache)
            FORCE_BUILD_NO_CACHE=true
            shift
            ;;
        -h | --help)
            echo "Generates RoboticsAcademy RoboticsBackend image"
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

echo "ROBOTICS_ACADEMY:-------------:$ROBOTICS_ACADEMY (Owner: $ACADEMY_OWNER)"
echo "ROBOTICS_INFRASTRUCTURE:------:$ROBOTICS_INFRASTRUCTURE (Owner: $INFRA_OWNER)"
echo "RAM:--------------------------:$RAM"
echo "ROS_DISTRO:-------------------:$ROS_DISTRO"
echo "IMAGE_TAG:--------------------:$IMAGE_TAG"
echo

if [[ $ROS_DISTRO == "humble" ]]; then
    DOCKERFILE_BASE="Dockerfile.dependencies_humble"
    DOCKERFILE="Dockerfile.humble"
else
    echo "Error: Unknown ROS_DISTRO ($ROS_DISTRO). Please set it to 'humble'."
    exit 1
fi

if $FORCE_BUILD_NO_CACHE; then
  NO_CACHE="--no-cache"
else
  NO_CACHE=""
fi

if $FORCE_BUILD_NO_CACHE || $FORCE_BUILD || [[ "$(docker images -q jderobot/robotics-applications:dependencies-$ROS_DISTRO 2> /dev/null)" == "" ]]; then
  echo "===================== BUILDING $ROS_DISTRO BASE IMAGE ====================="
  echo "Building base using $DOCKERFILE_BASE for ROS $ROS_DISTRO"
  docker build $NO_CACHE -f $DOCKERFILE_BASE \
    --build-arg TARGETARCH=$(uname -m | sed 's/x86_64/amd64/;s/arm64/arm64/') \
    -t jderobot/robotics-applications:dependencies-$ROS_DISTRO .
fi

if [ $? -eq 0 ]; then
    echo "Docker Base Image Build Successful"
else
    echo "Docker Base Image Build FAILED...exiting"
    exit
fi

echo "===================== BUILDING $ROS_DISTRO RoboticsBackend ====================="
echo "Building RoboticsBackend using $DOCKERFILE for ROS $ROS_DISTRO"

docker build --no-cache -f $DOCKERFILE \
  --build-arg ROBOTICS_ACADEMY=$ROBOTICS_ACADEMY \
  --build-arg ROBOTICS_INFRASTRUCTURE=$ROBOTICS_INFRASTRUCTURE \
  --build-arg RAM=$RAM \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  --build-arg IMAGE_TAG=$IMAGE_TAG \
  --build-arg ACADEMY_OWNER=$ACADEMY_OWNER \
  --build-arg INFRA_OWNER=$INFRA_OWNER \
  -t jderobot/robotics-academy:$IMAGE_TAG .