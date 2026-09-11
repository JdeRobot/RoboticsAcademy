#!/bin/bash

# Default values if not specified
ROS_DISTRO="humble"
IMAGE_TAG="test"
FORCE_BUILD=false
FORCE_BUILD_NO_CACHE=false

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_ACADEMY_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOCAL_INFRA_DIR="$LOCAL_ACADEMY_DIR/RoboticsInfrastructure"

Help()
{
   echo "Syntax: build-offline.sh [options]"
   echo
   echo "Builds the RoboticsBackend image from the local checkouts on disk:"
   echo "  RoboticsAcademy:        $LOCAL_ACADEMY_DIR"
   echo "  RoboticsInfrastructure: $LOCAL_INFRA_DIR"
   echo "Includes uncommitted edits to files git already tracks; a brand new"
   echo "file needs 'git add' first (no commit needed) to be picked up."
   echo
   echo "Options:"
   echo "  -h                  Print this Help."
   echo "  -f                  Force creation of the base image. If omitted, the base image is created only if it doesn't exist."
   echo "  -F                  Force creation of the base image without using docker cache."
   echo "  -r, --ros <value>   ROS Distro (humble).       Default: humble"
   echo "  -t, --tag <value>   Tag name of the image.     Default: test"
   echo
   echo "Example:"
   echo "   ./build-offline.sh -t my_image"
   echo
}

stage_local_repo()
{
    local repo_dir="$1"
    local label="$2"

    if [ ! -d "$repo_dir/.git" ]; then
        echo "Error: $label ($repo_dir) is not a git checkout, can't build offline from it." >&2
        exit 1
    fi

    local stage_dir
    stage_dir="$(mktemp -d)"
    STAGE_DIRS+=("$stage_dir")
    git -C "$repo_dir" ls-files -z | tar -cf - --null -C "$repo_dir" -T - | tar -xf - -C "$stage_dir"
    echo "$stage_dir"
}

STAGE_DIRS=()
cleanup_stage_dirs()
{
    for d in "${STAGE_DIRS[@]}"; do
        rm -rf "$d"
    done
}
trap cleanup_stage_dirs EXIT

while [[ $# -gt 0 ]]; do
    case "$1" in
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
        -F | --force-no-cache)
            FORCE_BUILD_NO_CACHE=true
            shift
            ;;
        -h | --help)
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

echo "ROBOTICS_ACADEMY:-------------:local checkout ($LOCAL_ACADEMY_DIR)"
echo "ROBOTICS_INFRASTRUCTURE:------:local checkout ($LOCAL_INFRA_DIR)"
echo "ROS_DISTRO:-------------------:$ROS_DISTRO"
echo "IMAGE_TAG:--------------------:$IMAGE_TAG"
echo

if [[ $ROS_DISTRO == "humble" ]]; then
    DOCKERFILE_BASE="Dockerfile.dependencies_humble"
    DOCKERFILE="Dockerfile.humble.offline"
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

echo "===================== BUILDING $ROS_DISTRO RoboticsBackend (offline) ====================="
echo "Building RoboticsBackend using $DOCKERFILE for ROS $ROS_DISTRO"

academy_stage_dir="$(stage_local_repo "$LOCAL_ACADEMY_DIR" "RoboticsAcademy")"
infra_stage_dir="$(stage_local_repo "$LOCAL_INFRA_DIR" "RoboticsInfrastructure")"

docker build --no-cache -f $DOCKERFILE \
  --build-arg IMAGE_TAG=$IMAGE_TAG \
  --build-context local-academy="$academy_stage_dir" \
  --build-context local-infra="$infra_stage_dir" \
  -t jderobot/robotics-academy:$IMAGE_TAG .
