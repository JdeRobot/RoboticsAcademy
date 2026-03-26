#!/bin/bash
# Default: cpu and offline
gpu_mode="false"
nvidia="false"
base_path_offline="compose_cfg/"
compose_file="user_humble_cpu"
base_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/compose_cfg/"

# Function to display usage information
show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  -g  Enable GPU mode (uses user_humble_gpu compose file)"
  echo "  -n  Enable Nvidia support (uses user_humble_nvidia compose file)"
  echo "  -h  Display this help message"
  echo ""
  echo "Examples:"
  echo "  $(basename "$0")          # Run in CPU mode (default)"
  echo "  $(basename "$0") -g       # Run in GPU mode"
  echo "  $(basename "$0") -n       # Run with Nvidia support"
}

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$nvidia" = "true" ]; then
    docker compose --compatibility down
  else
    docker compose down
  fi
  if [ -f docker-compose.yaml ]; then
    rm docker-compose.yaml
  fi
  exit 0
}

# Preflight checks
preflight_checks() {
  if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH. Please install Docker first."
    echo "See: https://docs.docker.com/engine/install/"
    exit 1
  fi

  if ! docker compose version &> /dev/null; then
    echo "Error: Docker Compose V2 is not installed."
    echo "See: https://docs.docker.com/compose/install/"
    exit 1
  fi

  if ! docker info &> /dev/null; then
    echo "Error: Docker daemon is not running. Please start Docker."
    exit 1
  fi
}

# Loop through the arguments
while getopts ":gnh" opt; do
  case $opt in
    g) gpu_mode="true" ;;
    n) nvidia="true" ;;
    h) show_help; exit 0 ;;
    \?) echo "Error: Invalid option: -$OPTARG" >&2; show_help; exit 1 ;;
  esac
done

# Run preflight checks before doing anything else
preflight_checks

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

# Set the compose file based on flags
if [ "$gpu_mode" = "true" ]; then
  compose_file="user_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  compose_file="user_humble_nvidia"
fi

# Check the mode (offline if compose_cfg dir exists, online otherwise)
if [ -d compose_cfg ]; then
  # Offline mode
  cp "$base_path_offline$compose_file.yaml" docker-compose.yaml
else
  # Online mode: fetch compose file and verify it was downloaded correctly
  echo "compose_cfg/ not found, fetching compose file from upstream..."
  curl -sL "$base_path_online$compose_file.yaml" -o docker-compose.yaml

  if [ $? -ne 0 ] || [ ! -s docker-compose.yaml ]; then
    echo "Error: Failed to download compose file from:"
    echo "  $base_path_online$compose_file.yaml"
    echo "Check your internet connection or run from the repo root directory."
    rm -f docker-compose.yaml
    exit 1
  fi
fi

# Execute docker compose
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi

cleanup
