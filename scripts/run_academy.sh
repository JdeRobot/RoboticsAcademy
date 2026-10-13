#!/bin/bash
# Default: cpu and offline
gpu_mode="false"
nvidia="false"
engine="docker"
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
  echo "  -p  Use rootless Podman instead of Docker (requires -n for GPU)"
  echo "  -h  Display this help message"
  echo ""
  echo "Examples:"
  echo "  $(basename "$0")          # Run in CPU mode (default)"
  echo "  $(basename "$0") -g       # Run in GPU mode"
  echo "  $(basename "$0") -n       # Run with Nvidia support"
  echo "  $(basename "$0") -p -n    # Run with rootless Podman and Nvidia"
}

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$engine" = "podman" ]; then
    # Silence the errors for containers podman-compose already removed
    $compose_cmd down 2>/dev/null
  elif [ "$nvidia" = "true" ]; then
    $compose_cmd --compatibility down
  else
    $compose_cmd down
  fi
  if [ -f docker-compose.yaml ]; then
    rm docker-compose.yaml
  fi
  exit 0
}

# Preflight checks
preflight_checks() {
  # Podman checks: rootless, binaries and CDI spec
  if [ "$engine" = "podman" ]; then
    if [ "$(id -u)" -eq 0 ]; then
      echo "Error: Rootless Podman must be run without sudo. Execute the script as a standard user."
      exit 1
    fi

    for binary in podman podman-compose; do
      if ! command -v "$binary" &> /dev/null; then
        echo "Error: $binary is not installed or not in PATH."
        echo "See: https://podman.io/docs/installation"
        exit 1
      fi
    done

    if [ "$gpu_mode" = "true" ]; then
      echo "Error: Podman GPU acceleration requires NVIDIA CDI. Use -p -n instead of -p -g."
      exit 1
    fi

    # A CDI spec is needed to resolve the nvidia.com/gpu device
    if [ "$nvidia" = "true" ]; then
      if ! compgen -G "/var/run/cdi/*.yaml" > /dev/null && \
         ! compgen -G "/var/run/cdi/*.json" > /dev/null && \
         ! compgen -G "/etc/cdi/*.yaml"     > /dev/null && \
         ! compgen -G "/etc/cdi/*.json"     > /dev/null; then
        echo "Error: No CDI spec found in /var/run/cdi or /etc/cdi."
        echo "Generate one with: sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml"
        exit 1
      fi
    fi

    # Podman has no daemon, skip the docker checks
    return
  fi

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
while getopts ":gnhp" opt; do
  case $opt in
    g) gpu_mode="true" ;;
    n) nvidia="true" ;;
    p) engine="podman" ;;
    h) show_help; exit 0 ;;
    \?) echo "Error: Invalid option: -$OPTARG" >&2; show_help; exit 1 ;;
  esac
done

# Set the compose command based on the engine
if [ "$engine" = "podman" ]; then
  compose_cmd="podman-compose -p roboticsacademy"
else
  compose_cmd="docker compose"
fi

# Run preflight checks before doing anything else
preflight_checks

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

# Set the compose file based on flags
if [ "$gpu_mode" = "true" ]; then
  compose_file="user_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  if [ "$engine" = "podman" ]; then
    compose_file="user_humble_podman_nvidia"
  else
    compose_file="user_humble_nvidia"
  fi
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

# Execute compose (--compatibility only applies to the docker nvidia config)
if [ "$engine" = "podman" ]; then
  $compose_cmd up
elif [ "$nvidia" = "true" ]; then
  $compose_cmd --compatibility up
else
  $compose_cmd up
fi

cleanup
