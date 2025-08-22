#!/bin/bash

# Initialize variables with default values
ram_version="https://github.com/JdeRobot/RoboticsApplicationManager.git"
branch="humble-devel"
radi_version="humble"
gpu_mode="false"
nvidia="false"
compose_file="dev_humble_cpu"

# Function to display help message
show_help() {
  echo "Options:"
  echo "  -r  Specify the RAM version repository URL (default: https://github.com/JdeRobot/RoboticsApplicationManager.git)"
  echo "  -b  Specify the branch of RAM (default: humble-devel)"
  echo "  -i  Specify the ROS2 version (default: humble)"
  echo "  -g  Enable GPU mode (default: false)"
  echo "  -n  Enable Nvidia support (default: false)"
  echo "  -h  Display this help message"
}

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  if [ "$nvidia" = "true" ]; then
    docker compose --compatibility down
  else
    docker compose down
  fi
  rm docker-compose.yaml
  
  exit 0
}

while getopts ":r:b:i:g:n:t:h" opt; do
  case $opt in
    r) ram_version="$OPTARG" ;;
    b) branch="$OPTARG" ;;
    i) radi_version="$OPTARG" ;; 
    g) gpu_mode="true" ;; 
    n) nvidia="true" ;;
    h) show_help; exit 0 ;;  # Display help message and exit
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Set up trap to catch interrupt signal (Ctrl+C) and execute cleanup function
trap 'cleanup' INT

echo "RAM src: $ram_version"
echo "RAM branch: $branch"
echo "RoboticsBackend version: $radi_version"

# Check docker compose installation
if ! command -v docker compose &> /dev/null; then
  echo "Docker Compose V2 is not installed. Please install it."
fi

# Clone the desired RAM fork and branch
if ! [ -d src ]; then
  git clone $ram_version -b $branch src;
  chown -R $(id -u):$(id -g) src/
fi

# Prepare nvm
export NVM_DIR=$HOME/.nvm;
source $NVM_DIR/nvm.sh;
if ! command -v nvm &> /dev/null; then
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
  export NVM_DIR=$HOME/.nvm;
  source $NVM_DIR/nvm.sh;
fi

# Prepare yarn 
if ! command -v yarn --version &> /dev/null; then
  npm install --global yarn
fi

# Prepare the commons zip file
cd common
cd console_interfaces
zip -r ../common.zip console_interfaces/
cd ..
cd gui_interfaces
zip -r -u ../common.zip gui_interfaces/
cd ..
cd hal_interfaces
zip -r -u ../common.zip hal_interfaces/
cd ../..
mv common/common.zip react_frontend/src/common.zip

# Prepare the frontend
nvm install 17
nvm use 17

# Checking if the frontend needs compilation
cd react_frontend/
DIRECTORY_TO_MONITOR="."

new_checksum=$(find "$DIRECTORY_TO_MONITOR" \( -path "*/node_modules" -o \
            -path "*/__pycache__" -o \
            -path "*/migrations" -o \
            -name "yarn.lock" -o \
            -name "checksum.txt" \) -prune \
            -o -type f -exec md5sum {} + | \
            sort | \
            md5sum | \
            awk '{print $1}')

existing_checksum_file="$DIRECTORY_TO_MONITOR/checksum.txt"

if [ -f "$existing_checksum_file" ]; then
    existing_checksum=$(cat "$existing_checksum_file")
    if [ "$existing_checksum" != "$new_checksum" ]; then
        echo "$new_checksum" > "$existing_checksum_file"
        yarn install 
        yarn dev &
        sleep 10
    else
        echo "No Compilation needed"
    fi
else
    echo "$new_checksum" > "$existing_checksum_file"
    yarn install 
    yarn dev &
    sleep 10
fi

cd ..

# Prepare the compose file
if [ "$gpu_mode" = "true" ]; then
  compose_file="dev_humble_gpu"
fi
if [ "$nvidia" = "true" ]; then
  compose_file="dev_humble_nvidia"
fi
cp compose_cfg/$compose_file.yaml docker-compose.yaml

# Proceed with docker-compose commands
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi 

cleanup