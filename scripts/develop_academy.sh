#!/bin/bash

# Initialize variables with default values
ram_version="https://github.com/JdeRobot/RoboticsApplicationManager.git"
branch="humble-devel"
radi_version="humble"
gpu_mode="false"
nvidia="false"
mac="false"
compose_file="dev_humble_cpu"

# Function to display help message
show_help() {
  echo "Options:"
  echo "  -r  Specify the RAM version repository URL (default: https://github.com/JdeRobot/RoboticsApplicationManager.git)"
  echo "  -b  Specify the branch of RAM (default: humble-devel)"
  echo "  -i  Specify the ROS2 version (default: humble)"
  echo "  -g  Enable GPU mode (default: false)"
  echo "  -n  Enable Nvidia support (default: false)"
  echo "  -m  Enable Apple Silicon Mac mode with Rosetta x86_64 emulation (default: false)"
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
  rm react_frontend/checksum.txt
  
  exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -r) 
            ram_version="$2"
            shift 2
            ;;
        -b)
            branch="$2"
            shift 2
            ;;
        -i)
            radi_version="$2"
            shift 2
            ;;
        -g)
            gpu_mode="true"
            shift 1
            ;;
        -n)
            nvidia="true"
            shift 1
            ;;
        -m)
            mac="true"
            shift 1
            ;;
        -h | --help) # display Help
            show_help
            exit 0
            ;;
        *)
            echo "Invalid Option: $1"
            Help
            exit 1
            ;;
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
if ! command -v yarn &> /dev/null; then
  echo "Yarn is not installed. Installing Yarn..."
  
  # Check if npm exists or not
  if command -v npm &> /dev/null; then
    npm install --global yarn
  else
    echo "npm is not installed. Installing Node.js and npm first..."
    
    # Detect OS and install npm and node.js accordingly
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
      curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
      sudo apt-get install -y nodejs
    elif [[ "$OSTYPE" == "darwin"* ]]; then
      if command -v brew &> /dev/null; then
        brew install node
      else
        echo "Homebrew not found. Please install Yarn manually: https://yarnpkg.com/getting-started/install"
        exit 1
      fi
    else
      echo "Unsupported OS. Please install Yarn manually: https://yarnpkg.com/getting-started/install"
      exit 1
    fi
    
    npm install --global yarn
  fi
  echo "Yarn installed successfully."
else
  echo "Yarn is already installed."
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
nvm install 20
nvm use 20

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
if [ "$mac" = "true" ]; then
  compose_file="dev_humble_cpu_mac"
fi
cp compose_cfg/$compose_file.yaml docker-compose.yaml

# Proceed with docker-compose commands
if [ "$nvidia" = "true" ]; then
  docker compose --compatibility up
else
  docker compose up
fi 

cleanup
