#!/bin/sh

# Initialize variables with default values
ram_version="https://github.com/JdeRobot/RoboticsApplicationManager.git"
branch="humble-devel"
radi_version="humble"
gpu_mode="false"
nvidia="false"
compose_file="dev_humble_cpu"
container_name="latest"

# Function to display help message
show_help() {
  echo "Options:"
  echo "  -r  Specify the RAM version repository URL (default: https://github.com/JdeRobot/RoboticsApplicationManager.git)"
  echo "  -b  Specify the branch of RAM (default: humble-devel)"
  echo "  -i  Specify the ROS2 version (default: humble)"
  echo "  -g  Enable GPU mode (default: false)"
  echo "  -n  Enable Nvidia support (default: false)"
  echo "  -t  Specify the container image (default: latest)"
  echo "  -h  Display this help message"
}

while getopts ":r:b:i:g:n:t:h" opt; do
  case $opt in
    r) ram_version="$OPTARG" ;;
    b) branch="$OPTARG" ;;
    i) radi_version="$OPTARG" ;; 
    g) gpu_mode="true" ;; 
    n) nvidia="true" ;;
    t) container_name="$OPTARG" ;;
    h) show_help; exit 0 ;;  # Display help message and exit
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Check if a container with the name is running or exists and remove it if necessary
running_container=$(docker ps -a --filter "ancestor=jderobot/robotics-backend:$container_name" --format "{{.ID}}")
if [ -n "$running_container" ]; then
  echo "Removing existing container(s)..."
  docker rm $running_container
fi

echo "RAM src: $ram_version"
echo "RAM branch: $branch"
echo "RoboticsBackend version: $radi_version"

# Install docker-compose if not installed
if ! command -v docker-compose &> /dev/null; then
  sudo apt install docker-compose
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

# Prepare the frontend
nvm install 17
nvm use 17
cd react_frontend/
yarn install
yarn build
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
docker compose down;
rm docker-compose.yaml
