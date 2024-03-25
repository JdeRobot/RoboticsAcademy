#!/bin/sh

# Initialize variables with default values
version="https://github.com/JdeRobot/RoboticsApplicationManager.git"
branch=main

# Loop through the arguments using a while loop
while getopts "rb" opt; do
  case $opt in
    l) version="$OPTARG" ;;   # If -l option is provided, set list to true
    b) branch="$OPTARG" ;;   # If -b option is provided, set boolean to true
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

echo "RAM src: $version"
echo "RAM branch: $branch"

# Install docker-compose if not installed
if ! command -v docker-compose &> /dev/null; then
  sudo apt install docker-compose
fi

# Clone the desired RAM fork and branch
if ! [ -d src ]; then
  git clone $version -b $branch src;
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

# Prepare the frontend
nvm install 16
nvm use 16
cd react_frontend/
yarn install
yarn build
cd ..

# Proceed with docker-compose commands
docker-compose up; 
docker-compose down
