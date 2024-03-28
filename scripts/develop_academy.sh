#!/bin/sh

# ANSI color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' 

# Initialize variables with default values
ram_version="https://github.com/JdeRobot/RoboticsApplicationManager.git"
branch=main
radi_version="humble"

# Loop through the arguments using a while loop
while getopts ":r:b:i:" opt; do
  case $opt in
    r) ram_version="$OPTARG" ;;
    b) branch="$OPTARG" ;;
    i) radi_version="$OPTARG" ;; 
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

echo "RAM src: $ram_version"
echo "RAM branch: $branch"
echo "RADI version: $radi_version"

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

# Prepare the frontend
nvm install 16
nvm use 16
cd react_frontend/
if command -v npm &> /dev/null; then
    npm install --force
    npm run dev & 
    sleep 10

else
    yarn install 
    yarn dev run &
    sleep 10
fi

cd ..

# Prepare the compose file
compose_file="dev_humble_cpu.yaml"
if [ $radi_version != "humble" ]; then
  compose_file="dev_noetic_cpu.yaml"
fi
cp compose_cfg/$compose_file docker-compose.yaml


# Proceed with docker-compose commands
while true; do
    echo -e "${GREEN}Enter your choice${NC}"
    echo -e "${GREEN}1.For docker-compose up : up${NC}"
    echo -e "${GREEN}2.For docker-compose down : down${NC}"
    echo -e "${GREEN}3.For remove docker-compose.yaml : rm${NC}"

    read choice 
    
    case $choice in
        up)
            docker-compose up & 
            ;;
        down)
            docker-compose down &
            ;;
        rm)
            rm docker-compose.yaml &
            echo -e "${YELLOW}Removing docker-compose.yaml and closing this
            process${NC}"
            exit 0
            ;;
    esac
    sleep 10
done
