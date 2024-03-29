#!/bin/sh

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

echo $new_checksum

existing_checksum_file="$DIRECTORY_TO_MONITOR/checksum.txt"
echo $existing_checksum_file

if [ -f "$existing_checksum_file" ]; then
    existing_checksum=$(cat "$existing_checksum_file")
    echo $existing_checksum
    if [ "$existing_checksum" != "$new_checksum" ]; then
        echo "$new_checksum" > "$existing_checksum_file"
        if command -v npm &> /dev/null; then
            npm install --force
            npm run dev & 
            sleep 10

        else
            yarn install 
            yarn dev run &
            sleep 10
        fi
    else
        echo "No Compilation needed"
    fi
else
    echo "$new_checksum" > "$existing_checksum_file"
    npm install --force
    npm run dev & 
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
docker-compose up
docker-compose down
rm docker-compose.yaml
