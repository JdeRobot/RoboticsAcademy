#!/bin/sh

# Default: cpu and offline
gpu_mode="false"
base_path_offline="compose_cfg/"
compose_file="user_humble_cpu"
base_path_online="https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/humble-devel/compose_cfg/"

# Loop through the arguments using a while loop
while getopts ":g  " opt; do
  case $opt in
    g) gpu_mode="true" ;; 
    \?) echo "Invalid option: -$OPTARG" >&2 ;;   # If an invalid option is provided, print an error message
  esac
done

# Set the compose file
if [ "$gpu_mode" = "true" ]; then
  compose_file="user_humble_gpu"
fi

# Check the mode
if [ -d compose_cfg ]; then
  cp $base_path_offline$compose_file.yaml docker-compose.yaml
else
  curl -sL $base_path_online$compose_file.yaml -o docker-compose.yaml
fi

# Execute docker compose
docker-compose up; 
docker-compose down; 
rm docker-compose.yaml
