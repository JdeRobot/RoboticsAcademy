#!/bin/bash

# Function to clean up the containers
cleanup() {
  echo "Cleaning up..."
  docker compose down
  rm docker-compose.yaml
  
  exit 0
}

trap 'cleanup' INT

cp compose_cfg/docker-compose.yaml docker-compose.yaml

# Proceed with docker-compose commands
docker compose up --build --force-recreate