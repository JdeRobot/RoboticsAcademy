#!/bin/sh

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    sudo apt install docker-compose
fi

# Proceed with docker-compose commands
docker-compose up; 
docker-compose down
