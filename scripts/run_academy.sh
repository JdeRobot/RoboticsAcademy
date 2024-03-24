#!/bin/sh

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    sudo apt install docker-compose
fi

curl -sL https://raw.githubusercontent.com/JdeRobot/RoboticsAcademy/master/docker-compose-cpu.yaml -o docker-compose.yml && docker-compose up; docker-compose down; rm docker-compose.yml
